# -*- coding: utf-8 -*-
"""

@author: Atul Kumar
"""

import cv2
import numpy as np
import time
import os
from gpiozero import Motor,Device,DistanceSensor
from gpiozero.pins.pigpio import PiGPIOFactory
from picamera2 import Picamera2

# 1. HARDWARE SETUP
try:
    Device.pin_factory=PiGPIOFactory()
except:
    print("[WARN] pigpiod not running. Run 'sudo systemctl start pigpiod'")
LEFT_FWD,LEFT_BWD,LEFT_EN=17,18,27
RIGHT_FWD,RIGHT_BWD,RIGHT_EN=22,23,24
US_TRIG,US_ECHO=5,6
left_motor=Motor(forward=LEFT_FWD,backward=LEFT_BWD,enable=LEFT_EN)
right_motor=Motor(forward=RIGHT_FWD,backward=RIGHT_BWD,enable=RIGHT_EN)
us=DistanceSensor(echo=US_ECHO,trigger=US_TRIG)
def set_motor_speeds(left,right):
    right=-right # Invert Right Motor
    left=max(-1.0,min(1.0,left))
    right=max(-1.0,min(1.0,right))
    left_motor.value=left
    right_motor.value=right
def stop():
    left_motor.value=0
    right_motor.value=0

# 2. VISION & TUNING
FRAME_WIDTH=320
FRAME_HEIGHT=240
CENTER_X=FRAME_WIDTH//2
TEMPLATE_PATH="targets/box.jpg"
MATCH_THRESHOLD=0.50
Kp=0.5 # Steering Sensitivity
FWD_SPEED=0.45 # Base Speed
clahe_tool=cv2.createCLAHE(clipLimit=4.0,tileGridSize=(4,4))

# 3. PRE-PROCESSING
if not os.path.exists(TEMPLATE_PATH):
    print("[ERROR] box.jpg not found!")
    exit()
raw_template=cv2.imread(TEMPLATE_PATH,0)
base_template=cv2.resize(raw_template,(50,50))
t_h,t_w=base_template.shape
print("[INFO] Robot Ready. Adaptive + Multi-Scale Mode.")

# 4. MAIN LOOP
picam2=Picamera2()
config=picam2.create_preview_configuration(main={"format":'XRGB8888',"size":(FRAME_WIDTH,FRAME_HEIGHT)})
picam2.configure(config)
picam2.start()
time.sleep(2)
try:
    while True:
        dist=us.distance
        if 0<dist<0.25:
            print(f"[SAFETY] Stop! ({dist*100:.1f}cm)")
            stop()
            time.sleep(0.05)
            continue
        frame=picam2.capture_array()
        frame_bgr=cv2.cvtColor(frame,cv2.COLOR_RGBA2BGR)
        gray_frame=cv2.cvtColor(frame_bgr,cv2.COLOR_BGR2GRAY)
        avg_brightness=np.mean(gray_frame)
        if avg_brightness<85:
            processed_frame=clahe_tool.apply(gray_frame)
            mode="DARK"
        else:
            processed_frame=gray_frame
            mode="BRIGHT"
        found=None
        for scale in [0.5,1.0,1.5]:
            new_w=int(t_w*scale)
            new_h=int(t_h*scale)
            if mode=="DARK":
                resized_template=cv2.resize(clahe_tool.apply(raw_template),(new_w,new_h))
            else:
                resized_template=cv2.resize(raw_template,(new_w,new_h))
            if new_h>FRAME_HEIGHT or new_w>FRAME_WIDTH:continue
            res=cv2.matchTemplate(processed_frame,resized_template,cv2.TM_CCOEFF_NORMED)
            min_val,max_val,min_loc,max_loc=cv2.minMaxLoc(res)
            if found is None or max_val>found[0]:
                found=(max_val,max_loc,new_w)
        max_val,max_loc,final_w=found
        if max_val>MATCH_THRESHOLD:
            x,y=max_loc
            cx=x+(final_w//2)
            if max_val>0.85:
                print(f"[STOP] Reached Target! ({max_val:.2f})")
                stop()
            else:
                error=(cx-CENTER_X)/(FRAME_WIDTH/2)
                turn=error*Kp
                current_base=FWD_SPEED*(1-abs(error))
                left=current_base+turn
                right=current_base-turn
                print(f"[{mode}] Sc:{final_w} | Err:{error:.2f} | Val:{max_val:.2f}",end='\r')
                set_motor_speeds(left,right)
        else:
            print(f"[WAIT] Waiting... ({max_val:.2f})",end='\r')
            stop()

except KeyboardInterrupt:
    print("\nStopped")
finally:
    stop()
    picam2.stop()
