# Tracker 1
# combining Grasshopper 3 camera and motors to move the camera to track
# jelly fish (and other things - like light)
# Brennan Hoppa

# imports ---------------------
import PySpin # type: ignore
import sys
import numpy as np
import matplotlib.pyplot as plt # type: ignore
import cv2 # type: ignore
import time
from gpiozero import LED, Button, Device
import pygame

# variable setup 
greenled = LED(4)
switch = Button(17)

dirUD = LED(23)
pulseUD = LED(18)
onoffUD = LED(24) # if true/high, motor is unpower, can move with hand

dirLR = LED(25)
pulseLR = LED(12)
onoffLR = LED(16) # if true/high, motor is unpower, can move with hand

screen_width = 800
screen_height = 600

baseStepsPerRevolution = 200 # step angle is 1.8 deg, 360/18 = 200
microsteps = 1 # adjust off of switches
stepsPerRevolution = baseStepsPerRevolution * microsteps

downpath = "/home/user/Documents/arrows/down.png"
downleftpath = "/home/user/Documents/arrows/downleft.png"
leftpath = "/home/user/Documents/arrows/left.png"
upleftpath = "/home/user/Documents/arrows/upleft.png"
uppath = "/home/user/Documents/arrows/up.png"
uprightpath = "/home/user/Documents/arrows/upright.png"
rightpath = "/home/user/Documents/arrows/right.png"
downrightpath = "/home/user/Documents/arrows/downright.png"

idown =pygame.image.load(downpath)
idownleft =pygame.image.load(downleftpath)
ileft =pygame.image.load(leftpath)
iupleft =pygame.image.load(upleftpath)
iup = pygame.image.load(uppath)
iupright =pygame.image.load(uprightpath)
iright =pygame.image.load(rightpath)
idownright =pygame.image.load(downrightpath)

arrows = [idown,idownleft,ileft,iupleft,iup,iupright,iright,idownright]
rects = []
for i,a in enumerate(arrows):
    w,h = a.get_size()
    sf = min(screen_width / w, screen_height / h)
    nw = int(w*sf)
    nh = int(h*sf)
    na = pygame.transform.scale(a,(nw,nh))
    ir = na.get_rect()
    ir.center = (screen_width//2, screen_height//2)
    arrows[i] = na
    rects.append(ir)

def set_direction(instruction, dir):
    if instruction == "forward":
        dir.on()
    else:
        dir.off()

def enable_driver(enable_motor, enabler):
    if enable_motor:
        enabler.off()
    else:
        enabler.on()

def move_motor(steps, stepper, instruction, dir):
    set_direction(instruction,dir)
    for _ in range(steps):
        # we are going to start with this constant movement speed, pretend I only have this speed available
        # control motor movement simply by number of steps which corresponds to distance
        # for speed - motor has pps 5000 - so the sum of the two sleeps below has to be greater than 0.0002
        stepper.on()
        # 0.0005 is good
        # at 0.001 for now
        time.sleep(0.001) # adjust for speed - min 0.0001
        stepper.off()
        time.sleep(0.001) # adjust - min 0.0001

def move_2_motors(steps, stepper1, stepper2, instruction1, instruction2, dirLR, dirUD):
    set_direction(instruction1,dirLR)
    set_direction(instruction2,dirUD)
    for _ in range(steps):
        stepper1.on()
        stepper2.on()
        time.sleep(0.001)
        stepper1.off()
        stepper2.off()
        time.sleep(0.001)

# EVERYTHING ABOVE I HAVE

def get_dir(keys):
    l = 1 if keys[pygame.K_LEFT] else 0
    r = 1 if keys[pygame.K_RIGHT] else 0
    u = 1 if keys[pygame.K_UP] else 0
    d = 1 if keys[pygame.K_DOWN] else 0
    rl = 0
    if r and not l:
        rl = 1
    elif not r and l:
        rl = -1
    ud = 0
    if u and not d:
        ud = 1
    elif not u and d:
        ud = -1
    return (rl,ud)
        
def motor_moves():
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            print("trying to quit")
        elif event.type == pygame.KEYDOWN:
            pass
            # global onoffUDb, onoffLRb
            # if event.key == pygame.K_1:
                
                
                
            #     if onoffUDb:
            #         enable_driver(False,onoffLR)
            #         onoffUDb = 0
            #     else:
            #         enable_driver(True,onoffLR)
            #         onoffUDb = 1
            # if event.key == pygame.K_2:
            #     if onoffLRb:
            #         enable_driver(False,onoffUD)
            #         onoffLRb = 0
            #     else:
            #         enable_driver(True,onoffUD)
            #         onoffLRb = 1
    keys = pygame.key.get_pressed()

    movement_dir = get_dir(keys)
    steps = 50

    if movement_dir[0] == -1:
        if movement_dir[1] == -1:
            # down left
            move_2_motors(steps,pulseLR,pulseUD,'backward','backward',dirLR,dirUD)
            screen.fill((10,20,40))
            screen.blit(arrows[1],rects[1])
            pygame.display.flip()
        elif movement_dir[1] == 1:
            # up left
            move_2_motors(steps,pulseLR,pulseUD,'forward','backward',dirLR,dirUD)
            screen.fill((10,20,40))
            screen.blit(arrows[3],rects[3])
            pygame.display.flip()
        else:
            # left
            move_motor(steps,pulseUD,'backward',dirUD)
            screen.fill((10,20,40))
            screen.blit(arrows[2],rects[2])
            pygame.display.flip()
    elif movement_dir[0] == 1:
        if movement_dir[1] == -1:
            # down right
            move_2_motors(steps,pulseLR,pulseUD,'backward','forward',dirLR,dirUD)
            screen.fill((10,20,40))
            screen.blit(arrows[7],rects[7])
            pygame.display.flip()
        elif movement_dir[1] == 1:
            # up right
            move_2_motors(steps,pulseLR,pulseUD,'forward','forward',dirLR,dirUD)
            screen.fill((10,20,40))
            screen.blit(arrows[5],rects[5])
            pygame.display.flip()
        else:
            # right
            move_motor(steps,pulseUD,'forward',dirUD)
            screen.fill((10,20,40))
            screen.blit(arrows[6],rects[6])
            pygame.display.flip()
    else:
        if movement_dir[1] == -1:
            # down
            move_motor(steps,pulseLR,'backward',dirLR)
            screen.fill((10,20,40))
            screen.blit(arrows[0],rects[0])
            pygame.display.flip()
        elif movement_dir[1] == 1:
            # up
            move_motor(steps,pulseLR,'forward',dirLR)
            screen.fill((10,20,40))
            screen.blit(arrows[4],rects[4])
            pygame.display.flip()
        else:
            # nothing
            screen.fill((10,20,40))
            pygame.display.flip()


    # move_motor(1000, 'forward')
    # time.sleep(1)
    # move_motor(200, 'backward')
    # time.sleep(1)

def main():
    try:
        enable_driver(True, onoffLR)
        enable_driver(True, onoffUD)
        while True:
            motor_moves()
    except KeyboardInterrupt:
        print("Stopping the motor")
    finally:
        enable_driver(False, onoffLR) 
        enable_driver(False, onoffUD)

if __name__ == '__main__':
    pygame.init()

    greenled.on()
    
    screen = pygame.display.set_mode((screen_width,screen_height))
    pygame.display.set_caption("arrow detector")
    main()
    greenled.off()
    print("LED OFF, everything done")
    pygame.quit()
    sys.exit()


    # print("a")
    # if switch.is_pressed:
    #     print('x')
    #     led.on()
    # else:
    #     led.off()
    