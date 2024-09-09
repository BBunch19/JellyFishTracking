from gpiozero import LED, Button, Device
from gpiozero.pins.rpigpio import RPiGPIOFactory
import time
import pygame
import sys

# led = LED(4)
# switch = Button(17)

# LR
dir2 = LED(5)
step2 = LED(13)
enable2 = LED(26)
# UD
dir1 = LED(24)
step1 = LED(25)
enable1 = LED(23)

m1 = 1
m2 = 1

screen_width = 1
screen_height = 1

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

# timer = 0.0009 # max
timer = 0.0004 # min - good
# timer = 0.0008
STEPS = 50
import threading


def move_motor(steps, stepper, instruction, dir):
    global timestart
    set_direction(instruction,dir)
    for _ in range(steps):
        # timer = rand
        # we are going to start with this constant movement speed, pretend I only have this speed available
        # control motor movement simply by number of steps which corresponds to distance
        # for speed - motor has pps 5000 - so the sum of the two sleeps below has to be greater than 0.0002
        stepper.on()
        # 0.0005 is good
        # at 0.001 for now
        # was 0.007
        # what a bout time up
        time.sleep(timer) # adjust for speed - min 0.0001
        stepper.off()
        time.sleep(timer) # adjust - min 0.0001

    # print('end time: ', time.time()-timestart)
    # timestart = time.time()

def move_2_motors(steps, stepper1, stepper2, instruction1, instruction2, dir1, dir2):
    set_direction(instruction1,dir1)
    set_direction(instruction2,dir2)
    for x in range(steps):
        stepper1.on()
        stepper2.on()
        time.sleep(timer)
        stepper1.off()
        stepper2.off()
        time.sleep(timer)

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

def move_motor_simple(steps,pulse,instruct,dir,timewait):
    set_direction(instruct,dir)
    for _ in range(steps):
        pulse.on()
        time.sleep(timewait) 
        pulse.off()
        time.sleep(timewait) 
    # print('endtime: ', time.time() - timestart)
timestart = time.time()
# th = 0
# thread3 = threading.Thread(target=move_motor,args=(100,step2,'backward',dir2))

def motor_moves():
    # global th, thread3
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            print("trying to quit")
        elif event.type == pygame.KEYDOWN:
            global m1, m2
            if event.key == pygame.K_1:
                if m1:
                    enable_driver(False,enable1)
                    m1 = 0
                else:
                    enable_driver(True,enable1)
                    m1 = 1
            if event.key == pygame.K_2:
                if m2:
                    enable_driver(False,enable2)
                    m2 = 0
                else:
                    enable_driver(True,enable2)
                    m2 = 1
            if event.key == pygame.K_p:
                move_motor(100,step2,'forward',dir2)
    keys = pygame.key.get_pressed()

    # if th == 1:
    #     thread3.join()
    #     th = 0

    movement_dir = get_dir(keys)
    steps = STEPS
    end = time.time()
    if keys[pygame.K_o]:
        print('o: ', time.time()-end)
        move_motor(100,step2,'backward',dir2)
        print('d: ')
        end = time.time()

    if movement_dir[0] == -1:
        if movement_dir[1] == -1:
            # down left
            move_2_motors(steps,step1,step2,'backward','backward',dir1,dir2)
            # screen.fill((10,20,40))
            # screen.blit(arrows[1],rects[1])
            pygame.display.flip()
        elif movement_dir[1] == 1:
            # up left
            move_2_motors(steps,step1,step2,'forward','backward',dir1,dir2)
            # screen.fill((10,20,40))
            # screen.blit(arrows[3],rects[3])
            pygame.display.flip()
        else:
            # left
            # print('move left')
            move_motor(steps,step2,'backward',dir2)
            # thread3.start()
            # th=1


            # screen.fill((10,20,40))
            # screen.blit(arrows[2],rects[2])
            pygame.display.flip()
    elif movement_dir[0] == 1:
        if movement_dir[1] == -1:
            # down right
            move_2_motors(steps,step1,step2,'backward','forward',dir1,dir2)
            # screen.fill((10,20,40))
            # screen.blit(arrows[7],rects[7])
            pygame.display.flip()
        elif movement_dir[1] == 1:
            # up right
            # thread1 = threading.Thread(target=move_motor_simple, args=(99,step1,'forward',dir1,0.0002))
            # thread2 = threading.Thread(target=move_motor_simple, args=(33,step2,'forward',dir2,0.0006))
            # thread1.start()
            # thread2.start()
            # thread1.join()
            # thread2.join()
            
            
            move_2_motors(steps,step1,step2,'forward','forward',dir1,dir2)
            # screen.fill((10,20,40))
            # screen.blit(arrows[5],rects[5])
            pygame.display.flip()
        else:
            # right
            # print('move right')
            move_motor(steps,step2,'forward',dir2)
            # screen.fill((10,20,40))
            # screen.blit(arrows[6],rects[6])
            pygame.display.flip()
    else:
        if movement_dir[1] == -1:
            # down
            move_motor(steps,step1,'backward',dir1)
            # screen.fill((10,20,40))
            # screen.blit(arrows[0],rects[0])
            pygame.display.flip()
        elif movement_dir[1] == 1:
            # up
            move_motor(steps,step1,'forward',dir1)
            # screen.fill((10,20,40))
            # screen.blit(arrows[4],rects[4])
            pygame.display.flip()
        else:
            # nothing
            # screen.fill((10,20,40))
            pygame.display.flip()


    # move_motor(1000, 'forward')
    # time.sleep(1)
    # move_motor(200, 'backward')
    # time.sleep(1)

def main():
    try:
        enable_driver(True, enable1)
        enable_driver(True, enable2)
        while True:
            motor_moves()
    except KeyboardInterrupt:
        print("Stopping the motor")
    finally:
        enable_driver(False, enable1) 
        enable_driver(False, enable2)

if __name__ == '__main__':
    pygame.init()

    # led.on()
    
    screen = pygame.display.set_mode((screen_width,screen_height))
    pygame.display.set_caption("arrow detector")
    main()
    # led.off()
    print("LED OFF, everything done")
    pygame.quit()
    sys.exit()


    # print("a")
    # if switch.is_pressed:
    #     print('x')
    #     led.on()
    # else:
    #     led.off()
    