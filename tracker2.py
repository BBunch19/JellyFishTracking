# Tracker 2
# combining Grasshopper 3 camera and motors to move the camera to track
# jelly fish (and other things - like light)
# Brennan Hoppa
#
# Updates from tracker1: initialization / boundary settings, camera/motors knowing position

# imports ---------------------
import PySpin # type: ignore
import sys
import numpy as np
import matplotlib.pyplot as plt # type: ignore
import cv2 # type: ignore
import time
from gpiozero import LED, Button, Device
import pygame
import subprocess
import os
import socket
import threading
import concurrent.futures
import psutil
from scipy.spatial import distance # type: ignore

# set up ---------------------------------
pygame.init()
screen = pygame.display.set_mode((0, 0), pygame.FULLSCREEN)
pygame.display.set_caption("Pygame Text Input Example")
screen_width, screen_height = screen.get_size()

# LED / MOTOR Setup
# greenled = LED(4)
# switch = Button(1)
# LR
dirX = LED(5)
pulseX = LED(13)
onoffX = LED(26)  # if true/high, motor is unpower, can move with hand
# UD
dirY = LED(24)
pulseY = LED(25)
onoffY = LED(23)  # if true/high, motor is unpower, can move with hand


baseStepsPerRevolution = 200 # step angle is 1.8 deg, 360/18 = 200
microsteps = 1 # adjust off of switches
stepsPerRevolution = baseStepsPerRevolution * microsteps

# arrow setup
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
for i,a in enumerate(arrows):
    w,h = a.get_size()
    target_w = 400
    sf = int(target_w / w)
    target_h = int(h * sf)
    na = pygame.transform.scale(a,(target_w,target_h))
    ir = na.get_rect()
    arrows[i] = na

# Tracking setup -----------------------
NUM_IMAGES = 30  # number of images to use in AVI file
# FPS / vid length is not working properly at all - fix later
VIDEO_NUMBER = "two"
THRESHOLD = 70 # for light stuff
ALPHA = 100

# Steps tracking setup ------------------------------
def read_coords(file_name):
    numbers = [0,0]
    try:
        with open(file_name, 'r') as file:
            line = file.read().strip()
            numbers = [int(num) for num in line.split(',')]
    except FileNotFoundError:
        print("Coordinate file not found")
    except ValueError:
        print("Not valid coordinates")
    return numbers

def write_coords(file_name,numbers):
    with open(file_name, 'w') as file:
        file.write(', '.join(map(str,numbers)))

xcord, ycord = read_coords('motor_coords')
xtanko, ytanko = read_coords('tank_o_coords')
xtankm, ytankm = read_coords('tank_maxs')
xloc, yloc = xcord - xtanko, ycord - ytanko
xglobsteps, yglobsteps = 21700, 21700

coord_lock = threading.Lock()

timestart = time.time()
# print(xcord)
# print(ycord)

# Font setup ------------------------
font = pygame.font.Font(None, 44)
helptextfont = pygame.font.Font(None, 24)
input_box_width = 140
input_box_height = 74
input_box = pygame.Rect((screen_width // 2) - (input_box_width // 2), screen_height - input_box_height - 20, input_box_width, input_box_height)
color_inactive = pygame.Color('lightskyblue3')
color_active = pygame.Color('dodgerblue2')
color = color_inactive
active = False
text = ''
help_text = ''
inputtxt = 'input:'
instructions = [
    "Commands:",
    "m - switches to manual motor control",
    "t - switches to tracking motor control",
    "arrows - move motors (don't do inside input box)",
    "s - change speed (1-100)",
    "o - turn motors on or off allowing hand turning",
    "c - change tank origin and size",
    "b - turn on/off tank boundaries (camera staying only within the tank)",
    "x - re zero x and y coordinates manually",
    "i - change time step of the motors",
    "z - change manual steps of the motors",
    "j - switch between tracking a light or JF in tracking mode",
    "r - turn recording on/off",
    "ctrl c / esc - shuts down"
]
instructions_pos = (screen_width - 1200,50)
done = False

defaultspeed = 100 # default speed
speed = 100 # speed variable to change
manualsteps = 100 # adjust number of steps / amount of rot per arrow press *** ADJUST THIS
mode = 'manual'
trackingspeed = 0 # figure out what to do with this
xmotor = 'on'
ymotor = 'on'

minscreenspeed = 1
maxscreenspeed = 100
motorspeedmin_step_time = 0.0004 # actual min in 0.0001 but motors don't really work - this is FAST
motorspeedmax_step_time = 0.0009 # this could be longer but doesn't work well past this - this is SLOW

mottime = 0.0005

nextin_speed = False
nextin_motor = False
nextin_origin = False
nextin_maxs = False
tank_boundaries = 'off'
nextin_tbc = False
nextin_rezero = False
nextin_time = False
nextin_steps = False
trackJF = False
recording = 'off'

trackinginitialized = 0
cam = ''
cam_list = []
system = ''
images = list()
processor = 0
spaceholderimagepath = "/home/user/Documents/spaceholder.jpg"
shi = cv2.imread(spaceholderimagepath)

# HELPER FUNCTIONS MOTORS
def set_direction(instruction, dir):
    if instruction == "forward":
        dir.on()
    else:
        dir.off()

def swap_driver(enabler):
    if enabler.is_lit:
        enabler.off()
    else:
        enabler.on()

def move_motor(steps, stepper, instruction, dir, netdir, timing):
    set_direction(instruction,dir)
    # use linear mapping to do screen speed to motor time step

    global timestart, mottime
    timemapped = timing
    for x in range(steps):
        # we are going to start with this constant movement speed, pretend I only have this speed available
        # control motor movement simply by number of steps which corresponds to distance
        # for speed - motor has pps 5000 - so the sum of the two sleeps below has to be greater than 0.0002
        result = change_coord(netdir)
        if not result:
            return
        # print(x)
        stepper.on()
        time.sleep(timemapped) # adjust for speed - min 0.0001
        stepper.off()
        time.sleep(timemapped) # adjust - min 0.0001
    # print(timemapped)
    # print('end time 1 move motor: ', time.time()-timestart)
    timestart = time.time()

def change_coord(dir):
    global xcord, ycord, nextin_rezero, tank_boundaries
    # with coord_lock:
    if not nextin_rezero:
        if (xcord == 0 and dir == 'l') or (xcord == xglobsteps and dir == 'r') or (ycord == 0 and dir == 'd') or (ycord == yglobsteps and dir == 'u'):
            return False
        if tank_boundaries == 'on':
            if (xcord == xtanko and dir == 'l') or (xcord == xtankm and dir == 'r') or (ycord == ytanko and dir == 'd') or (ycord == ytankm and dir == 'u'):
                return False
    if dir == 'l':
        xcord -= 1
    if dir == 'r':
        xcord += 1
    if dir == 'u':
        ycord += 1
    if dir == 'd':
        ycord -= 1
    return True

def check_change_coord(dir):
    global xcord, ycord, tank_boundaries
    if (xcord == 0 and dir == 'l') or (xcord == 21700 and dir == 'r') or (ycord == 0 and dir == 'd') or (ycord == 21700 and dir == 'u'):
        return False
    if tank_boundaries == 'on':
        if (xcord == xtanko and dir == 'l') or (xcord == xtankm and dir == 'r') or (ycord == ytanko and dir == 'd') or (ycord == ytankm and dir == 'u'):
            return False
    return True
    
def move_2_motors(steps, stepper1, stepper2, instruction1, instruction2, dirX, dirY, netdir):
    global mottime
    set_direction(instruction1,dirY)
    set_direction(instruction2,dirX)
    speedp = 1 - (speed - minscreenspeed) / maxscreenspeed
    timemapped = motorspeedmin_step_time + speedp * (motorspeedmax_step_time - motorspeedmin_step_time)
    timemapped = mottime
    for _ in range(steps):
        eachdir = list(netdir)
        for d in eachdir:
            if d == 'd' or d == 'u':
                rUD = change_coord(d)
            if d == 'r' or d == 'l':
                rRL = change_coord(d)
        if not rUD and not rRL:
            return
        if rUD:
            stepper2.on()
        if rRL:
            stepper1.on()
        time.sleep(timemapped)
        stepper1.off()
        stepper2.off()
        time.sleep(timemapped)

def motors_tracking(err,timex,timey, stepsx, stepsy):
    netdir = ''
    if err[1] < 0:
        netdir += 'd'
    if err[1] > 0:
        netdir += 'u'
    if err[0] < 0:
        netdir += 'l'
    if err[0] > 0:
        netdir += 'r'
    if len(netdir) == 0:
        return # no moving of motors
    if len(netdir) == 1:
        if netdir[0] == 'u':
            # move_one_tracking(stepsy,pulseY,'forward',dirY, timey, netdir)
            th = threading.Thread(target=move_motor,args=(stepsy,pulseY,'forward',dirY,netdir,timey))
        elif netdir[0] == 'd':
            # move_one_tracking(stepsy,pulseY,'backward',dirY, timey, netdir)
            th = threading.Thread(target=move_motor,args=(stepsy,pulseY,'backward',dirY,netdir,timey))
        elif netdir[0] == 'l':
            # move_one_tracking(stepsx, pulseX, 'backward', dirX, timex, netdir)
            th = threading.Thread(target=move_motor,args=(stepsx,pulseX,'backward',dirX,netdir,timex))
        elif netdir[0] == 'r':
            # move_one_tracking(stepsx, pulseX, 'forward', dirX, timex, netdir)
            th = threading.Thread(target=move_motor,args=(stepsx,pulseX,'forward',dirX,netdir,timex))
        th.start()
        return th, 0
    else:
        if netdir == 'dl':
            th, th2 = move_motors_tracking(stepsx, pulseX, 'backward', dirX, timex, stepsy,pulseY,'backward',dirY, timey, netdir)
        if netdir == 'dr':
            th, th2 = move_motors_tracking(stepsx, pulseX, 'forward', dirX, timex, stepsy,pulseY,'backward',dirY, timey, netdir)
        if netdir == 'ul':
            th, th2 = move_motors_tracking(stepsx, pulseX, 'backward', dirX, timex, stepsy,pulseY,'forward',dirY, timey, netdir)
        if netdir == 'ur':
            th, th2 = move_motors_tracking(stepsx, pulseX, 'forward', dirX, timex, stepsy,pulseY,'forward',dirY, timey, netdir)
        return th, th2

def move_one_tracking(steps, pulse, instruct, dir, timewait, netdir):
    set_direction(instruct,dir)
    for _ in range(steps):
        result = change_coord(netdir)
        if not result:
            return
        pulse.on()
        time.sleep(timewait) 
        pulse.off()
        time.sleep(timewait) 

def move_motor_simple(steps,pulse,instruct,dir,timewait):
    set_direction(instruct,dir)
    for _ in range(steps):
        pulse.on()
        time.sleep(timewait) 
        pulse.off()
        time.sleep(timewait) 

def move_motors_tracking(xsteps, xpulse, xinstruct, xdir, xtime, ysteps, ypulse, yinstruct, ydir, ytime, netdir):
    global ycord, xcord, tank_boundaries
    moveY, moveX = True, True
    d1 = netdir[0]
    if d1 == 'u':
        if ycord + ysteps > yglobsteps or (tank_boundaries == 'on' and ycord + ysteps > ytankm):
            moveY = False
        else:
            ycord += ysteps
    if d1 == 'd':
        if ycord - ysteps < 0 or (tank_boundaries == 'on' and ycord - ysteps < ytanko):
            moveY = False
        else: 
            ycord -= ysteps
    d2 = netdir[1]
    if d2 == 'r':
        if xcord + xsteps > xglobsteps or (tank_boundaries == 'on' and xcord + xsteps > xtankm):
            moveX = False
        else:
            xcord += xsteps
    if d2 == 'l':
        if xcord - xsteps < 0 or (tank_boundaries == 'on' and xcord - xsteps < xtanko):
            moveX = False
        else:
            xcord -= xsteps
    if moveX and moveY:
        thread1 = threading.Thread(target=move_motor_simple, args=(xsteps,xpulse,xinstruct,xdir,xtime))
        thread2 = threading.Thread(target=move_motor_simple, args=(ysteps,ypulse,yinstruct,ydir,ytime))
        thread1.start()
        thread2.start()
        return thread1, thread2
    elif moveX and not moveY:
        # th = move_one_tracking(xsteps,xpulse,xinstruct,xdir,xtime,netdir)
        th = threading.Thread(target=move_one_tracking, args=(xsteps,xpulse,xinstruct,xdir,xtime,netdir))
        th.start()
        return th
    elif moveY and not moveX:
        # th = move_one_tracking(ysteps,ypulse,yinstruct,ydir,ytime,netdir)
        th = threading.Thread(target=move_one_tracking, args=(ysteps, ypulse, yinstruct,ydir, ytime, netdir))
        th.start()
        return th
    else:
        # do nothing
        pass

 

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

def apply_move(movement_dir):
    if movement_dir[0] == -1:
        if movement_dir[1] == -1:
            # down left
            # move_2_motors(manualsteps,pulseX,pulseY,'backward','backward',dirX,dirY, 'dl')
            th = threading.Thread(target=move_2_motors,args=(manualsteps,pulseX,pulseY,'backward','backward',dirX,dirY,'dl'))
            th.start()
            arrowind = 1
        elif movement_dir[1] == 1:
            # up left
            # move_2_motors(manualsteps,pulseX,pulseY,'forward','backward',dirX,dirY, 'ul')
            th = threading.Thread(target=move_2_motors,args=(manualsteps,pulseX,pulseY,'forward','backward',dirX,dirY,'ul'))
            th.start()
            arrowind = 3
        else:
            # left
            # print('starting mot left move: ', time.time()-start)
            # move_motor(manualsteps,pulseX,'backward',dirX, 'l',mottime)
            # th = 0
            th = threading.Thread(target=move_motor,args=(manualsteps,pulseX,'backward',dirX,'l',mottime))
            th.start()
            arrowind = 2
    elif movement_dir[0] == 1:
        if movement_dir[1] == -1:
            # down right
            # move_2_motors(manualsteps,pulseX,pulseY,'backward','forward',dirX,dirY, 'dr')
            th = threading.Thread(target=move_2_motors,args=(manualsteps,pulseX,pulseY,'backward','forward',dirX,dirY,'dr'))
            th.start()
            arrowind = 7
        elif movement_dir[1] == 1:
            # up right
            # move_2_motors(manualsteps,pulseX,pulseY,'forward','forward',dirX,dirY, 'ur')
            th = threading.Thread(target=move_2_motors,args=(manualsteps,pulseX,pulseY,'forward','forward',dirX,dirY,'ur'))
            th.start()
            arrowind = 5
        else:
            # right
            # move_motor(manualsteps,pulseX,'forward',dirX, 'r')
            # print('starting mot right move: ', time.time()-start)
            th = threading.Thread(target=move_motor,args=(manualsteps,pulseX,'forward',dirX,'r',mottime))
            th.start()
            arrowind = 6
    else:
        if movement_dir[1] == -1:
            # down
            # move_motor(manualsteps,pulseY,'backward',dirY, 'd')
            th = threading.Thread(target=move_motor,args=(manualsteps,pulseY,'backward',dirY,'d',mottime))
            th.start()
            arrowind = 0
        elif movement_dir[1] == 1:
            # up
            # move_motor(manualsteps,pulseY,'forward',dirY, 'u')
            th = threading.Thread(target=move_motor,args=(manualsteps,pulseY,'forward',dirY,'u',mottime))
            th.start()
            arrowind = 4
        else:
            # nothing
            arrowind = 100
            th = 0
    return arrowind, th

# HELPER FUNCTIONS CAMERA
def find_dark_spots(image, threshold):
    _,thresh = cv2.threshold(image,threshold,255,cv2.THRESH_BINARY_INV) # adjust the first value to get lighter/smaller spots - higher is more sensitive
    contours, _ = cv2.findContours(thresh,cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    dark_spots = [cv2.boundingRect(contour) for contour in contours]
    return dark_spots

def distance_from_center(point,center):
    return distance.euclidean(point, center)

# New JF function
def findJF(image,radius,threshold,min_contour_area,cluster_distance_threshold,min_cluster_size):
    height, width = image.shape[:2]
    center = (width // 2, height // 2)
    mask = np.zeros((height, width), dtype=np.uint8)
    cv2.circle(mask, center, radius, 255, -1)
    masked_image = cv2.bitwise_and(image, image, mask=mask)
    gray = cv2.cvtColor(masked_image, cv2.COLOR_BGR2GRAY)
    _, binary = cv2.threshold(gray, threshold, 255, cv2.THRESH_BINARY_INV)  # Adjust threshold as needed
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    centroids = []
    for contour in contours:
        if cv2.contourArea(contour) > min_contour_area:
            M = cv2.moments(contour)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                centroids.append((cX, cY))
                cv2.circle(masked_image, (cX, cY), 3, (0, 255, 0), -1)
    clusters = []
    for i, centroid in enumerate(centroids):
        cluster = [centroid]
        for j, other_centroid in enumerate(centroids):
            if i != j and distance.euclidean(centroid, other_centroid) < cluster_distance_threshold:
                cluster.append(other_centroid)
        if len(cluster) >= min_cluster_size:
            clusters.append(cluster)
    for cluster in clusters:
        for centroid in cluster:
            cv2.circle(masked_image, centroid, 5, (0, 0, 255), -1)
    if clusters:
        closest_cluster = min(clusters, key=lambda c: distance_from_center(np.mean(c, axis=0),center))
        avg_cX, avg_cY = np.mean(closest_cluster, axis=0).astype(int)
        # print(f"Centroid of the closest cluster: ({avg_cX}, {avg_cY})")
        
        # Draw a purple dot on the centroid of the closest cluster
        cv2.circle(masked_image, (avg_cX, avg_cY), 7, (255, 0, 255), -1)
        return masked_image, avg_cX, avg_cY
    else:
        return masked_image, 0,0



def group_spots(dark_spots, alpha):
    groups = []
    for x,y,w,h in dark_spots:
        added_to_group = False
        for group in groups:
            gx, gy = np.mean([x + w / 2 for x, _, w, _ in group]), np.mean([y + h / 2 for _, y, _, h in group])
            dist = np.sqrt((gx - (x + w/2))**2 + (gy - (y+h/2))**2)
            if dist <= alpha:
                group.append((x,y,w,h))
                added_to_group = True
        if not added_to_group:
            groups.append([(x,y,w,h)])
    return groups

def find_jelly_group(groups):
    # only works if we actually get all 5 and may still need to refine
    # returns list of groups rn, we just want 1
    jg = [g for g in groups if len(g) == 5]
    return jg

def find_white_spot(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    _, thresh = cv2.threshold(gray,240,255,cv2.THRESH_BINARY)
    contours,_ = cv2.findContours(thresh,cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cx,cy = 0,0
    if contours:
        largest = max(contours, key=cv2.contourArea)
        mom = cv2.moments(largest)
        if mom['m00'] != 0:
            cx = int(mom['m10'] / mom['m00'])
            cy = int(mom['m01'] / mom['m00'])
            cv2.drawContours(image,[largest], -1, (0,255,0),2)
            cv2.circle(image, (cx,cy), 5, (255,0,0), -1)
        else:
            print('no centroid')
        # x,y,w,h = cv2.boundingRect(largest)  
    return image, cx, cy

def find_dark_spots(image,threshold):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    _, thresh = cv2.threshold(gray,threshold,255,cv2.THRESH_BINARY_INV) #adjust threshold - higher is more sensitive, will get smaller spots
    contours, _ = cv2.findContours(thresh,cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    dark_spots = [cv2.boundingRect(contour) for contour in contours]
    return dark_spots

def group_spots(dark_spots, alpha):
    groups = []
    for x,y,w,h in dark_spots:
        added_to_group = False
        for group in groups:
            gx, gy = np.mean([x + w / 2 for x, _, w, _ in group]), np.mean([y + h / 2 for _, y, _, h in group])
            dist = np.sqrt((gx - (x + w/2))**2 + (gy - (y+h/2))**2)
            if dist <= alpha:
                group.append((x,y,w,h))
                added_to_group = True
        if not added_to_group:
            groups.append([(x,y,w,h)])
    return groups

def find_jelly_group(groups):
    # only works if we actually get all 5 and may still need to refine
    # returns list of groups rn, we just want 1
    jg = [g for g in groups if len(g) == 5]
    return jg

def initialize_tracking():
    global mode, speed, help_text, cam, images, cam_list, system, trackinginitialized, processor
    system = PySpin.System.GetInstance()
    cam_list = system.GetCameras()
    numcams = cam_list.GetSize()
    if numcams == 0:
        mode = 'manual'
        speed = defaultspeed
        help_text = "Tracking unavailable due to no cameras being found"
        cam_list.Clear()
        system.ReleaseInstance()
        return False
    elif numcams > 1:
        mode = 'manual'
        speed = defaultspeed
        help_text = "Tracking unavailable due to too many cameras being found"
        cam_list.Clear()
        system.ReleaseInstance()
        return False
    else:
        cam = cam_list[0]
        try:
            cam.Init()
        except:
            print("camera did not initialize")
            cam.DeInit()
            # camera is plugged into bus 001, device 004 - power cycle the port
            bus_number = '1'
            device_number = '4'
            subprocess.run(['sudo','echo','0','>',f'/sys/bus/usb/devices/{bus_number}-{device_number}/power/level'])
            time.sleep(1)
            subprocess.run(['sudo','echo','1','>',f'/sys/bus/usb/devices/{bus_number}-{device_number}/power/level'])
            print('Power cycled camera')
            python = sys.executable
            os.execl(python, python, *sys.argv)
        nodemap = cam.GetNodeMap()
        node_acquisition_mode = PySpin.CEnumerationPtr(nodemap.GetNode("AcquisitionMode"))
        if PySpin.IsAvailable(node_acquisition_mode) and PySpin.IsWritable(node_acquisition_mode):
            node_acquisition_mode_continuous = node_acquisition_mode.GetEntryByName("Continuous")
            if PySpin.IsAvailable(node_acquisition_mode_continuous) and PySpin.IsReadable(node_acquisition_mode_continuous):
                acquisition_mode_continuous = node_acquisition_mode_continuous.GetValue()
                node_acquisition_mode.SetIntValue(acquisition_mode_continuous)
                # print("Acquisition mode set to Continuous")

        # Set pixel format to Mono8
        node_pixel_format = PySpin.CEnumerationPtr(nodemap.GetNode("PixelFormat"))
        if PySpin.IsAvailable(node_pixel_format) and PySpin.IsWritable(node_pixel_format):
            node_pixel_format_mono8 = node_pixel_format.GetEntryByName("Mono8")
            if PySpin.IsAvailable(node_pixel_format_mono8) and PySpin.IsReadable(node_pixel_format_mono8):
                pixel_format_mono8 = node_pixel_format_mono8.GetValue()
                node_pixel_format.SetIntValue(pixel_format_mono8)
                # print("Pixel format set to Mono8")
        
        
        nodefr = PySpin.CFloatPtr(nodemap.GetNode("AcquisitionFrameRate"))
        fr = nodefr.GetValue()
        print("framerate: ", fr)




        w = cam.Width.GetValue()
        h = cam.Height.GetValue()
        print(w, h)
        # wmin = cam.Width.GetMin()
        # wmax = cam.Width.GetMax()
        # hmin = cam.Height.GetMin()
        # hmax = cam.Height.GetMax()
        # print("widht min max:  ", wmin, wmax)
        # print('height min max:  ', hmin, hmax)
        # cam.Width.SetValue(2048)
        # cam.Height.SetValue(2048)
        # w = cam.Width.GetValue()
        # h = cam.Height.GetValue()
        # print(w, h)
        # cam.DecimationHorizontal.SetValue(2)
        # cam.DecimationVertical.SetValue(2)


        cam.BeginAcquisition()
        images = list()
        processor = PySpin.ImageProcessor()
        processor.SetColorProcessing(PySpin.SPINNAKER_COLOR_PROCESSING_ALGORITHM_HQ_LINEAR)
        trackinginitialized = 1

        # return False # test code
        return True

host = '10.29.156.85' # get ip of the computer - on windows open the terminal, type ipconfig, it's the IPv4 address
port = 12345 # set this on the receiver

# saving connection function 
def get_connected():
    global host, port, help_text
    try: 
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(20)
        s.connect((host,port))
        s.settimeout(None)
        return s
    except:
        return False

def send_image(sock,name,image,timestamp):
    # print('in thing, timestamp: ', timestamp)
    sock.sendall(name.encode() + b'\n')
    sock.sendall(b'START_IMG\n')
    _, image_data = cv2.imencode('.jpg', image)
    image_bytes = image_data.tobytes()
    # print(len(image_bytes))
    sock.sendall(image_bytes)
    sock.sendall(b'\nEND_IMG\n')
    sock.sendall(str(timestamp).encode() + b'\n')
    sock.sendall(b'\nEOF\n')

# constant screen things
instructions_surfaces = [font.render(line, True, pygame.Color('white')) for line in instructions]


# screen update function
def update_screen():
    txt_surface = font.render(text, True, color)
    help_surface = helptextfont.render(help_text, True, pygame.Color('white'))
    input_surface = font.render(inputtxt, True, pygame.Color('white'))
    mode_surface = font.render(f'Current mode: {mode}',True,pygame.Color('green'))
    speed_surface = font.render(f'Current speed: {speed}',True,pygame.Color('green'))
    # default = onoffX.is_lit
    motorxsur = font.render(f'X Motor: {xmotor}',True,pygame.Color('green'))
    motorysur = font.render(f'Y Motor: {ymotor}',True,pygame.Color('green'))
    gcoordssur= font.render(f'Global Coordinates (steps): {xcord} , {ycord}',True,pygame.Color('white'))
    gcoordssurin = font.render(f'Global Coordinates (in): {round(xcord/620,1)} , {round(ycord/620,1)}',True,pygame.Color('white'))
    lcoordorisur= font.render(f'Tank Origin (steps, in): {xtanko} , {ytanko} ({round(xtanko/620,1)}" , {round(ytanko/620,1)}")',True,pygame.Color('white'))
    lcoordmaxsur= font.render(f'Tank Size (steps, in): {xtankm-xtanko} , {ytankm-ytanko} ({round((xtankm-xtanko)/620,1)}" , {round((ytankm-ytanko)/620,1)}")',True,pygame.Color('white'))
    timesur = font.render(f'Motor Time (sec): {mottime}', True,pygame.Color('white'))
    stepssur = font.render(f'Motor Steps: {manualsteps}', True,pygame.Color('white'))
    if trackJF:
        JFtsur = font.render(f'In tracking mode, tracking: Jellyfish', True,pygame.Color('white'))
    else:
        JFtsur = font.render(f'In tracking mode, tracking: Light', True,pygame.Color('white'))
    if xcord<xtanko or xcord>xtankm:
        if ycord<ytanko or ycord>ytankm:
            lcoordssur= font.render(f'Tank Coordinates (steps): OUT OF TANK , OUT OF TANK',True,pygame.Color('white'))
            lcoordssurin = font.render(f'Tank Coordinates (in): OUT OF TANK , OUT OF TANK',True,pygame.Color('white'))
        else:
            lcoordssur= font.render(f'Tank Coordinates (steps): OUT OF TANK , {yloc}',True,pygame.Color('white'))
            lcoordssurin = font.render(f'Tank Coordinates (in): OUT OF TANK , {round(yloc/620,1)}',True,pygame.Color('white'))    
    elif ycord<ytanko or ycord>ytankm:
        lcoordssur= font.render(f'Tank Coordinates (steps): {xloc} , OUT OF TANK',True,pygame.Color('white'))
        lcoordssurin = font.render(f'Tank Coordinates (in): {round(xloc/620,1)} , OUT OF TANK',True,pygame.Color('white'))
    else:
        lcoordssur= font.render(f'Tank Coordinates (steps): {xloc} , {yloc}',True,pygame.Color('white'))
        lcoordssurin = font.render(f'Tank Coordinates (in): {round(xloc/620,1)} , {round(yloc/620,1)}',True,pygame.Color('white'))
    tboundsur = font.render(f'Tank Boundaries: {tank_boundaries}', True, pygame.Color('green'))
    recsur = font.render(f'Recording: {recording}', True, pygame.Color('green'))

    # Resize the box if the text is too long.
    width = max(200, txt_surface.get_width()+10)
    input_box.w = width

    # Fill the screen with a color (RGB)
    screen.fill((30, 30, 30))
    # Blit the screen
    screen.blit(help_surface, (input_box.x, input_box.y - help_surface.get_height() - 10))
    screen.blit(input_surface,(input_box.x - input_surface.get_width() - 10, input_box.y + 20))
    screen.blit(txt_surface, (input_box.x+5, input_box.y+5))
    for idx, surf in enumerate(instructions_surfaces):
        ls = 30
        screen.blit(surf, (instructions_pos[0], instructions_pos[1]+idx*ls))
        if idx == len(instructions_surfaces)-1:
            screen.blit(mode_surface,(instructions_pos[0],instructions_pos[1]+idx*ls+ls*2))
            screen.blit(speed_surface,(instructions_pos[0],instructions_pos[1]+idx*ls+ls*3))
            screen.blit(motorxsur,(instructions_pos[0],instructions_pos[1]+idx*ls+ls*4))
            screen.blit(motorysur,(instructions_pos[0],instructions_pos[1]+idx*ls+ls*5))
            screen.blit(gcoordssur,(instructions_pos[0],instructions_pos[1]+idx*ls+ls*6))
            screen.blit(gcoordssurin,(instructions_pos[0],instructions_pos[1]+idx*ls+ls*7))
            screen.blit(lcoordorisur,(instructions_pos[0],instructions_pos[1]+idx*ls+ls*8))
            screen.blit(lcoordmaxsur,(instructions_pos[0],instructions_pos[1]+idx*ls+ls*9))            
            screen.blit(lcoordssur,(instructions_pos[0],instructions_pos[1]+idx*ls+ls*10))
            screen.blit(lcoordssurin,(instructions_pos[0],instructions_pos[1]+idx*ls+ls*11))
            screen.blit(tboundsur,(instructions_pos[0],instructions_pos[1]+idx*ls+ls*12))
            screen.blit(timesur,(instructions_pos[0],instructions_pos[1]+idx*ls+ls*13))
            screen.blit(stepssur,(instructions_pos[0],instructions_pos[1]+idx*ls+ls*14))
            screen.blit(JFtsur,(instructions_pos[0],instructions_pos[1]+idx*ls+ls*15))
            screen.blit(recsur,(instructions_pos[0],instructions_pos[1]+idx*ls+ls*16))

    pygame.draw.rect(screen, color, input_box, 2)
    # if arrowind != 100:
    #    screen.blit(arrows[arrowind],(10,10))
    
    isurf = pygame.surfarray.make_surface(image)
    screen.blit(isurf,(0,screen_height - target_hei))

    # Update the display
    pygame.display.flip()


#### actual start of the program
# init the camera no matter what
if not trackinginitialized:
    ans = initialize_tracking()

# test code
if ans == False:
    try:
        cam.DeInit()
        del cam
        cam_list.Clear()
        system.ReleaseInstance()
    except:
        pass
    final_coords = [xcord,ycord]
    tankocoords = [xtanko,ytanko]
    tankmcoords = [xtankm,ytankm]
    write_coords('motor_coords',final_coords)
    write_coords('tank_o_coords',tankocoords)
    write_coords('tank_maxs',tankmcoords)
    pygame.quit()
    sys.exit()
# end test code

start = time.time()
isaver = 1
imagecounter = 0
# main event loop -------------------------

while not done:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            done = True
        elif event.type == pygame.MOUSEBUTTONDOWN:
            # If the user clicked on the input_box rect.
            if input_box.collidepoint(event.pos):
                # Toggle the active variable.
                active = not active
            else:
                active = False
            # Change the current color of the input box.
            color = color_active if active else color_inactive
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_ESCAPE:
                done = True
            elif event.key == pygame.K_c and pygame.key.get_mods() & pygame.KMOD_CTRL:
                done = True
            elif event.key == pygame.K_TAB:
                if active:
                    active = False
                else:
                    active = True
                color = color_active if active else color_inactive
            elif active:
                if event.key == pygame.K_RETURN:
                    # analyze inputted text
                    text = text.lower()
                    if nextin_speed:
                        try:
                            speed_intermediate = int(text)
                            if speed_intermediate > 100 or speed < 1:
                                raise KeyError
                            else:
                                speed = speed_intermediate
                                nextin_speed = False
                                help_text = ''
                        except:
                            help_text = "Not an integer"
                    elif nextin_motor:
                        try:
                            nextin_motor = False
                            help_text = ''
                            if text.lower() == 'x':
                                swap_driver(onoffX)
                                if xmotor == 'on':
                                    xmotor = 'off'
                                else:
                                    xmotor = 'on'
                            elif text.lower() == 'y':
                                swap_driver(onoffY)
                                if ymotor == 'on':
                                    ymotor = 'off'
                                else:
                                    ymotor = 'on'
                            elif text.lower() == 'b':
                                pass
                            else:
                                nextin_motor = True
                                raise KeyError
                        except:
                            help_text = "Not x,y, or b"
                    elif nextin_steps:
                        try:
                            manualsteps = int(text)
                            nextin_steps = False
                            help_text = ''
                        except:
                            help_text = 'Enter an integer for steps'                        
                    elif nextin_time:
                        try:
                            mottime = float(text)
                            nextin_time = False
                            help_text = ''
                        except:
                            help_text = 'Enter a number for time' 
                    elif nextin_origin:
                        if text == 'n':
                            xtanko, ytanko = xcord, ycord
                            nextin_origin = False
                            nextin_maxs = True
                            help_text = 'Move to the Top Right corner of tank, enter "n" for new maxes, or just enter to keep the old maxes'
                        elif text == '':
                            nextin_origin = False
                            nextin_maxs = True
                            help_text = 'Move to the Top Right corner of tank, enter "n" for new maxes, or just enter to keep the old maxes'
                        else:
                            help_text = 'Not valid input, please move to BL corner off tank and enter "n" or just press enter to keep the old origin'
                    elif nextin_maxs:
                        if xcord < xtanko or ycord < ytanko:
                            help_text = 'Incorrectly set boundaries, please repeat process at Bottom Left Corner with "n" or just enter'
                        if text == 'n':
                            xtankm, ytankm = xcord,ycord
                            nextin_maxs = False
                            help_text = "Successfully updated tank dimensions and location"
                        elif text == '':
                            nextin_maxs = False
                            help_text = "Successfully updated tank dimensions and location"
                        else:
                            help_text = 'Not valid input, please move to TR corner of tank and enter "n" or just enter'
                    elif nextin_tbc:
                        if text == '':
                            nextin_tbc = False
                            xtankc = xtanko+round((xtankm-xtanko)/2)
                            ytankc = ytanko+round((ytankm-ytanko)/2)
                            dx = xcord-xtankc
                            dy = ycord-ytankc  
                            if dx < 0:
                                move_motor(abs(dx),pulseX,'forward',dirX, 'r',mottime)
                            if dx > 0:
                                move_motor(abs(dx),pulseX,'backward',dirX, 'l',mottime)
                            if dy < 0:
                                move_motor(abs(dy),pulseY,'forward',dirY,'u',mottime)
                            if dy > 0:
                                move_motor(abs(dy),pulseY,'backward',dirY, 'd',mottime)
                            help_text = ''
                            tank_boundaries = 'on'
                        elif text == 'x':
                            nextin_tbc = False
                            help_text = ''
                        else:
                            help_text = 'Please press enter to move to the center of your tank or x to cancel'
                    elif nextin_rezero:
                        if text == '':
                            xcord,ycord = 0,0
                            help_text = 'Coordinates updated to 0,0'
                            nextin_rezero = False
                        else:
                            help_text = 'Please just hit enter with no text in the box when you are at the bottom left corner of the xy table'
                    elif text == 'm':
                        if mode != 'manual':
                            mode='manual'
                            help_text = ''
                            trackinginitialized = 0
                            try:
                                cam.Deinit()
                            except:
                                pass
                    elif text == 't':
                        if mode != 'tracking':
                            mode='tracking'
                            help_text='tracking, enter m to exit'
                    elif text == 's' and mode == 'manual':
                        nextin_speed = True
                        help_text = 'Type in desired speed (1-100) and press enter'
                    elif text == 's' and mode == 'tracking':
                        help_text = 'Cannot change speed in tracking mode'
                    elif text == 'o' and mode == 'manual':
                        nextin_motor = True
                        help_text = "Type in x or y to turn that motor on/off, or b to go back"
                    elif text == 'o' and mode == 'tracking':
                        help_text = 'Cannot turn on/off motors in tracking mode'
                    elif text == 'c':
                        mode='manual'
                        help_text = 'Move to Bottom Left corner of tank, enter "n" for new origin, or just enter to keep the old origin'
                        nextin_origin = True
                    elif text == 'x':
                        mode='manual'
                        help_text = 'Move to the bottom left corner of the x y gantry, bumping up as little as possible, then press enter'
                        nextin_rezero = True
                    elif text == 'z':
                        help_text = 'Enter number of manual steps'
                        nextin_steps = True
                    elif text == 'j':
                        trackJF = not trackJF
                    elif text == 'i':
                        nextin_time = True
                        help_text = 'Enter time step for motors'
                    elif text == 'b':
                        if tank_boundaries == 'on':
                            tank_boundaries = 'off'
                        else:
                            if xcord<xtanko or xcord>xtankm or ycord<ytanko or ycord>ytankm:
                                help_text = "Not currently in tank. Press enter to move to center of the tank, or x to cancel"
                                nextin_tbc = True
                            else:
                                tank_boundaries = 'on'
                    elif text == 'r':
                        if recording == 'off':
                            sock = get_connected()
                            if sock == False:
                                help_text = 'Unable to connect, ensure host, port is correct and receiver is running'
                            else:
                                help_text = 'Connected, recording has started'
                                recording = 'on'
                        elif recording == 'on':
                            recording = 'off'
                            try: 
                                sock.close()
                            except:
                                pass
                    else:
                        help_text = "Not recognized command"
                    text = ''
                elif event.key == pygame.K_BACKSPACE:
                    text = text[:-1]
                else:
                    text += event.unicode
            else:
                pass
    try:
        image_result = cam.GetNextImage(1) # was 1000 - time waiting for new image before moving on
        timestamp = image_result.GetTimeStamp()
        # print(timestamp/1e9)
        pixel_format = image_result.GetPixelFormatName()
        image_array = image_result.GetNDArray()   
        image_result.Release()
        image_rgb = cv2.cvtColor(image_array, cv2.COLOR_BGR2RGB)
        target_wid = 600
        target_hei = int((target_wid / image_rgb.shape[1])*image_rgb.shape[0])
        image = cv2.resize(image_rgb, (target_wid, target_hei))
        image = cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE)
        imagecounter += 1

        check_time = time.time()
        # print(check_time - timestart)
        if check_time - timestart < 1.1 and check_time - timestart > 0.97:
            print('recording is: ', recording, ' and fps this second: ', imagecounter)
            imagecounter = 0
            timestart = time.time()
        elif check_time - timestart > 1.05:
            timestart = time.time()

        isaver += 1
        if isaver % 1 == 0:
            name = 'image' + str(isaver) + '.jpg'
            # print(name)
            if recording == 'on':
                # try:
                #     imth.join()
                # except:
                #     pass
                
                send_image(sock,name,image,timestamp)

                # imth = threading.Thread(target=send_image, args=(sock,name,image))
                # imth.start()
            # cv2.imwrite(os.path.join('saved',name),image)

        shi = image
        dur = time.time() - start
        # print(f'elapsed time: {dur:.2f} sec')
        # print('original dims: ', image_rgb.shape[1], ' x ', image_rgb.shape[0])

    except:
        # print('didnt get new image')
        image = shi
        target_wid = 600
        target_hei = 600
    
    cpu_usage = psutil.cpu_percent()
    memory_info = psutil.virtual_memory()
    # print("Cpu usage: ", cpu_usage)
    # print('mem usage: ', memory_info.percent)

    if mode == 'manual':
        try:
            th.join()
        except:
            pass
        keys = pygame.key.get_pressed()
        movement_dir = get_dir(keys)
        # print('applying move:')
        arrowind, th = apply_move(movement_dir)
    else:
        # TRACKING *** JUST DO A WHITE LIGHT FIRST
        if trackJF:
            radius = 330
            thresh = 100
            minCA = .1
            cdthresh = 50
            mcs = 4
            image, cx, cy = findJF(image,radius,thresh,minCA,cdthresh,mcs)
            # print(time.time()-start)



            # below is so I can try man movement while testing above
            # keys = pygame.key.get_pressed()
            # movement_dir = get_dir(keys)
            # arrowind, th = apply_move(movement_dir)
            # cx = 0 

                
                
        else:
            image, cx, cy = find_white_spot(image)
        
        icx = round(target_wid/2)
        icy = round(target_hei/2)
        center = (icx,icy)
        cv2.drawMarker(image,center,(0,0,255),markerType=cv2.MARKER_CROSS)
        
        try:
            th.join()
        except:
            pass
        try:
            th2.join()
        except:
            pass

        if cx == 0 or cy == 0:
            # no light spot found or no jf found
            # Switch back to manual to find it
            pass
        else:
            err = [cy - icy, -1*(cx-icx)] # funky b/c image is rotated
        
            ts = 25 
            myttime = 0.0005
            arbmin = 50
            if abs(err[0]) > arbmin:
                if err[0] < 10:
                    # left
                    th = threading.Thread(target=move_motor, args=(ts,pulseX,'backward',dirX,'l',myttime))
                    th.start()
                else:
                    # move motor R 
                    th = threading.Thread(target=move_motor, args=(ts,pulseX,'forward',dirX,'r',myttime))
                    th.start()
            if abs(err[1]) > arbmin:
                if err[1] < 10:
                    # move motor Down
                    th2 = threading.Thread(target=move_motor, args=(ts,pulseY,'backward',dirY,'d',myttime))
                    th2.start()
                else:
                    # move motor Up 
                    th2 = threading.Thread(target=move_motor, args=(ts,pulseY,'forward',dirY,'u',myttime))
                    th2.start()
    
    xloc, yloc = xcord - xtanko, ycord - ytanko

    # Render the current text.
    if recording == 'on' and imagecounter % 5 == 0:
        update_screen()
    elif recording == 'off':
        update_screen()
    else:
        pass


# Quit Pygame / ending program ----------------------------
try:
    cam.DeInit()
    del cam
    cam_list.Clear()
    system.ReleaseInstance()
except:
    pass

try:
    sock.close()
except:
    pass

final_coords = [xcord,ycord]
tankocoords = [xtanko,ytanko]
tankmcoords = [xtankm,ytankm]
write_coords('motor_coords',final_coords)
write_coords('tank_o_coords',tankocoords)
write_coords('tank_maxs',tankmcoords)

# print(xcord)
# print(ycord)

pygame.quit()
sys.exit()
