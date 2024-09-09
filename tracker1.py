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
import subprocess
import os

# set up ---------------------------------
pygame.init()
screen = pygame.display.set_mode((0, 0), pygame.FULLSCREEN)
pygame.display.set_caption("Pygame Text Input Example")
screen_width, screen_height = screen.get_size()

# LED / MOTOR Setup
greenled = LED(4)
switch = Button(17)
dirX = LED(23)
pulseX = LED(18)
onoffX = LED(24) # if true/high, motor is unpower, can move with hand
dirY = LED(25)
pulseY = LED(12)
onoffY = LED(16) # if true/high, motor is unpower, can move with hand

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

# Font setup ------------------------
font = pygame.font.Font(None, 70)
helptextfont = pygame.font.Font(None, 44)
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
    " ",
    "Commands just for manual mode:",
    "arrows - move motors (don't do inside input box)",
    "s - change speed (1-100)",
    "o - turn motors on or off allowing hand turning",
    "ctrl c / esc - shuts down"
]
instructions_pos = (screen_width - 1200,50)
done = False

defaultspeed = 70 # default speed
speed = 70 # speed variable to change
manualsteps = 50 # adjust number of steps / amount of rot per arrow press *** ADJUST THIS
mode = 'manual'
trackingspeed = 0 # figure out what to do with this
xmotor = 'on'
ymotor = 'on'

minscreenspeed = 1
maxscreenspeed = 100
motorspeedmin_step_time = 0.0007 # actual min in 0.0001 but motors don't really work - this is FAST
motorspeedmax_step_time = 0.002 # this could be longer but doesn't work well past this - this is SLOW
nextin_speed = False
nextin_motor = False

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
    
def move_motor(steps, stepper, instruction, dir):
    set_direction(instruction,dir)
    for _ in range(steps):
        # we are going to start with this constant movement speed, pretend I only have this speed available
        # control motor movement simply by number of steps which corresponds to distance
        # for speed - motor has pps 5000 - so the sum of the two sleeps below has to be greater than 0.0002
        
        # use linear mapping to do screen speed to motor time step
        speedp = 1 - (speed - minscreenspeed) / maxscreenspeed
        timemapped = motorspeedmin_step_time + speedp * (motorspeedmax_step_time - motorspeedmin_step_time)
        # timemapped = motorspeedmin_step_time + (motorspeedmax_step_time - motorspeedmin_step_time) * (speed - minscreenspeed) / (maxscreenspeed - minscreenspeed)
        stepper.on()
        time.sleep(timemapped) # adjust for speed - min 0.0001
        stepper.off()
        time.sleep(timemapped) # adjust - min 0.0001

def move_2_motors(steps, stepper1, stepper2, instruction1, instruction2, dirX, dirY):
    set_direction(instruction1,dirY)
    set_direction(instruction2,dirX)
    for _ in range(steps):
        speedp = 1 - (speed - minscreenspeed) / maxscreenspeed
        timemapped = motorspeedmin_step_time + speedp * (motorspeedmax_step_time - motorspeedmin_step_time)
        stepper1.on()
        stepper2.on()
        time.sleep(timemapped)
        stepper1.off()
        stepper2.off()
        time.sleep(timemapped)

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
            move_2_motors(manualsteps,pulseX,pulseY,'backward','backward',dirX,dirY)
            arrowind = 1
        elif movement_dir[1] == 1:
            # up left
            move_2_motors(manualsteps,pulseX,pulseY,'forward','backward',dirX,dirY)
            arrowind = 3
        else:
            # left
            move_motor(manualsteps,pulseX,'backward',dirX)
            arrowind = 2
    elif movement_dir[0] == 1:
        if movement_dir[1] == -1:
            # down right
            move_2_motors(manualsteps,pulseX,pulseY,'backward','forward',dirX,dirY)
            arrowind = 7
        elif movement_dir[1] == 1:
            # up right
            move_2_motors(manualsteps,pulseX,pulseY,'forward','forward',dirX,dirY)
            arrowind = 5
        else:
            # right
            move_motor(manualsteps,pulseX,'forward',dirX)
            arrowind = 6
    else:
        if movement_dir[1] == -1:
            # down
            move_motor(manualsteps,pulseY,'backward',dirY)
            arrowind = 0
        elif movement_dir[1] == 1:
            # up
            move_motor(manualsteps,pulseY,'forward',dirY)
            arrowind = 4
        else:
            # nothing
            arrowind = 100
    return arrowind

# HELPER FUNCTIONS CAMERA
def find_dark_spots(image, threshold):
    _,thresh = cv2.threshold(image,threshold,255,cv2.THRESH_BINARY_INV) # adjust the first value to get lighter/smaller spots - higher is more sensitive
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
        cam.BeginAcquisition()
        images = list()
        processor = PySpin.ImageProcessor()
        processor.SetColorProcessing(PySpin.SPINNAKER_COLOR_PROCESSING_ALGORITHM_HQ_LINEAR)
        trackinginitialized = 1
        return True

#### actual start of the program
# init the camera no matter what
if not trackinginitialized:
    ans = initialize_tracking()

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
            elif active:
                if event.key == pygame.K_RETURN:
                    # analyze inputted text
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
                    elif text == 'r':
                        # TURN ON RECORDING TO SAVE VIDS
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
        image_result = cam.GetNextImage(10) # was 1000 
        pixel_format = image_result.GetPixelFormatName()
        image_array = image_result.GetNDArray()   
        image_result.Release()
        image_rgb = cv2.cvtColor(image_array, cv2.COLOR_BGR2RGB)
        target_wid = 600
        target_hei = int((target_wid / image_rgb.shape[1])*image_rgb.shape[0])
        image = cv2.resize(image_rgb, (target_wid, target_hei))
        image = cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE)
        shi = image
    except:
        # print('didnt get new image')
        image = shi
        target_wid = 600
        target_hei = 600
    
     
    # for saving - do if recording on?
    # images.append(processor.Convert(image_result, PySpin.PixelFormat_Mono8)) # dont currently need b/c not making video / storing images like this

    if mode == 'manual':
        keys = pygame.key.get_pressed()
        movement_dir = get_dir(keys)
        arrowind = apply_move(movement_dir)
    else:
        # TRACKING *** JUST DO A WHITE LIGHT FIRST
        image, cx, cy = find_white_spot(image)
        icx = round(target_wid/2)
        icy = round(target_hei/2)
        center = (icx,icy)
        cv2.drawMarker(image,center,(0,0,255),markerType=cv2.MARKER_CROSS)
        
        if cx == 0 or cy == 0:
            # no light spot found
            pass
        else:
            err = [cy - icy, cx - icx] # maybe b/c I rotated x became y or something but this works so ok
            # print(err)
            if abs(err[0]) > 10:
                if err[0] < 10:
                    # left
                    move_motor(manualsteps,pulseX,'backward',dirX)
                    pass
                else:
                    # move motor R 
                    move_motor(manualsteps,pulseX,'forward',dirX)
                    pass
            if abs(err[1]) > 10:
                if err[1] > 10:
                    # move motor Down
                    move_motor(manualsteps,pulseY,'backward',dirY)
                    pass
                else:
                    # move motor Up 
                    move_motor(manualsteps,pulseY,'forward',dirY)
                    pass

    # Render the current text.
    txt_surface = font.render(text, True, color)
    help_surface = helptextfont.render(help_text, True, pygame.Color('white'))
    instructions_surfaces = [font.render(line, True, pygame.Color('white')) for line in instructions]
    input_surface = font.render(inputtxt, True, pygame.Color('white'))
    mode_surface = font.render(f'Current mode: {mode}',True,pygame.Color('green'))
    speed_surface = font.render(f'Current speed: {speed}',True,pygame.Color('green'))
    default = onoffX.is_lit
    motorxsur = font.render(f'X Motor: {xmotor}',True,pygame.Color('green'))
    motorysur = font.render(f'Y Motor: {ymotor}',True,pygame.Color('green'))

    # Resize the box if the text is too long.
    width = max(200, txt_surface.get_width()+10)
    input_box.w = width

    # Fill the screen with a color (RGB)
    screen.fill((30, 30, 30))
    # Blit the screen
    screen.blit(help_surface, (input_box.x, input_box.y - help_surface.get_height() - 10))
    screen.blit(input_surface,(input_box.x - input_surface.get_width() - 10, input_box.y + 6))
    screen.blit(txt_surface, (input_box.x+5, input_box.y+5))
    for idx, surf in enumerate(instructions_surfaces):
        screen.blit(surf, (instructions_pos[0], instructions_pos[1]+idx*45))
        if idx == len(instructions_surfaces)-1:
            screen.blit(mode_surface,(instructions_pos[0],instructions_pos[1]+idx*45+90))
            screen.blit(speed_surface,(instructions_pos[0],instructions_pos[1]+idx*45+135))
            screen.blit(motorxsur,(instructions_pos[0],instructions_pos[1]+idx*45+180))
            screen.blit(motorysur,(instructions_pos[0],instructions_pos[1]+idx*45+225))
    pygame.draw.rect(screen, color, input_box, 2)
    if arrowind != 100:
       screen.blit(arrows[arrowind],(10,10))
    
    isurf = pygame.surfarray.make_surface(image)
    screen.blit(isurf,(0,screen_height - target_hei))

    # Update the display
    pygame.display.flip()


# Quit Pygame / ending program ----------------------------
try:
    cam.DeInit()
    del cam
    cam_list.Clear()
    system.ReleaseInstance()
except:
    pass
pygame.quit()
sys.exit()
