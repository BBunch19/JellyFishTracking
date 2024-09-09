from gpiozero import LED, Button
from gpiozero.pins.rpigpio import RPiGPIOFactory
import time
import RPi.GPIO as GPIO
# from signal import pause

GPIO.setmode(GPIO.BCM)
factory = RPiGPIOFactory()

led = LED(4, pin_factory=factory)
switch = Button(17, pin_factory=factory)

while True:
    print("a")
    if switch.is_pressed:
        print('x')
        led.on()
    else:
        led.off()
    

# switch.when_pressed = led.on
# switch.when_released = led.off
# pause()