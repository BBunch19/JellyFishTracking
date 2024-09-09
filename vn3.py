from gpiozero import LED
import time
print('yre')
led = LED(4)
print('y')

try:
    while True:
        led.on()
        time.sleep(1)
        led.off()
        time.sleep(1)
except KeyboardInterrupt:
    led.off()

# import pigpio
# pi = pigpio.pi('soft',8888)
# if not pi.connected:
#     print('fao')
#     exit()

# pi.set_mode(4,pigpio.OUTPUT)
