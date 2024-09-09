from gpiozero import LED
import time

pin = 21
led = LED(pin)

num = 100000

start_time = time.time()

# for _ in range(num):
#     led.on()
#     led.off()

end_time = time.time()

et = end_time-start_time
tr = num / et

print(f"ur : {tr:.2f} Hz")
led.on()
time.sleep(1)
led.off()