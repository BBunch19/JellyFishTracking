from flask import Flask, request, jsonify
from gpiozero import LED
import time


app = Flask(__name__)

# Set up GPIO
pin = 21
led = LED(pin)

@app.route('/control', methods=['POST'])
def control_led():
    data = request.json
    state = data.get('state')
    clienttime = data.get('time')
    servertime = time.time()
    if state == 'on':
        led.on()
    elif state == 'off':
        led.off()
    return jsonify({
        'received_time': clienttime,
        'server_time': servertime
    })

@app.route('/shutdown', methods=['POST'])
def shutdown():
    led.off()
    return 'OK'

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)