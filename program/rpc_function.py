import serial
import paho.mqtt.client as paho
import time
import threading
serdev = '/dev/ttyACM0'
s = serial.Serial(serdev, 9600)
msg_mode = 1

def terminate_gesture():
    s.write(bytes("/gesture_terminate/run\r", 'UTF-8'))
    time.sleep(1)

def activate_gesture():
    msg_mode = 0
    print("Opening Gesture UI")
    s.write(bytes("/gesture_activate/run\r", 'UTF-8'))
    time.sleep(1)

def terminate_tilt():
    s.write(bytes("/tilt_terminate/run\r", 'UTF-8'))
    time.sleep(1)

def activate_tilt():
    msg_mode = 0
    print("Opening Tilt UI")
    s.write(bytes("/tilt_activate/run\r", 'UTF-8'))
    time.sleep(1)

###############################################################################################

mqttc = paho.Client()
host = "192.168.0.103"
topic = "Mbed"

def on_connect(self, mosq, obj, rc):
    print("Connected rc: " + str(rc))

def on_message(mosq, obj, msg):
    # print('msg:'+msg.topic)
    split = str(msg.payload).split('---')
    print('Msg: '+split[2])
    if len(split)>1:
        if(split[1]=='close gesture'):
            time.sleep(1)
            print('Closing Gesture UI')
            terminate_gesture()
            msg_mode=1
            print('\nWhich mode? (gesture/tilt/exit) ')
        elif(split[1]=='close tilt'):
            time.sleep(1)
            print('Closing Tilt Detection UI')
            terminate_tilt()
            msg_mode=1
            print('\nWhich mode? (gesture/tilt/exit) ')
    else:
        print(split[0])

def on_subscribe(mosq, obj, mid, granted_qos):
    print("Subscribed OK")

def on_unsubscribe(mosq, obj, mid, granted_qos):
    print("Unsubscribed OK")

# Set callbacks
mqttc.on_message = on_message
mqttc.on_connect = on_connect
mqttc.on_subscribe = on_subscribe
mqttc.on_unsubscribe = on_unsubscribe

# Connect and subscribe
print("Connecting to " + host + "/" + topic)
mqttc.connect(host, port=1883, keepalive=60)
mqttc.subscribe(topic, 0)

def mqtt_loop():
    # Loop forever, receiving messages
    mqttc.loop_forever()

mqtt_thread = threading.Thread(target=mqtt_loop, daemon=True)
mqtt_thread.start()
############################################################################################

s.write(bytes("\r", 'UTF-8'))
line=s.readline() # Read an echo string from mbed terminated with '\n' (putc())
print(line)
line=s.readline() # Read an echo string from mbed terminated with '\n' (RPC reply)
print(line)
time.sleep(1)

while(1):
    action = input('\nWhich mode? (gesture/tilt/exit) \n')
    if action == 'gesture':
            activate_gesture()
            msg_mode=0

    elif action == 'tilt':
            activate_tilt()
            msg_mode=0

    elif action == 'exit':
        print('Exiting Python')
        break
    else:
        print('Try again!\n')

s.close()
