import serial
import time
serdev = '/dev/ttyACM14'
s = serial.Serial(serdev, 9600)

def terminate_gesture():
    s.write(bytes("/gesture_terminate/run\r", 'UTF-8'))
    # line=s.readline() # Read an echo string from mbed terminated with '\n' (putc())
    # print(line)
    # line=s.readline() # Read an echo string from mbed terminated with '\n' (RPC reply)
    # print(line)
    time.sleep(1)

def activate_gesture():
    s.write(bytes("/gesture_activate/run\r", 'UTF-8'))
    time.sleep(1)

def terminate_tilt():
    s.write(bytes("/tilt_terminate/run\r", 'UTF-8'))
    time.sleep(1)

def activate_tilt():
    s.write(bytes("/tilt_activate/run\r", 'UTF-8'))
    time.sleep(1)
 
############################################################################################

s.write(bytes("\r", 'UTF-8'))
line=s.readline() # Read an echo string from mbed terminated with '\n' (putc())
print(line)
line=s.readline() # Read an echo string from mbed terminated with '\n' (RPC reply)
print(line)
time.sleep(1)
while(1):
    action = input('\nWhich mode? (gesture/tilt/exit) ')

    if action == 'gesture':
        gesture_action = input('Activate or Terminate? (a/t)\nIf terminate, press USER_BTN first! ')
        if gesture_action == 'a':
            activate_gesture()
        elif gesture_action == 't':
            terminate_gesture()

    elif action == 'tilt':
        tilt_action = input('Activate or Terminate? (a/t) ')
        if tilt_action == 'a':
            activate_tilt()
        elif tilt_action == 't':
            terminate_tilt()

    elif action == 'exit':
        print('Exiting Python')
        break
    
    else:
        print('Try again!\n')

s.close()
