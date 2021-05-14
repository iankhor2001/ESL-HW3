# ESL-HW3

///////////////////////////////////////////////////////

mlearning folder:
This is the codes I used to train my TF gestures.

program folder:
The main program.

///////////////////////////////////////////////////////

When loaded into the program, the board will try to connect to the pc host/python.
The process will not show to the user. This is a background process.
There will be a wifi thread always active with mqtt_queue. 
Inside mqtt_queue, the publish_message() function will be run in a infinite loop with If conditions inside.
When the global variable send_msg==1, the function will publish a message.
The content of the message is based on another global variable msg_mode.
There is a format for mqtt messages: "---<Action>---<Message>---" 
<Action> will tell Python what to do
<Message> is what will be shown to the User on PC.

Using the python file <rpc_function.py>, user can control the board using RPC functions.
There is two functions available. (Input: gesture/tilt)

Once a mode is selected, Python will send a RPC function through serial to the board.

If the user type "gesture" in Python:
The board will call gesture() in gesT thread (gesture_queue).
In this mode, moving the board up and down will change the selected maximum angle.
If the button is pressed, the board will send a mqtt message to Python;
after Python received the message, it will send a RPC message to board to close the Gesture UI.

If the user type "tilt" in Python:
The board will call tilt_wifi() in tiltT thread (tilt_queue).
In this mode, the board will take in the tilt angle of the board and send data to Python.
Initially, the board will wait 3 seconds for user to put the board in the desired angle (reference angle), and record the tilt angle with tilt_init().
Then it will start tilt_angle function to detect the angle relative to the reference angle.
If the tilt angle exceeds the maximum angle selected from the Gesture UI, the board will send a MQTT message to Python and record it.
Once the angle exceeds the maximum angle 5 times, it will send a MQTT message to Python.
Python will then send a RPC function to board to close the Tilt Angle UI.

All these information sent to Python from board will also be shown on the uLCD.

As for the LEDs:
This is the configuration [LED1,LED2]:
[1,0] : Gesture mode
[0,1] : Tilt Angle mode - Recording Angle
[1:1] : Tilt Angle mode - Initialize angle

LED3 is used to show message sending. LED3 will flip for every message sent.


