import serial
import time
from frame_catcher import Ball_handler
import cv2
import struct

def nothing(x):
    pass

bh = Ball_handler()
## track bar after ball_handler init
cv2.createTrackbar("Kc_x","pid_menu", 0, 1000, nothing)
cv2.createTrackbar("Ki_x","pid_menu", 0, 1000, nothing)
cv2.createTrackbar("Kd_x","pid_menu", 0, 1000, nothing)

cv2.createTrackbar("Kc_y","pid_menu", 0, 1000, nothing)
cv2.createTrackbar("Ki_y","pid_menu", 0, 1000, nothing)
cv2.createTrackbar("Kd_y","pid_menu", 0, 1000, nothing)


# here you should add your camera ip address
cam_address = "http://192.168.ip:port/video" # mjpegfeed?640x480

# usb port for serial communication
PORT = 'COM13' # Replace 'COM3' with your Arduino's port

# Open the serial port
arduino = serial.Serial(PORT, 115200)  

print(f"starting the communication process for {PORT}")

time.sleep(2) # 2 seconds delay

data = 0

cap = cv2.VideoCapture(cam_address)

mouse_pos = [0, 0]
default_pos = [2000, 2000]

def mouse_click(event, x, y, flags, param):
    global mouse_pos
    if event == cv2.EVENT_LBUTTONDOWN:
        mouse_pos[0] = x
        mouse_pos[1] = y

cv2.setMouseCallback("Processed Frame", mouse_click)

while True:

    t1 = time.time()
    
    response = arduino.read().decode()
    print(f"Arduino: {response}")

    ret, frame = cap.read()

    if not ret:
        print("Failed to grab frame")
        break

    ball_cneter = bh.ball_finder(frame, mouse_pos)

    print(f"@@@@ this is x: {mouse_pos[0]}, this is y: {mouse_pos[1]} @@@")

    if (cv2.getTrackbarPos("on/off", "pid_menu") == 1):

        arduino.write(b''.join([ball_cneter[0].to_bytes(2, 'big'), ball_cneter[1].to_bytes(2, 'big'), mouse_pos[0].to_bytes(2, 'big'), mouse_pos[1].to_bytes(2, 'big')]))

    else:

        arduino.write(b''.join([default_pos[0].to_bytes(2, 'big'), default_pos[1].to_bytes(2, 'big'), mouse_pos[0].to_bytes(2, 'big'), mouse_pos[1].to_bytes(2, 'big')]))

    
    Kc_x = cv2.getTrackbarPos("Kc_x","pid_menu")/1000
    Kd_x = cv2.getTrackbarPos("Kd_x","pid_menu")/1000
    Ki_x = cv2.getTrackbarPos("Ki_x","pid_menu")/10000
    
    Kdb_x = struct.pack("f", Kd_x) 
    Kib_x = struct.pack("f", Ki_x) 
    Kcb_x = struct.pack("f", Kc_x) 

    arduino.write(Kdb_x)
    arduino.write(Kib_x)
    arduino.write(Kcb_x)


    Kc_y = cv2.getTrackbarPos("Kc_y","pid_menu")/1000
    Kd_y = cv2.getTrackbarPos("Kd_y","pid_menu")/1000
    Ki_y = cv2.getTrackbarPos("Ki_y","pid_menu")/10000
    
    Kdb_y = struct.pack("f", Kd_y) 
    Kib_y = struct.pack("f", Ki_y) 
    Kcb_y = struct.pack("f", Kc_y) 

    arduino.write(Kdb_y)
    arduino.write(Kib_y)
    arduino.write(Kcb_y)

    if cv2.waitKey(1) & 0xFF == ord('q'): # 5
        break

    t2 = time.time()
    
    if (t2 - t1) != 0:
        print(f"data sending rate: {1/(t2 - t1)} per sec")


cap.release()
cv2.destroyAllWindows()
arduino.close()
