from adafruit_servokit import ServoKit
from time import sleep
import time
import numpy as np
from picamera2 import Picamera2
import cv2

# Initialize the servo on PCA9685 board
kit = ServoKit(channels=16)
servo_h = kit.servo[1]
servo_v = kit.servo[0]

# set up camera
picam2 = Picamera2()
picam2.preview_configuration.main.size = (640, 480)
picam2.preview_configuration.main.format = "RGB888"
picam2.configure("preview")
picam2.start()

# set up video recorder
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('output.avi', fourcc, 20.0, (640, 480))

# load haar cascade for face detection
face_cascade = cv2.CascadeClassifier("haarcascade_frontalface_default.xml")

MAX_ANGLE = 160
MIN_ANGLE = 20
def set_angle(servo, angle):
    # handle edge cases
    if (angle > MAX_ANGLE): angle = MAX_ANGLE
    elif (angle < MIN_ANGLE): angle = MIN_ANGLE
    # update servo angle
    servo.angle = angle

def face_compare(f1, f2):
    x1, y1, w1, h1 = f1
    x2, y2, w2, h2 = f2

    value = 0
    value += abs(x1 - x2) ** 2
    value += abs(y1 - y2) ** 2
    value += (3 * abs(w1 - w2)) ** 2
    value += (3 * abs(h1 - h2)) ** 2

    return np.sqrt(value)


angle_h = 90
angle_v = 120
# Main program loop
try:
    # record previous face values
    prev_face = None

    while True:
        # display the frame
        frame = picam2.capture_array() # get frame
        frame = cv2.flip(frame, 0) # flip frame

        # convert to grayscale for haar cascade
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # detect faces
        faces = face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5)

        # if detected face
        if len(faces) > 0:
            # update previous face if there was none
            if prev_face is None: prev_face = faces[np.argmax(faces[:, 2])]

            # identify which current face is the previous face
            comparisons = [face_compare(prev_face, face) for face in faces]
            if (np.min(comparisons) > 300): # prevent big skips
                index = None
                prev_face = None

            else:
                index = np.argmin(comparisons)
            # draw recatangles around each face
            for i, (x, y, w, h) in enumerate(faces):
                color = (0, 255, 0) if i == index else (0, 0, 255)
                cv2.rectangle(frame, (x, y), (x+w, y+h), color, 3)


            if (index is not None):
                # update previous face to new face
                prev_face = faces[index]

                x, y, w, h = prev_face

                # compute center of face
                face_x = x + w // 2
                face_y = y + h // 2

                # draw center point
                cv2.circle(frame, (face_x, face_y), 5, (255, 255, 255), -1)

                    
                r = 0.05 # percent of dead zone
                da = 15 # increment of max angle change
                screen_x, screen_y = (640, 480) # screen pixels

                # Check if Horizontal Angle needs update
                x_dif = abs(face_x - screen_x / 2) / (screen_x / 2) # fraction of uncentered
                    
                if (face_x < (1 - r) * screen_x / 2):
                    # print('decr')
                    angle_h -= da * x_dif
                    if (angle_h < MIN_ANGLE): angle_h = MIN_ANGLE
                elif (face_x > (1 + r) * screen_x / 2):
                    # print('incr')
                    angle_h += da * x_dif
                    if (angle_h > MAX_ANGLE): angle_h = MAX_ANGLE

                # Check if Vertical Angle needs update
                y_dif = abs(face_y - screen_y / 2) / (screen_y / 2) # fraction of uncentered
                    
                if (face_y < (1 - r) * screen_y / 2):
                    # print('decr')
                    angle_v += da * y_dif
                    if (angle_v > MAX_ANGLE): angle_v = MAX_ANGLE
                elif (face_y > (1 + r) * screen_y / 2):
                    # print('incr')
                    angle_v -= da * y_dif
                    if (angle_v < MIN_ANGLE): angle_v = MIN_ANGLE
        
        else:
            prev_face = None


            

        cv2.imshow("Camera", frame) # display frame
        out.write(frame) # save frame for recording

        set_angle(servo_h, angle_h) # set angle of horizontal servo
        set_angle(servo_v, angle_v) # set angle of vertical servo

        if cv2.waitKey(1) == ord('q'):
            break
        
except KeyboardInterrupt:
    print("Program stopped by user")


# handle termination of video recorder and windows
out.release()  
cv2.destroyAllWindows()
