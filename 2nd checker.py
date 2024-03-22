import cv2
import numpy as np
import math
import serial
import time

# Initialize serial communication with the Arduino
ser = serial.Serial('COM3', 9600) # replace COM3 with the name of the name of the serial port on your computer

# Initialize the servos
x_angle = 90 # initial angle for X axis servo
y_angle = 90 # initial angle for Y axis servo
z_angle = 90 # initial angle for Z axis servo

# Define the arm parameters (length of each link in cm)
L1 = 180
L2 = 180

# Define the forward kinematics function
def forward_kinematics(theta1, theta2, theta3):
    x = L1*math.cos(math.radians(theta1)) + L2*math.cos(math.radians(theta1+theta2)) + L3*math.cos(math.radians(theta1+theta2+theta3))
    y = L1*math.sin(math.radians(theta1)) + L2*math.sin(math.radians(theta1+theta2)) + L3*math.sin(math.radians(theta1+theta2+theta3))
    z = L4
    return (x, y, z)

def inverse_kinematics(x, y, z):
    # Calculate theta1 using the inverse tangent function
    theta1 = math.degrees(math.atan2(y, x))

    # Calculate the distance from the base to the end effector in the XY plane
    r = math.sqrt(x**2 + y**2)

    # Calculate the angle between the second link and the XY plane
    phi = math.atan2(z - L1, r)

    # Calculate the distance from the base to the end effector in 3D space
    d = math.sqrt((z - L1)**2 + r**2)

    # Calculate the angles for the second and third links using the law of cosines
    c2 = (L2**2 - L1**2 - d**2) / (-2*L1*d)

    if abs(c2) > 1:
        print("Target position is outside of reachable workspace!")
        return None

    theta2 = math.degrees(math.atan2(math.sqrt(max(0,1-c2**2)), c2) - phi)
    theta3 = math.degrees(math.atan2(z-L1-r*math.cos(math.radians(theta2))), r*math.sin(math.radians(theta2)))

    return (theta1, theta2, theta3)




# Capture video from the camera
cap = cv2.VideoCapture(1)

# Set the camera resolution
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# Define the HSV color range for the blue object to be detected
lower_color = np.array([90, 70, 50])
upper_color = np.array([130, 255, 255])



# Create a loop to detect and track the object in real-time
while True:
    # Read the frame from the camera
    ret, frame = cap.read()

    # Convert the frame to the HSV color space
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Create a mask to filter out the object based on the defined HSV color range
    mask = cv2.inRange(hsv_frame, lower_color, upper_color)

    # Find the contours of the object in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
       
    # Iterate over the contours and draw a bounding box around the object
    max_area = 0
    max_contour = None
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > max_area:
            max_area = area
            max_contour = contour

    if max_contour is not None:
        # Get the centroid of the object
        moments = cv2.moments(max_contour)
        cx = int(moments['m10'] / moments['m00'])
        cy = int(moments['m01'] / moments['m00'])

        # Display the centroid on the frame
        cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)

        # Convert the centroid coordinates to arm coordinates
        x = cx - 320
        y = cy - 240
        z = 0

        # Calculate the servo angles required to move the end effector to the desired position using inverse kinematics
        theta1, theta2, theta3 = inverse_kinematics(x, y, z)

        # Send the servo angles to the servos using the Arduino board
        ser.write(str(int(x_angle+theta1)).encode('utf-8'))
        time.sleep(0.1)
        ser.write(str(int(y_angle+theta2)).encode('utf-8'))
        time.sleep(0.1)
        ser.write(str(int(z_angle+theta3)).encode('utf-8'))

        # Update the servo angles
        x_angle += theta1
        y_angle += theta2
        z_angle += theta3

    # Display the frame
    cv2.imshow('frame', frame)

    # Wait for a key press and exit the loop if the "q" key is pressed
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break

# Release the resources
cap.release()
cv2.destroyAllWindows()
ser.close()

