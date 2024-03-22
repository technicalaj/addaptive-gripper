import cv2
import serial
import time

ser = serial.Serial('COM3', 9600)  # replace COM3 with the port used by your Arduino board
cap = cv2.VideoCapture(0)

# Define the color range of the blue object you want to track
lower_color = (20, 100, 100)  # lower bound of the yellow object color in HSV
upper_color = (30, 255, 255)  # upper bound of the yellow object color in HSV

# Define the center threshold (percentage of the frame width)
center_threshold = 0.1

# Define the minimum time the object should be centered (in seconds)
min_center_time = 1.0

# Initialize the time the object has been centered
centered_time = 0.0

# Initialize the previous center coordinates
prev_cx = -1
prev_cy = -1

# Initialize a variable to keep track of whether the command has been sent or not
command_sent = False

while True:
    ret, frame = cap.read()
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Create a mask to detect the blue object within the specified color range
    mask = cv2.inRange(hsv_frame, lower_color, upper_color)

    # Find the contours of the blue object
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Find the largest contour and calculate its center
    if contours:
        max_contour = max(contours, key=cv2.contourArea)
        moments = cv2.moments(max_contour)
        if moments['m00'] != 0:
            cx = int(moments['m10'] / moments['m00'])
            cy = int(moments['m01'] / moments['m00'])

            cv2.drawContours(frame, [max_contour], 0, (0, 255, 0), 2)  # Draw the contour
            
            cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)

            # Calculate the percentage of the object's x-coordinate relative to the frame width
            object_percentage = float(cx) / float(frame.shape[1])

            # Check if the object is centered
            if abs(0.5 - object_percentage) <= center_threshold:
                if cx != prev_cx or cy != prev_cy:
                    centered_time += 1.0 / cap.get(cv2.CAP_PROP_FPS)
                    if centered_time >= min_center_time and not command_sent:
                        ser.write(b"1\n") # send command to rotate LED motor
                        command_sent = True
                    else:
                        ser.write(b"0\n") # send command to stop LED motor
    
            else:
                centered_time = 0.0
                if not command_sent:
                    ser.write(b"2\n") # send command to move motor to a random position
                    command_sent = True

            # Update the previous center coordinates
            prev_cx = cx
            prev_cy = cy

        else:
            if not command_sent:
                ser.write(b"2\n") # send command to move motor to a random position
                command_sent = True

    cv2.imshow('frame', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows
