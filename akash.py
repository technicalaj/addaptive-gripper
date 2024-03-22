import cv2
import serial

ser = serial.Serial('COM7', 9600)  # replace COM3 with the port used by your Arduino board
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    # process the frame to get the servo angles
    # ...

    # send the servo angles over serial communication
    ser.write(f"{80} {0} {60} {90}\n".encode())

    cv2.imshow('frame', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
