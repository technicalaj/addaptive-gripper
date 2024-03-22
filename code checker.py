import cv2

# Define the callback function for the mouse event
def mouse_callback(event, x, y, flags, param):
    global hsv_frame, object_hsv
    if event == cv2.EVENT_LBUTTONDOWN:
        object_hsv = hsv_frame[y, x]
        print("Object HSV:", object_hsv)

# Create a video capture object for the camera
cap = cv2.VideoCapture(1)

# Create a window for displaying the video stream
cv2.namedWindow("Video Stream")

# Set the callback function for the mouse event
cv2.setMouseCallback("Video Stream", mouse_callback)

# Initialize the object HSV value
object_hsv = None

while True:
    # Read a frame from the video stream
    ret, frame = cap.read()

    # Convert the frame to the HSV color space
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Display the video stream in the window
    cv2.imshow("Video Stream", frame)

    # Check for the "q" key to exit the program
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the video capture object and close the window
cap.release()
cv2.destroyAllWindows()
