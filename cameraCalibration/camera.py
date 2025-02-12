import cv2
import time

# Open the camera (default camera is usually at index 0)
cap = cv2.VideoCapture(2)

if not cap.isOpened():
    print("Error: Could not open video capture device.")
    exit()

# Set up a loop to capture a photo every second
while True:
    # Capture a frame from the camera
    ret, frame = cap.read()

    if not ret:
        print("Failed to capture image.")
        break

    # Get the current timestamp to use in the file name
    timestamp = time.strftime("%Y%m%d-%H%M%S")

    # Save the captured image with a timestamp as the filename
    filename = f"photo_{timestamp}.jpg"
    cv2.imwrite(filename, frame)

    # Wait for 1 second before capturing the next photo
    time.sleep(1)

# Release the camera when done
cap.release()
cv2.destroyAllWindows()
