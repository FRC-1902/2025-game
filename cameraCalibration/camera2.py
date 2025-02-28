import cv2
import time

# Initialize the camera (0 is usually the default camera)
camera = cv2.VideoCapture(2)

if not camera.isOpened():
    print("Error: Could not open camera.")
    exit()

# Initialize an image counter
image_counter = 1

try:
    last_capture_time = time.time()

    while True:
        # Capture frame-by-frame
        ret, frame = camera.read()
        if not ret:
            print("Error: Could not read frame.")
            break

        # Resize the frame to 800x600
        frame_resized = cv2.resize(frame, (1280, 720))

        # Display the resulting frame
        cv2.imshow('Webcam Feed', frame_resized)

        # Check if it's time to capture a new image
        current_time = time.time()
        if current_time - last_capture_time >= 1:
            # Save the frame with a sequential filename
            filename = f"{image_counter}.jpg"
            cv2.imwrite(filename, frame_resized)
            print(f"Captured: {filename}")

            # Increment the image counter
            image_counter += 1

            last_capture_time = current_time  # Update the last capture time

        # Check if the 'q' key is pressed to exit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("Exiting by user request.")
            break

except KeyboardInterrupt:
    print("Interrupted by user, exiting...")

finally:
    camera.release()
    cv2.destroyAllWindows()
