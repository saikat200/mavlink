import cv2

# Load the video
cap = cv2.VideoCapture('video.avi')

# Initialize the tracker
tracker = cv2.TrackerMeanShift_create()


# Read the first frame
ret, frame = cap.read()

# Detect the object using an object detection model
# and obtain the bounding box coordinates
bbox = detect_object(frame)

# Calculate the centroid of the bounding box
x, y, w, h = bbox
cx, cy = x + w/2, y + h/2

# Initialize the tracker with the ROI around the centroid
roi = (x, y, w, h)
tracker.init(frame, roi)

# Loop over all frames in the video
while True:
    # Read the next frame
    ret, frame = cap.read()

    # Check if the end of the video has been reached
    if not ret:
        break

    # Update the tracker with the current frame
    ret, roi = tracker.update(frame)

    # If tracking is successful, update the centroid
    if ret:
        x, y, w, h = roi
        cx, cy = x + w/2, y + h/2

    # Draw the bounding box and centroid on the frame
    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
    cv2.circle(frame, (int(cx), int(cy)), 5, (0, 0, 255), -1)

    # Display the frame
    cv2.imshow('frame', frame)

    # Exit the loop if 'q' is pressed
    if cv2.waitKey(1) == ord('q'):
        break

# Release the video and close all windows
cap.release()
cv2.destroyAllWindows()
