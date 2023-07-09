import cv2
import numpy as np


# Function to detect circles
def detect_circles(image):
    # Convert the image to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Apply Gaussian blur to reduce noise
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)

    # Detect circles using HoughCircles
    circles = cv2.HoughCircles(blurred, cv2.HOUGH_GRADIENT, dp=1, minDist=50, param1=50, param2=30, minRadius=10,
                               maxRadius=40)

    # If circles are found, return the circles
    if circles is not None:
        return np.round(circles[0, :]).astype("int")
    else:
        return []


# Function to track circles with bounding boxes
def track_circles(image, circles):
    for (x, y, r) in circles:
        # Draw a circle on the image
        cv2.circle(image, (x, y), r, (0, 255, 0), 2)

        # Draw a bounding box around the circle
        cv2.rectangle(image, (x - r, y - r), (x + r, y + r), (0, 255, 0), 2)

    # Display the image
    cv2.imshow("Circle Detection", image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


# Load the image
image = cv2.imread("torpedo.png")

# Convert the image to RGB
image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

# Detect circles
circles = detect_circles(image)

# Track circles with bounding boxes
track_circles(image, circles)
