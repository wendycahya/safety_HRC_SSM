import cv2
import datetime as dt
import numpy as np
import random

# Create empty lists for x and y data
xs = []
ys = []


# Create a blank image with white background
image = np.ones((480, 640, 3), dtype=np.uint8) * 255

# Create a named window for displaying the image
cv2.namedWindow('Real-time Plot', cv2.WINDOW_NORMAL)

# This function is called periodically
def update_plot():
    # Clear the image
    image.fill(255)

    # Read temperature (Celsius) from TMP102
    temp_c = round(random.randint(0, 9), 2)

    # Add current time and temperature to lists
    xs.append(dt.datetime.now().strftime('%H:%M:%S.%f'))
    ys.append(temp_c)

    # Limit x and y lists to 20 items
    xs = xs[-20:]
    ys = ys[-20:]

    # Draw lines connecting the data points
    for i in range(len(xs) - 1):
        cv2.line(image, (i * 30, 400 - int(ys[i] * 40)), ((i + 1) * 30, 400 - int(ys[i + 1] * 40)), (0, 0, 0), 2)

    # Display the image in the window
    cv2.imshow('Real-time Plot', image)

    # Wait for a key press and check if 'q' is pressed to exit
    if cv2.waitKey(1) == ord('q'):
        cv2.destroyAllWindows()
        return False

    return True

# Loop to update the plot periodically
while update_plot():
    pass