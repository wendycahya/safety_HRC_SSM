import cv2
import matplotlib.pyplot as plt
import numpy as np

# Initialize webcam
cap = cv2.VideoCapture(0)

# Create a figure and axes for live plotting
fig, ax = plt.subplots()

# Create an empty list to store data for plotting
data = []

# Function to update the plot
def update_plot():
    ax.clear()
    ax.plot(data)
    plt.axis('on')  # Turn off axis labels and ticks
    plt.xlabel("Time")
    plt.tight_layout()  # Adjust the plot to remove any padding
    plt.savefig('temp_plot.png')  # Save the plot as an image

# Create windows for video stream and plot
cv2.namedWindow('Video Stream')
cv2.namedWindow('Data Analysis')

# Main loop
while True:
    # Read video frame from webcam
    ret, frame = cap.read()

    # Simulate data generation (replace this with your own data source)
    # Here, we generate random data and append it to the list
    # Replace this with your actual variable that you want to plot
    import random
    new_data = random.randint(0, 100)
    data.append(new_data)

    # Update the plot
    update_plot()

    # Load the saved plot image
    plot_img = cv2.imread('temp_plot.png', cv2.IMREAD_UNCHANGED)

    # Resize the plot image to match the video frame size
    plot_img = cv2.resize(plot_img, (frame.shape[1], frame.shape[0]))

    # Display the video frame in the 'Video Stream' window
    cv2.imshow('Video Stream', frame)

    # Display the plot in the 'Live Plot' window
    cv2.imshow('Live Plot', plot_img[:, :, :3])

    # Check for 'q' key press to exit the loop
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the webcam and close all windows
cap.release()
cv2.destroyAllWindows()
