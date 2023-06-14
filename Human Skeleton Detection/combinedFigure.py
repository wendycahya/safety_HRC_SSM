import numpy as np
import matplotlib.pyplot as plt
import argparse
import cv2

import cvzone
from cvzone.FaceMeshModule import FaceMeshDetector
from datetime import datetime
import csv

start_time = datetime.now()
start = datetime.now()
milliseconds = 0
write_file = "TR-"+str(start)+"-SSMNewDemo.csv"
d =0

#Device connection
fpsReader = cvzone.FPS()
cap = cv2.VideoCapture(0)
detector = FaceMeshDetector(maxFaces=1)

start_time = datetime.now()
start = datetime.now()
milliseconds = 0
write_file = "TR-"+str(start)+"-SSMNewDemo.csv"
d =0

#Device connection
fpsReader = cvzone.FPS()
cap = cv2.VideoCapture(0)
detector = FaceMeshDetector(maxFaces=1)


color = args['color']
bins = args['bins']
resizeWidth = args['width']

# Initialize plot.
fig, ax = plt.subplots()
par = ax.twinx()

ax.set_title('Collaborative Workspace Analysis')
ax.set_xlabel('time')
ax.set_ylabel('distance(mm)')
par.set_ylabel('speed (mm/s)')
# Initialize plot line object(s). Turn on interactive plotting and show plot.
lw = 3
alpha = 0.5
lineR, = ax.plot(np.arange(bins), np.zeros((bins,)), c='r', lw=lw, alpha=alpha, label='Red')
lineG, = ax.plot(np.arange(bins), np.zeros((bins,)), c='g', lw=lw, alpha=alpha, label='Green')
lineB, = ax.plot(np.arange(bins), np.zeros((bins,)), c='b', lw=lw, alpha=alpha, label='Blue')

ax.set_xlim(0, bins-1)
ax.set_ylim(0, 1)
ax.legend()
plt.ion()
plt.show()

# Grab, process, and display video frames. Update plot line object(s).
while True:
    success, img = cap.read()
    imgMesh, faces = detector.findFaceMesh(img, draw=False)
    fps, imgCap = fpsReader.update(img, pos=(20, 20), color=(0, 255, 0), scale=2, thickness=2)


    # Normalize histograms based on number of pixels per frame.
    cv2.imshow('RGB', img)
    lineR.set_ydata(d)
    lineG.set_ydata(Vr)

    fig.canvas.draw()

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()