import cv2
import numpy as np
import sys
from PyQt5 import QtCore, QtWidgets, QtGui
from PyQt5.QtCore import pyqtSignal
import datetime


class ShowVideo(QtCore.QObject):
    camera_port = 0
    camera = cv2.VideoCapture(camera_port)
    face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')
    VideoSignal = QtCore.pyqtSignal(QtGui.QImage)

    def startVideo(self):
        run_video = True
        while run_video:
            ret, image = self.camera.read()
            face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')

            faces = face_cascade.detectMultiScale(image, 1.3, 5)
            date = datetime.datetime.now()
            cv2.putText(image, str(date), (370, 470), cv2.FONT_ITALIC, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

            cv2.putText(image, "By FALCON TUNISIA", (15, 470), cv2.FONT_ITALIC, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
            for (x, y, w, h) in faces:
                cv2.rectangle(image, (x, y), (x + w, y + h), (255, 255, 255), 5)
            # cv2.imshow('frame',image)

            color_swapped_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            height, width, _ = color_swapped_image.shape
            qt_image = QtGui.QImage(color_swapped_image.data, width, height, color_swapped_image.strides[0],
                                    QtGui.QImage.Format_RGB888)
            self.VideoSignal.emit(qt_image)


class ImageViewer(QtWidgets.QWidget):
    def paintEvent(self, event):
        painter = QtGui.QPainter(self)
        painter.drawImage(0, 0, self.image)

    def setImage(self, image):
        if image.isNull():
            print("Viewer Dropped frame!")
        self.image = image
        if image.size() != self.size():
            self.setFixedSize(image.size())
        self.update()


if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    thread = QtCore.QThread()
    thread.start()
    vid = ShowVideo()
    vid.moveToThread(thread)
    image_viewer = ImageViewer()
    vid.VideoSignal.connect(image_viewer.setImage)
    layout_widget = QtWidgets.QWidget()
    # start
    push_button1 = QtWidgets.QPushButton('Start')
    push_button1.clicked.connect(vid.startVideo)
    # calendr
    calendarWidget = QtWidgets.QCalendarWidget()
    calendarWidget.setGeometry(QtCore.QRect(180, 50, 411, 171))
    calendarWidget.setStyleSheet("alternate-background-color: rgb(204, 204, 204);\n"
                                 "background-color: rgb(177, 177, 177);")
    calendarWidget.setObjectName("calendarWidget")
    # symbole
    labelImg = QtWidgets.QLabel()
    labelImg.setPixmap(QtGui.QPixmap("test.png"))
    labelImg.setObjectName("labelImg")

    vertical_layout = QtWidgets.QVBoxLayout()
    layout_widget.setLayout(vertical_layout)
    vertical_layout.addWidget(labelImg)
    vertical_layout.addWidget(image_viewer)
    vertical_layout.addWidget(push_button1)
    vertical_layout.addWidget(calendarWidget)

    main_window = QtWidgets.QMainWindow()
    main_window.setCentralWidget(layout_widget)
    main_window.show()
    sys.exit(app.exec_())
    layout_widget.show()