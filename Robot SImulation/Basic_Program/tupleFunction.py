import numpy as np

nilai_mediapipeX = 0.12
nilai_mediapipeY = 0.1

a = tuple(np.multiply([nilai_mediapipeX, nilai_mediapipeY], [640, 480]).astype(int))

print("Nilai Mediapipe X Y", nilai_mediapipeX, nilai_mediapipeY)
print("Nilai Multiplication", a[0])
