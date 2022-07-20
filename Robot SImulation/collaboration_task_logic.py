import numpy as np

#initial workspace [0,0,0,0]
#algorithm
#looping until condition similar array
#1. sensor read the object position
#2. check array value
#3. order task
#4. check the value

start = [0, 0, 0, 0]
#progress = [A, B, C, D]
progress = [0, 1, 1, 1]
finish = [1, 1, 1, 1]
x = 0

while True:
    print("reading the condition", progress)
#condition A belum terisi
    if (progress==[0,0,0,0]):
        progress[0] = 1
        print("robot mengisi progress[0]", progress)
    elif (progress == [0, 1, 0, 0]):
        progress[0] = 1
        print("robot mengisi progress[0]", progress)
    elif (progress == [0, 0, 1, 0]):
        progress[0] = 1
        print("robot mengisi progress[0]", progress)
    elif (progress == [0, 0, 0, 1]):
        progress[0] = 1
        print("robot mengisi progress[0]", progress)
    elif (progress == [0, 1, 1, 0]):
        progress[0] = 1
        print("robot mengisi progress[0]", progress)
    elif (progress == [0, 1, 0, 1]):
        progress[0] = 1
        print("robot mengisi progress[0]", progress)
    elif (progress == [0, 0, 1, 1]):
        progress[0] = 1
        print("robot mengisi progress[0]", progress)
    elif (progress == [0, 1, 1, 1]):
        progress[0] = 1
        print("robot mengisi progress[0]", progress)
# condition B belum terisi
    elif (progress == [1, 0, 0, 0]):
        progress[1] = 1
        print("robot mengisi progress[1]", progress)
    elif (progress == [1, 0, 1, 0]):
        progress[1] = 1
        print("robot mengisi progress[1]", progress)
    elif (progress == [1, 0, 0, 1]):
        progress[1] = 1
        print("robot mengisi progress[1]", progress)
    elif (progress == [1, 0, 1, 1]):
        progress[1] = 1
        print("robot mengisi progress[1]", progress)
# condition C belum terisi
    elif (progress == [1, 1, 0, 0]):
        progress[2] = 1
        print("robot mengisi progress[2]", progress)
    elif (progress == [1, 1, 0, 1]):
        progress[2] = 1
        print("robot mengisi progress[2]", progress)
#condition D belum terisi
    elif (progress == [1, 1, 1, 0]):
        progress[3] = 1
        print("robot mengisi progress[3]", progress)

    if(progress == finish):
        print("Robot Stop\n")
        break