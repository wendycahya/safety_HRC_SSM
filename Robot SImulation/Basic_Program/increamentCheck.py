import numpy as np
dis = [0, 0, 0]
start = [1, 2, 3]
stop = [5, 5, 5]
def increament(start, stop):
    i = 0
    for i in range(0,3):
        dis[i]= stop[i] - start[i]
    print(dis)
    # while start <= stop:
    #     if start == stop:
    #         print("continue move")
    #         break
    #     start += 0.01
    #     print("generate number-", start)
    #     print("")

a = increament(start, stop)
print(a)
