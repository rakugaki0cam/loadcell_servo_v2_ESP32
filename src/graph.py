
#graph

from matplotlib import pyplot as pyp
data =[]
x=[]
w=[]
for _, _, dx, dw in data:
    x.append(dx)
    w.append(dw)
pyp.plot(x, w)
pyp.show()

