import matplotlib.pyplot as plt
import ast

f = open('centerError','r')
i = 0
j = 0
t1 = []
centerError = []
while i != '':
    i = f.readline(-1)
    if i != '':
	i = ast.literal_eval(i)
	centerError.append(i)
i = 0
for i in centerError:
    t1.append(j)
    j = j+1
for i in t1:
    if centerError[i] == 320:
	centerError[i] = 0

f.close()
plt.plot(t1,centerError)
plt.title('Center Error')
plt.ylabel('Distance from Center (pixels)')
plt.xlabel('Time (ms)')

plt.show()
