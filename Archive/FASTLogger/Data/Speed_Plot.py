import numpy as np
import matplotlib.pyplot as plt
from pdf import *

pp = PDF(0,plt)

####import C file
C = open('Bench_Test_C++_7_2_2021_9.31AM.txt')
print(np.shape(C))

####import python file
Py= open('Bench_Test_Python_7_2_2021_9.32AM.txt')
delimeter  = ','
print(np.shape(Py))

iteration = np.arange(1000)
C_Loop = []
Py_Loop = []

lenfile = 0

for line in C: 
	lenfile += 1
	row = line.split(delimeter)
	C_Loopc = np.float(row[1])
	

	C_Loop.append(C_Loopc)

lenfile = 0 

for line in Py: 
	lenfile += 1 
	row = line.split(delimeter) 
	Py_Loopc = np.float(row[1])

	Py_Loop.append(Py_Loopc)

C_Loop = np.asarray(C_Loop)
Py_Loop = np.asarray(Py_Loop)

fig = plt.figure()
plt.plot(iteration,C_Loop, label = 'C++')
plt.plot(iteration, Py_Loop, label = 'Python')
plt.xlabel('Iteration')
plt.ylabel('Time')
plt.legend()
plt.grid()
pp.savefig()

pp.close()