import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages

data = np.loadtxt('C:\Users\sdozi\OneDrive\Documents\E Sail\E Sail Documents\ESail Test Data\Data_27_June_2019\Pendulum_Test_1_1_July_2019.TXT')

time = data[:,0]/1000

ax9 = data[:,1]

ay9 = data[:,2]

az9 = data[:,3]

ax10 = data[:,4]

ay10 = data[:,5]

az10 = data[:,6]

print('Generating Plots')

pdfhandle = PdfPages('Pendulum_Test_1_1_July_2019.pdf')

plt.figure(1)
plt.plot(time,data[:,1], label = '9DOF',color = 'r')
plt.plot(time, data[:,4], label = '10DOF',color = 'b')
plt.title('9DOF and 10DOF Acceleration in x')
plt.xlabel('Time (s)')
plt.ylabel('Acceleration (m/s^2)')
plt.legend()
plt.show()
pdfhandle.savefig()

plt.figure(2)
plt.plot(time,data[:,2], label = '9DOF',color = 'r')
plt.plot(time,data[:,5], label = '10DOF',color = 'b')
plt.title('9DOF and 10DOF Acceleration in y')
plt.xlabel('Time (s)')
plt.ylabel('Acceleration (m/s^2)')
plt.legend()
plt.show()
pdfhandle.savefig()

plt.figure(3)
plt.plot(time,data[:,3], label = '9DOF',color = 'r')
plt. plot(time,data[:,6], label = '10DOF',color = 'b')
plt.title('9DOF and 10DOF Acceleration in z')
plt.xlabel('Time (s)')
plt.ylabel('Acceleration (m/s^2)')
plt.legend()
plt.show()
pdfhandle.savefig()
    
pdfhandle.close()