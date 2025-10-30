"""Testing AHRS.py with Lab data
Aramis Hoffmann
4/30/25"""

import numpy as np
import matplotlib.pyplot as plt
import AHRS

data = np.loadtxt("LABTEST_4_25_25.txt", delimiter=",")
t = data[:,0]
tclip = 1.0
ax = data[:,1]
ax = ax[t>tclip]
ay = data[:,2]
ay = ay[t>tclip]
az = data[:,3]
az = az[t>tclip]
gx = data[:,4]
gx = gx[t>tclip]
gy = data[:,5]
gy = gy[t>tclip]
gz = data[:,6]
gz = gz[t>tclip]
mx = data[:,7]
mx = mx[t>tclip]
my = data[:,8]
my = my[t>tclip]
mz = data[:,9]
mz = mz[t>tclip]
t = t[t>tclip]

plt.figure()
plt.plot(t,ax,label='ax')
plt.plot(t,ay,label='ay')
plt.plot(t,az,label='az')
plt.legend()
plt.grid()
plt.xlabel('Time (sec)')
plt.ylabel('Acceleration (m/s^2')

plt.figure()
plt.plot(t,gx,label='gx')
plt.plot(t,gy,label='gy')
plt.plot(t,gz,label='gz')
plt.legend()
plt.grid()
plt.xlabel('Time (sec)')
plt.ylabel('Gyro (rad/s)')

plt.figure()
plt.plot(t,mx,label='mx')
plt.plot(t,my,label='my')
plt.plot(t,mz,label='mz')
plt.legend()
plt.grid()
plt.xlabel('Time (sec)')
plt.ylabel('Magnetic Field (Gauss)')


angle_finder = AHRS.AHRS()
angle_finder.AHRS()
roll = np.zeros(len(t))
pitch = np.zeros(len(t))
yaw = np.zeros(len(t))
roll_ = np.zeros(len(t))
pitch_ = np.zeros(len(t))
yaw_ = np.zeros(len(t))
rpy = [0,0,0]
for i in range(1,len(t)):
    phi_ = np.arctan(ay[i]/az[i])
    theta_ = np.arctan(-ax[i]/(ay[i]*np.sin(phi_)+az[i]*np.cos(phi_)))
    psi_ = np.arctan((mz[i]*np.sin(phi_)-my[i]*np.cos(phi_))/(mx[i]*np.cos(theta_)+my[i]*np.sin(theta_)*np.sin(phi_)+mz[i]*np.sin(theta_)*np.cos(phi_)))
    roll_[i] = phi_ * 180.0/np.pi
    pitch_[i] = theta_ * 180.0/np.pi
    yaw_[i] = psi_ * 180.0/np.pi
    if (yaw_[i] < 0):
        yaw_[i] += 360
    rpy = [roll,pitch,yaw]
    dt = t[i]-t[i-1]
    angle_finder.update(ax[i], ay[i], az[i], gx[i], gy[i], gz[i], mx[i], my[i], mz[i], dt)
    rpy[0], rpy[1], rpy[2] = angle_finder.getEuler()
    roll[i] = rpy[0]
    pitch[i] = rpy[1]
    yaw[i] = rpy[2]
    if (yaw[i] < 0):
    	yaw[i] += 360

plt.figure()    
plt.plot(t, roll, t,roll_,)
plt.legend(["Roll AHRS","Roll Accel"])
plt.grid()
plt.xlabel("Time (s)"); plt.ylabel("Angle (Degrees)")

plt.figure()    
plt.plot(t, pitch, t,pitch_,)
plt.legend(["Pitch AHRS","Pitch Accel"])
plt.grid()
plt.xlabel("Time (s)"); plt.ylabel("Angle (Degrees)")

plt.figure()
plt.plot(t,yaw,t,yaw_)
plt.legend(["Yaw AHRS","Yaw Mag"])
plt.grid()
plt.xlabel('Time (sec)')
plt.ylabel('Angle (deg)')

plt.show()