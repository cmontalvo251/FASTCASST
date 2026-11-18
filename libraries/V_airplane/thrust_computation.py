import numpy as np
import matplotlib.pyplot as plt

OUTMIN = 992.
OUTMAX = 1950.

gt = np.linspace(OUTMIN,OUTMAX,1000)

kt = 3.86e-6 * (1900-1100)**2 / (OUTMAX-OUTMIN)**2
ktN = ( kt / 2.2 ) * 9.81
print('ktN = ',ktN)
#Toriginal = kt*gt**2
TtypoN = ktN*(gt-OUTMIN)**2

##Convert to angular velocity
V = 3.7*4
KV = 950
omega_max_rpm = KV*V
omega_max_rads = 2*np.pi/60 * omega_max_rpm
print('Omega Max (RPM) = ',omega_max_rpm)
print('Omega Max (rad/s) = ',omega_max_rads)
wt = omega_max_rads/(OUTMAX-OUTMIN)
print('wt = ',wt)
rho = 1.225
R = (9.5/12) / (2*3.28)
A = np.pi*R**2
CT = ktN / (rho*A*wt**2*R**2)
print('CT = ',CT)

w = wt*(gt-OUTMIN)
T = rho*A*(w*R)**2*CT

#plt.plot(gt,Toriginal,label='Equation in Paper')
plt.plot(gt,TtypoN,label='My guess at the real eqn')
plt.plot(gt,T,label='New Equation')
plt.legend()
plt.xlabel('us')
plt.ylabel('Thrust (N)')
plt.grid()
plt.show()