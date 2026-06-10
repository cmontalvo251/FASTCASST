import numpy as np
import matplotlib.pyplot as plt

###Params
b = 52./12.0
c = 8./12.0
rho = 0.002377 #slugs/ft^3
V = 44.0 #ft/s4
S = b*c
AR = b**2/S
Cla = 5.7
Cl0 = 0.3
W = 9.0

alfa_deg = np.linspace(-10,10,100)
alfa_rad = np.pi/180.*alfa_deg

Cl = Cla*alfa_rad + Cl0

alfa0 = -Cl0/Cla

CLa = Cla/(1+Cla/(np.pi*AR))

plt.plot(alfa_deg,Cl)

CL = CLa*(alfa_rad-alfa0)

plt.figure()
plt.plot(alfa_deg,CL)

L = 0.5*rho*(V**2)*S*CL

plt.figure()
plt.plot(alfa_deg,L)


###Now let's assume that Lift = W
V = np.linspace(20,100,100)
CL_req = np.sqrt(2*W/(rho*(V**2)*S))

plt.figure()
plt.plot(V,CL_req)

alfa_req_rad = CL_req/CLa + alfa0
alfa_req_deg = alfa_req_rad*180.0/np.pi

plt.figure()
plt.plot(V,alfa_req_deg)

plt.show()
