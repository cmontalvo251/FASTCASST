import numpy as np

rho = 0.002377 #slugs/ft^3
V = 44.0 #ft/s
b = 52.
c = 8
AR = b**2/416.0
Clmax = 1.5
CLmax = Clmax/(1+Clmax*np.pi/AR)
S = 416/144 #sq ft
L = 0.5*rho*(V**2)*S*CLmax
W = 9.0

CLmax_req = np.sqrt(2*W/(rho*(V**2)*S))
print(CLmax_req)

var = np.float(input('text'))