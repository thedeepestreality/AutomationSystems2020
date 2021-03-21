import numpy as np
import math
from scipy.integrate import odeint
import matplotlib.pyplot as plt

m = 1
L = 0.8
k = 1
g = 10

gm = k/(m*L*L)

dt = 1/240
tt = np.array([0, dt])
T = 20
TT = np.arange(0,T+dt,dt)
x0 = np.array([0,0.1])
xx = np.array(x0)
t = 0

def rp(in_,t):
    v = in_[0]
    x = in_[1]
    dv = -gm*v - g*math.sin(x)/L
    dx = v
    return np.array([dv,dx])

# xx = odeint(rp,x0,TT)

# symplectic euler method
while t <= T-dt:
    x0[0] += rp(x0,0)[0]*dt
    x0[1] += x0[0]*dt
    xx = np.vstack((xx,x0))
    t += dt

print(xx)

plt.plot(TT,xx[:,1],label='model')
plt.grid()

import pandas as pd
names = ['Time (s)', 'Joint 1', 'Force']
log_csv = pd.read_csv('log.csv', header=None, names=names)
plt.plot(log_csv['Time (s)'],log_csv['Joint 1'],label='real')
plt.legend()

#log_csv.plot(0, 2, grid=True)

plt.show()