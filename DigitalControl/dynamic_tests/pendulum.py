import numpy as np
import math
import copy
from scipy.integrate import odeint
from scipy.optimize import minimize
import matplotlib.pyplot as plt
import pandas as pd
names = ['Time (s)', 'Joint 1', 'Vel 1', 'Force', 'Target', 'Tar Vel']

OPTIM = False
EULER = False

m = 1
L = 0.8
k = 1
g = 10

gm = k/(m*L*L)
w = g/L
#1.56249836 12.49998696

print(f"GM: {gm}")
print(f"W : {w}")

dt = 1/240
tt = np.array([0, dt])
T = 6
TT = np.arange(0,T+dt,dt)
x0 = np.array([0,0.3])

t = 0

def rp(in_, t, a, b):
    v = in_[0]
    x = in_[1]
    dv = -a*v - b*math.sin(x) + 0.01/(m*L*L)
    dx = v
    return np.array([dv,dx])

# symplectic euler method
def symp_euler(fun, x0, TT, a, b):
    x1 = copy.copy(x0)
    xx = np.array(x1)
    for i in range(len(TT)-1):
        dt = (TT[i+1]-TT[i])
        x1[0] += rp(x1, 0, a, b)[0]*dt
        x1[1] += x1[0]*dt
        xx = np.vstack((xx,x1))
    return xx

def cost(q_exp, q_th):
    l2 = 0
    sz = len(q_exp)
    linf = abs(q_exp[0] - q_th[0])
    for i in range(sz-1):
        err = abs(q_exp[i] - q_th[i])
        if (err > linf):
            linf = err
        l2 += dt*err**2
    l2 = math.sqrt(l2)
    return (l2, linf)

def compare(x0, T, a = gm, b = w):

    if (EULER):
        xx = symp_euler(rp,x0,TT,a,b)
    else:
        xx = odeint(rp,x0,TT,args=(gm,w))

    # plt.plot(TT,xx[:,1],label='model')
    # plt.grid()
    plt.grid()

    log_csv = pd.read_csv('log.csv', header=None, names=names)
    log_q = log_csv['Joint 1']

    (l2,linf) = cost(log_q, xx[:,1])

    print(f"L2: {l2}")
    print(f"L_inf: {linf}")

    plt.plot(log_csv['Time (s)'], log_csv['Joint 1'],label='joint')

    plt.plot(log_csv['Time (s)'], log_csv['Vel 1'],label='vel')

    plt.plot(log_csv['Time (s)'], log_csv['Target'],label='Target')

    plt.plot(log_csv['Time (s)'], log_csv['Tar Vel'],label='Tar Vel')

    plt.legend()

    plt.show()
    

if __name__ == "__main__":
    log_csv = pd.read_csv('log.csv', header=None, names=names)
    log_q = log_csv['Joint 1']

    compare(x0,T,gm,w)

    if (OPTIM):
        def l2_cost(k):
            x = symp_euler(rp,x0,TT,k[0],k[1])
            return cost(log_q,x[:,1])[0]

        res = minimize(l2_cost,[gm,w])
        print(f"l2_res: {res.fun}")
        print(f"res k: {res.x}")

        compare(x0,T,res.x[0],res.x[1])