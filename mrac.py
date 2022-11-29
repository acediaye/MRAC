import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint

# # unknown
# a = 1
# b = 3
# # known
# gam = 2
# am = -4
# bm = 4

# time = np.arange(0, 10, 0.01)
# ref = 4
# # ref = 4*np.sin(3*time)

# def model(states, time, ref):
#     x, xm, kxhat, krhat = states
#     ref = 4*np.sin(3*time)
#     u = kxhat*x + krhat*ref
#     xdot = a*x + b*u
#     xmdot = am*xm + bm*ref
    
#     e = x - xm
#     kxhatdot = -gam*x*e
#     krhatdot = -gam*ref*e
#     return [xdot, xmdot, kxhatdot, krhatdot]

# x0 = [0, 0, 0, 0]
# states = odeint(model, x0, time, args=(ref,))
# print(np.shape(states))
# x = states[:,0]
# xm = states[:,1]
# kxhat = states[:,2]
# krhat = states[:,3]
# kx = (am-a)/b
# kr = bm/b

# plt.figure(1)
# plt.subplot(2,1,1)
# plt.plot(time, xm, label='xm')
# plt.plot(time, x, label='x')
# plt.legend()
# plt.subplot(2,1,2)
# plt.plot(time, kx*np.ones(len(time)), label='kx')
# plt.plot(time, kxhat, label='kxhat')
# plt.plot(time, kr*np.ones(len(time)), label='kr')
# plt.plot(time, krhat, label='krhat')
# plt.legend()
# plt.show()

# ==========================================================

# unknown
a = 1
b = 3
theta = np.array([[0], [-1], [1], [0]])
# known
am = -4
bm = 4
gam = 2
thetahat = np.array([[0], [0], [0], [0]])


time = np.arange(0, 50, 0.02)
def model(states, time):
    x, xm, kxhat, krhat, thehat1, thehat2, thehat3, thehat4 = states
    thetahat = np.array([[thehat1], [thehat2], [thehat3], [thehat4]])
    phi = np.array([[x**3], [np.exp(-((x+0.5)**2)*10)], [np.exp(-((x-0.5)**2)*10)], [np.sin(2*x)]])
    ref = np.sin(3*time) + np.sin(3*time/2) + np.sin(3*time/4) + np.sin(3*time/8)
    u = kxhat*x + krhat*ref + thetahat.T@phi
    
    xdot = a*x + b*(u-theta.T@phi)
    xmdot = am*x + bm*ref
    e = x - xm
    kxhatdot = -gam*x*e
    krhatdot = -gam*ref*e
    thetahatdot = -gam*phi*e
    
    thehatdot1 = thetahatdot[0,0]
    thehatdot2 = thetahatdot[1,0]
    thehatdot3 = thetahatdot[2,0]
    thehatdot4 = thetahatdot[3,0]
    # print(np.shape(xdot), np.shape(xmdot), np.shape(kxhatdot), np.shape(krhatdot), np.shape(thetahatdot), np.shape(thetahatdot1))
    return [xdot, xmdot, kxhatdot, krhatdot, thehatdot1, thehatdot2, thehatdot3, thehatdot4]

x0 = np.array([0, 0, 0, 0, 0, 0, 0, 0])
states = odeint(model, x0, time)
print(np.shape(states))

x = states[:,0]
xm = states[:,1]
kxhat = states[:,2]
krhat = states[:,3]
thetahat1 = states[:,4]
thetahat2 = states[:,5]
thetahat3 = states[:,6]
thetahat4 = states[:,7]
kx = (am-a)/b
kr = bm/b

plt.figure(1)
plt.subplot(3,1,1)
plt.plot(time, xm, label='xm')
plt.plot(time, x, label='x')
plt.legend()
plt.subplot(3,1,2)
plt.plot(time, kx*np.ones(len(time)), label='kx')
plt.plot(time, kxhat, label='kxhat')
plt.plot(time, kr*np.ones(len(time)), label='kr')
plt.plot(time, krhat, label='krhat')
plt.legend()
plt.subplot(3,1,3)
plt.plot(time, theta[0,0]*np.ones(len(time)), label='theta1')
plt.plot(time, thetahat1, '--', label='thetahat1')
plt.plot(time, theta[1,0]*np.ones(len(time)), label='theta2')
plt.plot(time, thetahat2, '--', label='thetahat2')
plt.plot(time, theta[2,0]*np.ones(len(time)), label='theta3')
plt.plot(time, thetahat2, '--', label='thetahat3')
plt.plot(time, theta[3,0]*np.ones(len(time)), label='theta4')
plt.plot(time, thetahat2, '--', label='thetahat4')
plt.legend()
plt.show()

