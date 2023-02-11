import numpy as np
import matplotlib.pyplot as plt
import control

# =====================================================
# First order SISO

am = -2
bm = 2
a = 4
b = 2

# adaptive gains
gam_x = 10
gam_r = 10

# true k gains
kx = (am-a)/b
kr = bm/b

# time and reference
dt = 0.01
time = np.arange(0, 20, dt)
ref = 1*np.ones((1, len(time)))  # array

# save matrix
xm = np.zeros((1, len(time)))  # 1xn
x = np.zeros((1, len(time)))  # 1xn
kxhat = np.zeros((1, len(time)))  # 1xn
krhat = np.zeros((1, len(time)))  # 1xn

# init
x[0] = 0
for i in range(len(time)-1):
    # control law
    u = kxhat[0,i]*x[0,i] + krhat[0,i]*ref[0,i]
    # adaptive law
    e = xm[0,i] - x[0,i]
    kxhatdot = gam_x*x[0,i]*e*np.sign(b)
    krhatdot = gam_r*ref[0,i]*e*np.sign(b)
    # dynamics
    xmdot = am*xm[0,i] + bm*ref[0,i]
    xdot = a*x[0,i] + b*u
    # integrating
    xm[0,i+1] = xm[0,i] + xmdot*dt
    x[0,i+1] = x[0,i] + xdot*dt
    kxhat[0,i+1] = kxhat[0,i] + kxhatdot*dt
    krhat[0,i+1] = krhat[0,i] + krhatdot*dt

# print(np.shape(xm[:,0]), np.shape(xm[0,0]))

# plotting
plt.figure(1)
plt.plot(time, xm[0,:], '-', label='xm')
plt.plot(time, x[0,:], '--', label='x')
plt.legend()
plt.title('MRAC 1st order SISO')
plt.xlabel('time')
plt.ylabel('magnitude')
plt.figure(2)
plt.plot(time, kx*np.ones(np.shape(time)), 'b-', label='kx')
plt.plot(time, kr*np.ones(np.shape(time)), 'r-', label='kr')
plt.plot(time, kxhat[0,:], 'b--', label='kxhat')
plt.plot(time, krhat[0,:], 'r--', label='krhat')
plt.legend()
plt.title('adaptive parameters')
plt.xlabel('time')
plt.ylabel('magnitude')
plt.show()
