import numpy as np
import matplotlib.pyplot as plt
import control

# ============================================
# 2nd order SISO

m = 10  # kg
b = 10  # N s/m
k = 20  # N/m
F = 1  # N

Am = np.array([[0, 1],
               [-k/m, -b/m]])
Bm = np.array([[0],
               [1/m]])

A = np.array([[0, 1],
              [-1, -1]])
B = np.array([[0],
              [1]])

Q = np.eye(2)
P = control.lyap(Am.T, Q)
Pbar = P[:,[1]]

# adaptive gains
gam_x = np.array([[10, 0],
                  [0, 10]])
gam_r = np.array([[10]])

# true k gains
kx = np.linalg.inv(B.T@B)@B.T@(Am-A)
kr = np.linalg.inv(B.T@B)@B.T@Bm

# time and reference
dt = 0.01
time = np.arange(0, 20, dt)
ref = F*np.ones((1, len(time)))  # array

# save matrix
xm = np.zeros((2, len(time)))  # 2xn
x = np.zeros((2, len(time)))  # 2xn
kxhat = np.zeros((len(time), 2))  # nx2
krhat = np.zeros((1, len(time)))  # 1xn

# init
x[:,[0]] = np.array([[0],
                     [0]])
for i in range(len(time)-1):
    # control law
    u = kxhat[[i],:]@x[:,[i]] + krhat[:,[i]]@ref[:,[i]]
    # adaptive law
    e = xm[:,[i]] - x[:,[i]]
    kxhatdot = (gam_x@x[:,[i]]@e.T@Pbar*np.sign(b)).T
    krhatdot = gam_r@ref[:,[i]]@e.T@Pbar*np.sign(b)
    # dynamics
    xmdot = Am@xm[:,[i]] + Bm@ref[:,[i]]
    xdot = A@x[:,[i]] + B@u
    # integrating
    xm[:,[i+1]] = xm[:,[i]] + xmdot*dt
    x[:,[i+1]] = x[:,[i]] + xdot*dt
    kxhat[[i+1],:] = kxhat[[i],:] + kxhatdot*dt
    krhat[:,[i+1]] = krhat[:,[i]] + krhatdot*dt
    
# print(np.shape(xm[0,[i+1]]), np.shape(xm[:,[i]]), np.shape(xmdot))

# plotting
plt.figure(1)
plt.plot(time, xm[0,:], '-', label='xm1')
plt.plot(time, xm[1,:], '-', label='xm2')
plt.plot(time, x[0,:], '--', label='x1')
plt.plot(time, x[1,:], '--', label='x2')
plt.legend()
plt.title('MRAC 2nd order SISO')
plt.xlabel('time')
plt.ylabel('magnitude')
plt.figure(2)
plt.plot(time, kx[0,0]*np.ones(np.shape(time)), 'b-', label='kx1')
plt.plot(time, kx[0,1]*np.ones(np.shape(time)), 'g-', label='kx2')
plt.plot(time, kr[0,0]*np.ones(np.shape(time)), 'r-', label='kr')
plt.plot(time, kxhat[:,0], 'b--', label='kxhat1')
plt.plot(time, kxhat[:,1], 'g--', label='kxhat2')
plt.plot(time, krhat[0,:], 'r--', label='krhat')
plt.legend()
plt.title('adaptive parameters')
plt.xlabel('time')
plt.ylabel('magnitude')
plt.show()

