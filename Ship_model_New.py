import math
import numpy as np
import matplotlib as plt



tau_forward = [1e5,0,0]
tau_positive = [1e5,0,1e6]
tau_negative = [1e5,0,-1e6]


M = [[1e7, 0, 0],
     [0, 1e7, 8.4e6],
     [0, 8.4e6, 5.8e9]]


# Damping

D = [[300000, 0, 0],
     [0, 550000, 600000],
     [0, 600000, 1.3e8]]

t_final = 2000  # final simulation time (sec)
h = 1           # time step


N = round(t_final/h)


R4 = 3
C4 = N

data_eta1 = []
data_nu1 = []
data_tau1 = []

for i in range(R4):  # A for loop for row entries
    a = []
    for j in range(C4):  # A for loop for column entries
        a.append(0)
    data_eta1.append(a)
    data_nu1.append(a)
    data_tau1.append(a)




data_eta = np.array(data_eta1)
data_nu = np.array(data_nu1)
data_tau = np.array(data_tau1)

eta = data_eta[:,0]
nu = data_nu[:,0]


tau = tau_forward

for i in range(1,N):
    psi = np.array(eta[2])
    R = [[math.cos(psi),-math.sin(psi),0],
          [math.sin(psi),math.cos(psi),0],
          [0,0,1]]

    if i*h < 50:
        tau = tau_forward
    else:
        if tau == tau_forward:
            tau = tau_positive
        elif psi > math.radians(20):
            tau = tau_negative
        elif psi < math.radians(-20):
            tau = tau_positive


    multi = D * nu
    inv_M = np.linalg.inv(M)
    acc = inv_M * (tau - multi)
    nu = nu + acc[0] * h
    spd = R * nu
    eta = eta + spd * h

    data_nu[:, i] = nu
    data_eta[:, i] = eta
    data_tau[:, i] = tau




plt.plot(data_eta[0,:],data_eta[1,:])