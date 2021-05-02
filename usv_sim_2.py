import math
import numpy as np


class usv_model_2:
    def __init__(self,
                 rho=1014,
                 g=9.80665,
                 L=48,
                 U_norm=8,
                 xg=-3.38):
        self.rho = rho
        self.g = g
        self.L = L
        self.U_norm = U_norm
        self.xg = xg
        self.m = 634.9*10**(-5) * (0.5*rho*L**3)
        self.Izz = 2.63*10**(-5) * (0.5*rho*L**5)

    def Dynamic_curr(self, velocity, position, curr, rudder_angle, dt):
        input = np.array(
            [velocity[0], velocity[1], velocity[2]*math.pi/180,
             position[2]*math.pi/180])
        u_c = curr[0]
        psi_c = curr[1]*math.pi/180

        u = input[0] + u_c*math.cos(input[3]-psi_c)  # surge vel.
        v = input[1] + u_c*math.sin(input[3]-psi_c)  # sway vel.
        r = input[2]  # yaw vel.

        U = np.sqrt(u**2 + v**2)  # speed

        # X - Coefficients
        Xu_dot = -31.0323*10**(-5) * (0.5*self.rho*self.L**3)
        Xuu = -167.7891*10**(-5) * (0.5*self.rho*self.L**2)
        Xvr = 209.5232*10**(-5) * (0.5*self.rho*self. L**3)
        Xdel = -2.382*10**(-5) * (0.5*self.rho*(self.L**2)*(U**2))
        Xdd = -242.1647*10**(-5) * (0.5*self.rho*(self.L**2)*(U**2))
        # N - Coefficients
        Nv_dot = 19.989*10**(-5) * (0.5*self.rho*(self.L**4))
        Nr_dot = -29.956*10**(-5) * (0.5*self.rho*(self.L**5))
        Nuv = -164.080*10**(-5) * (0.5*self.rho*(self.L**3))
        Nur = -175.104*10**(-5) * (0.5*self.rho*(self.L**4))
        Nrr = -156.364*10**(-5) * (0.5*self.rho*(self.L**5))
        Nrv = -579.631*10**(-5) * (0.5*self.rho*(self.L**4))
        Ndel = -166.365*10**(-5) * (0.5*self.rho*(self.L**3)*(U**2))
        # Y - Coefficients
        Yv_dot = -700.907*10**(-5) * (0.5*self.rho*(self.L**3))
        Yr_dot = -52.018*10**(-5) * (0.5*self.rho*(self.L**4))
        Yuv = -1010.163*10**(-5) * (0.5*self.rho*(self.L**2))
        Yur = 233.635*10**(-5) * (0.5*self.rho*(self.L**3))
        Yvv = -316.746*10**(-5) * (0.5*self.rho*(self.L**2))
        Yvr = -1416.083*10**(-5) * (0.5*self. rho*(self.L**3))
        Yrv = -324.593*10**(-5) * (0.5*self.rho*(self.L**3))
        Ydel = 370.6*10**(-5) * (0.5*self. rho*(self.L**2)*(U**2))

        # Hydrodynamic
        Ta = -Xuu*self.U_norm*self.U_norm  # Assumption: constant
        Xhyd = Xuu*u*np.abs(u) + Xvr*v*r + Ta
        Yhyd = Yuv*np.abs(u)*v + Yur*u*r + Yvv*v*np.abs(v) + Yvr*v*np.abs(r) \
            + Yrv*r*np.abs(v)
        Nhyd = Nuv*np.abs(u)*v + Nur*np.abs(u)*r + Nrr*r*np.abs(r) \
            + Nrv*r*np.abs(v)

        Xrudder = 0  # Xdd * rudder_angle * rudder_angle + Xdel * rudder_angle
        Yrudder = Ydel*rudder_angle
        Nrudder = Ndel*rudder_angle

        H = np.array([[self.m - Xu_dot, 0, 0, 0],
                      [0, self.m-Yv_dot, self.m*self.xg-Yr_dot, 0],
                      [0, self.m*self.xg-Nv_dot, self.Izz-Nr_dot, 0],
                      [0, 0, 0, 1]])
        f = np.array([Xhyd + self.m*(v*r+self.xg*r**2) + Xrudder,
                      Yhyd - self.m*u*r + Yrudder,
                      Nhyd - self.m*self.xg*u*r + Nrudder,
                      r])
        # output = np.matmul(np.linalg.inv(H), f).reshape((4))
        output = np.linalg.solve(H, f)

        velocity[0] = velocity[0] + output[0]*dt
        velocity[1] = velocity[1] + output[1]*dt
        velocity[2] = velocity[2] + output[2]*180./math.pi*dt
        position[2] = position[2] + velocity[2] * dt

        if position[2] > 180:
            position[2] = position[2]-360

        if position[2] < -180:
            position[2] = position[2]+360

        rot_matrix = np.array(
            [[math.cos(position[2]*math.pi/180),
              -math.sin(position[2]*math.pi/180)],
             [math.sin(position[2]*math.pi/180),
              math.cos(position[2]*math.pi/180)]])

        # pdot = np.array([[velocity[0]], [velocity[1]]])
        # pdot = pdot.reshape(2,1)
        pdot = np.array([velocity[0], velocity[1]])
        XYvel = np.dot(rot_matrix, pdot)
        position[0] += XYvel[0] * dt
        position[1] += XYvel[1] * dt

        # print(velocity,XYvel,position)
        return velocity, position
