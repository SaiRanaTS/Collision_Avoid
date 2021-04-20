import numpy as np
import math

def input_TS():
    pos3 = [1400, 2500]
    v_ts3 = 130
    ang3 = 180
    ts3_data = (pos3,v_ts3,ang3)

    return ts3_data


def create_TS(sim_time, num_timesteps):
    # TS 1
    # v = -80
    # p0 = np.array([1500, 1200])
    # ang1 = 0
    # rad = 20
    # obst = create_ship(p0, v, math.radians(ang1), sim_time,
    #                     num_timesteps).reshape(4, num_timesteps, 1)  # split the matrix into 4 - x values, y values , velocity and angle
    #obstacles = obst
    # #TS 2
    # v = -120
    # p0 = np.array([1250, 1000])
    # ang2 = 0
    # obst2 = create_ship(p0, v, math.radians(ang2), sim_time, num_timesteps).reshape(
    #     4, num_timesteps, 1)
    # obstacles = np.dstack((obstacles, obst2))
    # TS 3
    v = input_TS()[1]
    p0 = np.array(input_TS()[0])
    ang3 = input_TS()[2]
    obst = create_ship(p0, v, math.radians(ang3), sim_time, num_timesteps).reshape(4,
                                                                        num_timesteps, 1)
    obstacles = obst
    obstacles = np.dstack((obstacles, obst))
    # # TS 4
    # v = 150
    # p0 = np.array([2250, 3500])
    # ang4 = 200
    # obst = create_ship(p0, v, math.radians(ang4), sim_time, num_timesteps).reshape(4,
    #                                                                            num_timesteps, 1)
    # obstacles = np.dstack((obstacles, obst))
    #
    #
    # # TS 5
    # v = 150
    # p0 = np.array([1450, 3000])
    # ang5 = 240
    # obst = create_ship(p0, v, math.radians(ang4), sim_time, num_timesteps).reshape(4,
    #                                                                            num_timesteps, 1)
    # obstacles = np.dstack((obstacles, obst))
    #
    # # TS 6
    # v = 150
    # p0 = np.array([-1350, 2700])
    # ang6 = 0
    # obst = create_ship(p0, v, math.radians(ang6), sim_time, num_timesteps).reshape(4,
    #                                                                            num_timesteps, 1)
    # obstacles = np.dstack((obstacles, obst))
    #
    # # TS 7
    # v = 150
    # p0 = np.array([-1450, 1100])
    # ang7 = 25
    # obst = create_ship(p0, v, math.radians(ang7), sim_time, num_timesteps).reshape(4,
    #                                                                                 num_timesteps, 1)
    # obstacles = np.dstack((obstacles, obst))
    #
    #
    # # TS 8
    # v = 150
    # p0 = np.array([-1050, 1700])
    # ang8 = 10
    # obst = create_ship(p0, v, math.radians(ang8), sim_time, num_timesteps).reshape(4,
    #                                                                                 num_timesteps, 1)
    # obstacles = np.dstack((obstacles, obst))
    #
    #
    # # TS 9
    # v = -150
    # p0 = np.array([-1050, 1600])
    # ang9 = 140
    # obst = create_ship(p0, v, math.radians(ang9), sim_time, num_timesteps).reshape(4,
    #                                                                                 num_timesteps, 1)
    # obstacles = np.dstack((obstacles, obst))
    # #

    return obstacles


def create_ship(p0, v, theta, sim_time, num_timesteps):

    # Creates obstacles starting at p0 and moving at v in theta direction
    t = np.linspace(0, sim_time, num_timesteps)
    theta = theta * np.ones(np.shape(t))
    vx = v * np.cos(theta)
    vy = v * np.sin(theta)
    v = np.stack([vx, vy])
    p0 = p0.reshape((2, 1))
    p = p0 + np.cumsum(v, axis=1) * (sim_time / num_timesteps)
    p = np.concatenate((p, v)) #position and velocity along each time step
    return p