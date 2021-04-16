import numpy as np
import math

def create_obstacles(sim_time, num_timesteps):
    # Obstacle 1
    v = -20
    p0 = np.array([500, 1200])
    ang1 = 0
    obst = create_robot(p0, v, math.radians(ang1), sim_time,
                        num_timesteps).reshape(4, num_timesteps, 1)
    obstacles = obst
    # Obstacle 2
    v = -20
    p0 = np.array([500, 1000])
    ang2 = 0
    obst = create_robot(p0, v, math.radians(ang2), sim_time, num_timesteps).reshape(
        4, num_timesteps, 1)
    obstacles = np.dstack((obstacles, obst))
    # Obstacle 3
    v = -20
    p0 = np.array([1200, 2500])
    ang3 = 0
    obst = create_robot(p0, v, math.radians(ang3), sim_time, num_timesteps).reshape(4,
                                                                                num_timesteps, 1)
    obstacles = np.dstack((obstacles, obst))
    # Obstacle 4
    v = -20
    p0 = np.array([1350, 3500])
    ang4 = 0
    obst = create_robot(p0, v, math.radians(ang4), sim_time, num_timesteps).reshape(4,
                                                                               num_timesteps, 1)
    obstacles = np.dstack((obstacles, obst))


    # Obstacle 5
    v = -20
    p0 = np.array([1050, 3000])
    ang5 = 0
    obst = create_robot(p0, v, math.radians(ang4), sim_time, num_timesteps).reshape(4,
                                                                               num_timesteps, 1)
    obstacles = np.dstack((obstacles, obst))

    # Obstacle 6
    v = 20
    p0 = np.array([-1050, 2700])
    ang6 = 0
    obst = create_robot(p0, v, math.radians(ang6), sim_time, num_timesteps).reshape(4,
                                                                               num_timesteps, 1)
    obstacles = np.dstack((obstacles, obst))

    # Obstacle 7
    v = 30
    p0 = np.array([-1050, 2000])
    ang7 = 0
    obst = create_robot(p0, v, math.radians(ang7), sim_time, num_timesteps).reshape(4,
                                                                                    num_timesteps, 1)
    obstacles = np.dstack((obstacles, obst))


    # Obstacle 8
    v = 30
    p0 = np.array([-1050, 2600])
    ang8 = 0
    obst = create_robot(p0, v, math.radians(ang8), sim_time, num_timesteps).reshape(4,
                                                                                    num_timesteps, 1)
    obstacles = np.dstack((obstacles, obst))
    return obstacles


def create_robot(p0, v, theta, sim_time, num_timesteps):
    # Creates obstacles starting at p0 and moving at v in theta direction
    t = np.linspace(0, sim_time, num_timesteps)
    theta = theta * np.ones(np.shape(t))
    vx = v * np.cos(theta)
    vy = v * np.sin(theta)
    v = np.stack([vx, vy])
    p0 = p0.reshape((2, 1))
    p = p0 + np.cumsum(v, axis=1) * (sim_time / num_timesteps)
    p = np.concatenate((p, v))
    return p
