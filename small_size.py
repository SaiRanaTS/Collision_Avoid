"""
Collision avoidance using Velocity-obstacle method

"""
from small_plot import plot_robot_and_obstacles
from create_obstacles import create_TS
from control import compute_desired_velocity
import numpy as np
import math
import matplotlib.pyplot as plt


SIM_TIME = 40
TIMESTEP = 0.1
NUMBER_OF_TIMESTEPS = int(SIM_TIME/TIMESTEP)
ROBOT_RADIUS = 0.4
VMAX = 3
VMIN = 2

start = np.array([0, 0, 0, 0])
goal = np.array([5, 40, 0, 0])



def compute_velocity(ship, obstacles, v_desired):
    pA = ship[:2] # own ship position  x and y
    vA = ship[-2:]
    #print(obstacles)
    # Compute the constraints
    # for each velocity obstacles
    number_of_obstacles = np.shape(obstacles)[1]
    Amat = np.empty((number_of_obstacles * 2, 2))
    bvec = np.empty((number_of_obstacles * 2))
    for i in range(number_of_obstacles):
        obstacle = obstacles[:, i] #state of obstarcle in x y v and angle in array
        pB = obstacle[:2] # X and Y coordinates of the obstacle
        vB = obstacle[2:] # v and angle of obstacle
        vBx = vB[0]
        vBy = vB[1]
        ratioV = vBy/vBx
        #print(ratioV)
        ango = math.degrees(math.atan(ratioV))
        #print(ango)

        #print(obstacle)
        dispBA = pA - pB # displacement of obstacle to own ship
        distBA = np.linalg.norm(dispBA)
        thetaBA = np.arctan2(dispBA[1], dispBA[0])


        #COLREG Implementation
        #print(math.degrees(thetaBA))

        gotang = math.degrees(thetaBA)
        if gotang < 0:
            ang2own = 360 + (gotang)
        else:
            ang2own = gotang

        covrt_ang = ang2own
        col_ang = 360 - covrt_ang + ango

        if col_ang > 360:
            col_angP = col_ang - 360
        else: col_angP = col_ang


        if col_angP <=67.5 and col_angP>5:
            #appoch()
            print('A')
            if 2 * ROBOT_RADIUS > distBA:
                distBA = 1 * ROBOT_RADIUS
            phi_obst = np.arcsin(2 * ROBOT_RADIUS / distBA)
            # print(math.degrees(phi_obst))
            phi_left = thetaBA + phi_obst
            phi_right = thetaBA - phi_obst

        elif col_angP <=112.5 and col_angP>67.5:
            print('B')
            if 1 * ROBOT_RADIUS > distBA:
                distBA = 1 * ROBOT_RADIUS
            phi_obst = np.arcsin(2 * ROBOT_RADIUS / distBA)
            # print(math.degrees(phi_obst))
            phi_left = thetaBA + phi_obst
            phi_right = thetaBA - phi_obst

        elif col_angP <=210.0 and col_angP>112.5:
            print('C')
            if 1 * ROBOT_RADIUS > distBA:
                distBA = 1 * ROBOT_RADIUS
            phi_obst = np.arcsin(2.3 * ROBOT_RADIUS / distBA)
            # print(math.degrees(phi_obst))
            phi_left = thetaBA + phi_obst
            phi_right = thetaBA - phi_obst

        elif col_angP <=247.5 and col_angP>210.0:
            print('D')
            if 1 * ROBOT_RADIUS > distBA:
                distBA = 1 * ROBOT_RADIUS
            phi_obst = np.arcsin(2 * ROBOT_RADIUS / distBA)
            # print(math.degrees(phi_obst))
            phi_left = thetaBA + phi_obst
            phi_right = thetaBA - phi_obst

        elif col_angP <=355 and col_angP>247.5:
            print('E')
            if 1 * ROBOT_RADIUS > distBA:
                distBA = 1 * ROBOT_RADIUS
            phi_obst = np.arcsin(2 * ROBOT_RADIUS / distBA)
            # print(math.degrees(phi_obst))
            phi_left = thetaBA + phi_obst
            phi_right = thetaBA - phi_obst


        else :
            print('F')
            if 2.2 * ROBOT_RADIUS > distBA:
                distBA = 1.0 * ROBOT_RADIUS
            phi_obst = np.arcsin(1.0 * ROBOT_RADIUS / distBA)
            # print(math.degrees(phi_obst))
            phi_left = thetaBA + phi_obst
            phi_right = thetaBA - phi_obst




        # VO
        translation = vB
        Atemp, btemp = create_constraints(translation, phi_left, "left")
        Amat[i*2, :] = Atemp
        bvec[i*2] = btemp
        Atemp, btemp = create_constraints(translation, phi_right, "right")
        Amat[i*2 + 1, :] = Atemp
        bvec[i*2 + 1] = btemp

    # Create search-space
    th = np.linspace(0, 2*np.pi, 20)
    vel = np.linspace(0, VMAX, 5)

    vv, thth = np.meshgrid(vel, th)

    vx_sample = (vv * np.cos(thth)).flatten()
    vy_sample = (vv * np.sin(thth)).flatten()

    v_sample = np.stack((vx_sample, vy_sample))

    v_satisfying_constraints = check_constraints(v_sample, Amat, bvec)

    # Objective function
    size = np.shape(v_satisfying_constraints)[1]
    diffs = v_satisfying_constraints - \
        ((v_desired).reshape(2, 1) @ np.ones(size).reshape(1, size))
    norm = np.linalg.norm(diffs, axis=0)
    min_index = np.where(norm == np.amin(norm))[0][0]
    cmd_vel = (v_satisfying_constraints[:, min_index])
    vp = math.sqrt((cmd_vel[0])**2 + (cmd_vel[1])**2)
    #print(vp)
    return cmd_vel


def check_constraints(v_sample, Amat, bvec):
    length = np.shape(bvec)[0]

    for i in range(int(length/2)):
        v_sample = check_inside(v_sample, Amat[2*i:2*i+2, :], bvec[2*i:2*i+2])

    return v_sample


def check_inside(v, Amat, bvec):
    v_out = []
    for i in range(np.shape(v)[1]):
        if not ((Amat @ v[:, i] < bvec).all()):
            v_out.append(v[:, i])
    return np.array(v_out).T


def create_constraints(translation, angle, side):
    # create line
    origin = np.array([0, 0, 1])
    point = np.array([np.cos(angle), np.sin(angle)])
    line = np.cross(origin, point)
    line = translate_line(line, translation)

    if side == "left":
        line *= -1

    A = line[:2]
    b = -line[2]

    return A, b


def translate_line(line, translation):
    matrix = np.eye(3)
    matrix[2, :2] = -translation[:2]
    return matrix @ line

Osx_list = []
Osy_list = []
def update_state(x, v):
    new_state = np.empty((4))
    new_state[:2] = x[:2] + v * TIMESTEP
    new_state[-2:] = v
    Osx_list.append(round(x[0],2))
    Osy_list.append(round(x[1], 2))
    return new_state



filename = 'test'
TS = create_TS(SIM_TIME, NUMBER_OF_TIMESTEPS)
ts1_x = []
ts1_y = []




robot_state = start
robot_state_history = np.empty((4, NUMBER_OF_TIMESTEPS))

done = False
for i in range(NUMBER_OF_TIMESTEPS):
    v_desired = compute_desired_velocity(robot_state, goal, ROBOT_RADIUS, VMAX)
    control_vel = compute_velocity(robot_state, TS[:, i, :], v_desired)
    robot_state = update_state(robot_state, control_vel)
    robot_state_history[:4, i] = robot_state
    if control_vel[0] == 0:
        done = True


if done == True:
    print('Goal')

plot_robot_and_obstacles(
    robot_state_history, TS, ROBOT_RADIUS, NUMBER_OF_TIMESTEPS, SIM_TIME, filename)
