"""
Collision avoidance using Velocity-obstacle method

"""
from multi_ship_plot import plot_ship_and_obstacles
from create_obstacles import create_TS
from control import compute_desired_velocity
import numpy as np
import math
import CRI_FunExe
import matplotlib.pyplot as plt


SIM_TIME = 25.5
TIMESTEP = 0.1
NUMBER_OF_TIMESTEPS = int(SIM_TIME/TIMESTEP)
print('Number of Time Steps : ',NUMBER_OF_TIMESTEPS)
ROBOT_RADIUS = 100
VMAX = 190
VMIN = 190

start = np.array([0, 0, 0, 0])
goal = np.array([5, 4000, 0, 0])



def compute_velocity(ship, obstacles, v_desired,STN):
    pA = ship[:2] # own ship position  x and y
    xposOwn = pA[0] * 0.000539957# x pos of ownship
    yposOwn = pA[1] * 0.000539957# y pos of ownship

    vA = ship[-2:] # velocity and angle
    vAx = vA[0] #Velocity along X
    vAy = vA[1] #Velocity along Y
    #--------- Velocity of own ships --------
    Vo = math.sqrt((vAx)**2 + (vAy)**2) * 0.1
    # print('The X pos of Own Ship : ', xposOwn)
    # print('The Y pos of Own Ship : ', yposOwn)
    #print('Velocity of Own Ship : ',Vo)
    #print(Vo)

    ratioVown = vAx / vAy
    angown = math.degrees(math.atan(ratioVown)) # angle of the own ship heading

    if np.isnan(angown):
        angown = 0
    else:
        angown = angown

    if angown < 0:
        own_ang = 360 + angown
    else:
        own_ang = angown

    #print('Angle of Own Ship: ', own_ang)
    #print(round(own_ang,2))

    #-----------------------------------------------------
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
        xposTs = pB[0] * 0.000539957 # x pos of TSship
        yposTs = pB[1]  * 0.000539957# y pos of TSship
        Vt = math.sqrt((vBx) ** 2 + (vBy) ** 2) * 0.1

        # print('The X pos of TS',STN ,' : ',xposTs)
        # print('The Y pos of TS',STN ,' : ',yposTs)
        # print('Velocity of Tar Ship',STN ,' : ',Vt)
        #print(Vt)

        ratioV = vBx/vBy

        ango = math.degrees(math.atan(ratioV))
        if ango < 0:
            TS_ang = 360 + ango
        else:
            TS_ang = ango

        # print('Angle of Ts',STN ,' : ',TS_ang)
        #print(TS_ang)

    #----------------------------------------------------------------------------
        #================================ CRI ===================================

        cri_index = CRI_FunExe.CRI_call(Vo,Vt,xposOwn,yposOwn,xposTs,yposTs,own_ang,TS_ang)

        cri = cri_index[1]
        dcpa = cri_index[0]
        tcpa = cri_index[2]
        dbtn = cri_index[8]
        #print('The CRI index is : ',cri)
        #print(cri)
        #print(dcpa)
        #print(tcpa)
        #print(round(dbtn,3))




        #print(obstacle)
        dispBA = pA - pB # displacement of obstacle to own ship
        distBA = np.linalg.norm(dispBA)
        thetaBA = np.arctan2(dispBA[1], dispBA[0])


        #COLREG Implementation
        #print(math.degrees(thetaBA))

        gotang = - (math.degrees(thetaBA))

        if gotang < 0:
            ang2own = 360 + (gotang)
        else:
            ang2own = gotang
        #print(ang2own)

        col_angP = ang2own
        print(distBA)

        if cri > 0.7:

            if distBA < 500:


                if col_angP <=67.5 and col_angP>5:
                    #appoch()
                    print('A')
                    print('We are the give way ship: Action will be taken')
                    #print('Own Ship Lies in the COLREG region A')
                    if 2.5 * ROBOT_RADIUS > distBA:
                        distBA = 2.7 * ROBOT_RADIUS
                    phi_obst = np.arcsin(2.7 * ROBOT_RADIUS / distBA)
                    # print(math.degrees(phi_obst))
                    phi_left = (thetaBA + phi_obst)
                    phi_right = (thetaBA - 3*phi_obst)


                elif col_angP <=112.5 and col_angP>67.5:
                    #print('Own Ship Lies in the COLREG region B')
                    print('B')
                    print('We are the give way ship: Action will be taken')
                    if 2.5 * ROBOT_RADIUS > distBA:
                        distBA = 2.7 * ROBOT_RADIUS
                    phi_obst = np.arcsin(2.7 * ROBOT_RADIUS / distBA)
                    # print(math.degrees(phi_obst))
                    phi_left = (thetaBA + phi_obst)
                    phi_right =(thetaBA - 3*phi_obst)

                elif col_angP <=210.0 and col_angP>112.5:
                    print('C')
                    print('Ship Passed: Encounter Over')
                    #print('Own Ship Lies in the COLREG region C')
                    if 2.5* ROBOT_RADIUS > distBA:
                        distBA = 2.7 * ROBOT_RADIUS
                    phi_obst = np.arcsin(2.7 * ROBOT_RADIUS / distBA)
                    # print(math.degrees(phi_obst))
                    phi_left = thetaBA + phi_obst
                    phi_right = thetaBA - phi_obst

                elif col_angP <=247.5 and col_angP>210.0:
                    print('D')
                    print('We are Stand on ship: No Action Taken')
                    #print('Own Ship Lies in the COLREG region D')
                    if 2.7* ROBOT_RADIUS > distBA:
                        distBA = 3.2 * ROBOT_RADIUS
                    phi_obst = np.arcsin(3.2 * ROBOT_RADIUS / distBA)
                    # print(math.degrees(phi_obst))
                    phi_left = thetaBA + phi_obst
                    phi_right = thetaBA - phi_obst

                elif col_angP <=355 and col_angP>247.5:
                    print('E')
                    print('We are Stand on ship: No Action Taken')
                    #print('Own Ship Lies in the COLREG region E')
                    if 0.1 * ROBOT_RADIUS > distBA:
                        distBA = 0.1 * ROBOT_RADIUS
                    phi_obst = np.arcsin(0.1 * ROBOT_RADIUS / distBA)
                    # print(math.degrees(phi_obst))
                    phi_left = thetaBA + phi_obst
                    phi_right = thetaBA - phi_obst


                else :
                    #print('Own Ship Lies in the COLREG region F')
                    print('F')
                    print('Head Own : No Action Taken')
                    if 1.1 * ROBOT_RADIUS > distBA:
                        distBA = 1.5 * ROBOT_RADIUS
                    phi_obst = np.arcsin(1.1 * ROBOT_RADIUS / distBA)
                    # print(math.degrees(phi_obst))
                    phi_left = thetaBA + phi_obst
                    phi_right = thetaBA - phi_obst

            else:
                # print('Own Ship Lies in the COLREG region F')
                print('no action ')
                if 1.5 * ROBOT_RADIUS > distBA:
                    distBA = 1.7 * ROBOT_RADIUS
                phi_obst = np.arcsin(1.7 * ROBOT_RADIUS / distBA)
                # print(math.degrees(phi_obst))
                phi_left = thetaBA + phi_obst
                phi_right = thetaBA - phi_obst

        else:
            # print('Own Ship Lies in the COLREG region F')
            print('no action ')
            if 0.1 * ROBOT_RADIUS > distBA:
                distBA = 0.1 * ROBOT_RADIUS
            phi_obst = np.arcsin(0.1 * ROBOT_RADIUS / distBA)
            # print(math.degrees(phi_obst))
            phi_left = thetaBA + phi_obst
            phi_right = thetaBA - phi_obst



        #print('=================================================')


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
TSP = create_TS(SIM_TIME, NUMBER_OF_TIMESTEPS)
TS = TSP[0]
TSN= TSP[1]
#print(TSN)
ts1_x = []
ts1_y = []




robot_state = start
robot_state_history = np.empty((4, NUMBER_OF_TIMESTEPS))

done = False
for i in range(NUMBER_OF_TIMESTEPS):
    v_desired = compute_desired_velocity(robot_state, goal, ROBOT_RADIUS, VMAX)
    control_vel = compute_velocity(robot_state, TS[:, i, :], v_desired,TSN)
    robot_state = update_state(robot_state, control_vel)
    robot_state_history[:4, i] = robot_state
    if control_vel[0] == 0:
        done = True


if done == True:
    print('Goal')

plot_ship_and_obstacles(
    robot_state_history, TS, ROBOT_RADIUS, NUMBER_OF_TIMESTEPS, SIM_TIME, filename,start[0],start[1],goal[0],goal[1])