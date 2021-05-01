"""
Plotting tool for 2D multi-robot system

"""

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Circle
import numpy as np


def plot_ship_and_obstacles(ship, obstacles, robot_radius, num_steps, sim_time, filename,str1,str2,gol1,gol2):
    img = plt.imread("bpoA.png")

    fig, ax = plt.subplots()

    fig.set_size_inches(11.25, 7.5)
    plt.subplots_adjust(left=0.05, right=0.95, top=0.95, bottom=0.05)

    # ax.imshow(img, extent=[-4122, 6028, -400, 4675], alpha=1.96)
    # plt.xlim([-4122, 6028])
    # plt.ylim([-400, 4675])

    ax.imshow(img, extent=[3122, 2028, -400, 4675], alpha=1.96)
    plt.xlim([-3122, 2028])
    plt.ylim([-400, 4675])

    plt.xlabel('East (m)')
    plt.ylabel('North (m)')
    plt.grid(alpha=0.2)
    plt.plot(0, 0, '*r')
    plt.plot(0, 4000, '*r')

    ax.text(str1, str2, 'Start Point',color='Black', fontsize=10)
    ax.text((gol1+110), gol2, 'Destination', color='Black', fontsize=10)



    # ax.text(0.95, 0.01, 'CRI infused CA system ',
    #         verticalalignment='bottom', horizontalalignment='right',
    #         transform=ax.transAxes,
    #         color='Black', fontsize=8)
    ax.text(0.95, 0.01, 'CRI infused CA system ',
            verticalalignment='bottom', horizontalalignment='right',
            transform=ax.transAxes,
            color='Black', fontsize=8)


    ax.text(0.95, 0.95, 'CRI cut off : 0.8',
            horizontalalignment='right',
            verticalalignment='top',
            transform=ax.transAxes,color='Black', fontsize=8)

    ax.text(0.22, 0.95, 'COLREGS : OFF',
            horizontalalignment='right',
            verticalalignment='top',
            transform=ax.transAxes,color='Black', fontsize=8)

    # fig = plt.figure()
    # ax = fig.add_subplot(111, autoscale_on=False, xlim=(-120, 120), ylim=(0, 120))
    # ax.set_aspect('equal')
    # ax.grid()
    line, = ax.plot([], [], '-')





    robot_patch = Circle((ship[0, 0], ship[1, 0]), robot_radius, facecolor='white', edgecolor='black')
    obstacle_list = []
    for obstacle in range(np.shape(obstacles)[2]):
        obstacle = Circle((0, 0), robot_radius,
                          alpha=0.4)
        obstacle_list.append(obstacle)

    def init():
        ax.add_patch(robot_patch)
        for obstacle in obstacle_list:
            ax.add_patch(obstacle)
        line.set_data([], [])
        return [robot_patch] + [line] + obstacle_list

    def animate(i):
        robot_patch.center = (ship[0, i], ship[1, i])
        for j in range(len(obstacle_list)):
            obstacle_list[j].center = (obstacles[0, i, j], obstacles[1, i, j])
        line.set_data(ship[0, :i], ship[1, :i])

        return [robot_patch] + [line] + obstacle_list






    init()
    step = (sim_time / num_steps)
    for i in range(num_steps):
        animate(i)
        plt.pause(step)


    # Save animation
    # if not filename:
    #     return
    #
    # ani = animation.FuncAnimation(
    #     fig, animate, np.arange(1, num_steps), interval=200,
    #     blit=True, init_func=init)
    #
    # ani.save(filename, "ffmpeg", fps=30)


def plot_robot(robot, timestep, radius=1, is_obstacle=False):
    if robot is None:
        return
    center = robot[:2, timestep]
    x = center[0]
    y = center[1]
    plt.text(1000, 1000, 'Start Point', color='Black', fontsize=10)
    if is_obstacle:
        circle = plt.Circle((x, y), radius, color='red', ec='black')
        print(x)
        plt.plot(robot[0, :timestep], robot[1, :timestep], '--r',)

    else:
        circle = plt.Circle((x, y), radius, color='green', ec='black')
        plt.plot(robot[0, :timestep], robot[1, :timestep], 'blue')


    plt.gcf().gca().add_artist(circle)

