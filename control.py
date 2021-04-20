import numpy as np

def compute_desired_velocity(current_pos, goal_pos, robot_radius, vmax):
    disp_vec = (goal_pos - current_pos)[:2] # Displacement vector
    norm = np.linalg.norm(disp_vec) #l1 norm - distance
    if norm < robot_radius / 2: # reached the destination
        return np.zeros(2)
    disp_vec = disp_vec / norm
    np.shape(disp_vec)
    desired_vel = vmax * disp_vec
    return desired_vel #returns desired velocity along x and y for the bot
