import math
import numpy as np


class usv_model_3(object):
    def CalState(self, ref_path, position, velocity, temp_var, idx, dt):
        ref_pt = ref_path[idx]
        psi_path = math.atan2(ref_path[idx, 1] - ref_path[idx-1, 1],
                              ref_path[idx, 0] - ref_path[idx-1, 0])
        psi_refship = math.atan2(position[1] - ref_path[idx-1, 1],
                                 position[0] - ref_path[idx-1, 0])
        psi_rel = psi_path - psi_refship
        dist_refship = np.hypot(position[0] - ref_path[idx-1, 0],
                                position[1] - ref_path[idx-1, 1])

        psi_temp = temp_var[0]
        psi_temp_heading = temp_var[1]
        cross_temp = temp_var[2]

        cross = math.sin(psi_rel) * dist_refship
        psi_r = (math.pi / 3) * math.tanh(0.01 * cross) + psi_path

        # [TODO] ensure that position[2] is in degree
        rot_matrix = np.array(
            [[math.cos(position[2] * math.pi / 180),
              -math.sin(position[2] * math.pi / 180)],
             [math.sin(position[2] * math.pi / 180),
              math.cos(position[2] * math.pi / 180)]])
        pdot = np.array([velocity[0], velocity[1]])
        XYvel = np.dot(rot_matrix, pdot)

        psi_traj = math.atan2(XYvel[1], XYvel[0])
        psi_err_heading = psi_r - position[2] * math.pi / 180
        psi_err = psi_r - psi_traj

        psi_err_diff = psi_err - psi_temp

        psi_err_heading_diff = psi_err_heading - psi_temp_heading

        if psi_err_diff > math.pi:
            psi_err_diff -= 2 * math.pi
        if psi_err_diff < -math.pi:
            psi_err_diff += 2 * math.pi

        psi_err_dot = psi_err_diff / dt

        if psi_err > math.pi:
            psi_err -= 2 * math.pi
        if psi_err < -math.pi:
            psi_err += 2 * math.pi

        if psi_err_heading_diff > math.pi:
            psi_err_heading_diff = psi_err_heading_diff - 2*math.pi
        if psi_err_heading_diff < -math.pi:
            psi_err_heading_diff = psi_err_heading_diff + 2*math.pi

        psi_err_heading_dot = psi_err_heading_diff / dt

        if psi_err_heading > math.pi:
            psi_err_heading = psi_err_heading - 2*math.pi
        if psi_err_heading < -math.pi:
            psi_err_heading = psi_err_heading + 2*math.pi

        temp_var = np.array([psi_err, psi_err_heading, cross])

        return idx, psi_err, psi_err_dot, psi_err_heading, \
            psi_err_heading_dot, temp_var, cross

    def RudderDyn(self, rudder, action, del_sat, del_dot_sat, dt):
        action = np.clip(action, -del_sat, del_sat)
        del_dot = (action - rudder[0]) / dt
        del_dot = np.clip(del_dot, -del_dot_sat, del_dot_sat)
        rudder[0] += del_dot * dt
        rudder[1] = del_dot

        return rudder

    def GetState(self,
                 psi_err, psi_err_dot,
                 psi_err_heading, psi_err_heading_dot,
                 rudder, cross, action,
                 dist_to_goal, theta_to_goal):
        state = np.array([
            psi_err, psi_err_dot, psi_err_heading, psi_err_heading_dot,
            rudder[0], rudder[1], cross, action, dist_to_goal, theta_to_goal])
        return state
