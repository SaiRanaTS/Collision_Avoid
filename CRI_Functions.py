#!/usr/bin/env python
# coding: utf-8

# In[ ]:
import math
import numpy as np


############################################################################################
# Intermidiate Parameters Function

def Inter_Para(O_v, T_v, O_x, O_y, T_x, T_y, O_ang, T_ang):

    vox = round(O_v * math.sin(math.radians(O_ang)), 3)
    voy = round(O_v * math.cos(math.radians(O_ang)), 3)

    vtx = round(T_v * math.sin(math.radians(T_ang)), 3)
    vty = round(T_v * math.cos(math.radians(T_ang)), 3)

    vx = vtx - vox
    vxr = round(vx, 3)
    vy = vty - voy
    vyr = round(vy, 3)

    xot = T_x - O_x
    yot = T_y - O_y

    result_1 = (vox, voy, vtx, vty, vxr, vyr, xot, yot)
    return result_1


############################################################################################

# function Relative Speed
def relative_Velo(V_Ox, V_T, theta_O, theta_T):
    if V_Ox == 0:
        V_O = 0.00001
    else:
        V_O = V_Ox
    velo_ratio_re = (V_T / V_O)
    diff_angle_re = theta_O - theta_T
    cos_diff_re = math.cos(math.radians(diff_angle_re))
    velo_Ratio_sqr_re = velo_ratio_re ** 2
    inside_f_re = (1 + velo_Ratio_sqr_re - 2 * velo_ratio_re * cos_diff_re)
    Vot_re = round(V_O* math.sqrt(inside_f_re), 3)
    return Vot_re

############################################################################################

# function The Relative course and True course
def relative_Cor_TarS(VTx, VTy, Xot1, Yot1):
    #--------------------Theta OT ----------------------
    vx = VTx
    vy = VTy
    if vy == 0:
        vy = 0.000001
    else:
        vy = VTy
    vx_vy_ratio = vx / vy

    def thetaot(vr3,vx,vy):
        if vx >= 0 and vy >= 0:
            res1 = round(math.degrees(math.atan(vr3)),3)
            #print('Option 1', res1)
            return (res1)
        elif vx < 0 and vy < 0:
            res2 = round(math.degrees(math.atan(vr3) + math.pi),3)
            #print('Option 2',res2)
            return (res2)
        elif vx >= 0 and vy < 0:
            res3 = round(math.degrees(math.atan(vr3) + math.pi),3)
            #print('Option 3',res3)
            return (res3)
        elif vx < 0 and vy >= 0:
            res4 = round(math.degrees(math.atan(vr3) + 2*(math.pi)),3)
            #print('Option 4',res4)
            return (res4)

    theta_OT_Deg = thetaot(vx_vy_ratio,vx,vy)
    # if theta_OT_Deg < 0:
    #     theta_OT_Deg = 360 + theta_OT_Deg
    # elif theta_OT_Deg >= 360:
    #     theta_OT_Deg = theta_OT_Deg - 360
    # else:
    #     theta_OT_Deg = theta_OT_Deg

#-----------------------Alpha T -------------------------
    Xot = Xot1
    Yot = Yot1
    if Yot1 == 0:
        Yot = 0.000001
    else:
        Yot = Yot1

    xot_yot_ratio = Xot / Yot

    def Alpha_T(vr5,Xot,Yot):
        if Xot >= 0 and Yot >= 0:
            res1a = round(math.degrees(math.atan(vr5)),3)
            #print('Option 1', res1a)
            return (res1a)
        elif Xot < 0 and Yot < 0:
            res2a =  round(math.degrees(math.atan(vr5) + (math.pi) ),3)
            #print('Option 2',res2a)
            return (res2a)
        elif Xot >= 0 and Yot < 0:
            res3a =  round(math.degrees(math.atan(vr5) + (math.pi)),3)
            #print('Option 3',res3a)
            return (res3a)
        elif Xot < 0 and Yot >= 0:
            res4a =  round(math.degrees(math.atan(vr5) + (2*math.pi)),3)
            #print('Option 4',res4a)
            return (res4a)

    alpha_t_Deg = Alpha_T(xot_yot_ratio,Xot,Yot)
    # if  alpha_t_Deg < 0:
    #     alpha_t_Deg = 360 + alpha_t_Deg
    # elif alpha_t_Deg >= 360:
    #     alpha_t_Deg = alpha_t_Deg - 360
    # else:
    #     alpha_t_Deg = alpha_t_Deg

    result_2 = (theta_OT_Deg, alpha_t_Deg)
    return result_2






############################################################################################

# Relative Motion Parameters
def Relat_Motion_para(Xo, Yo, Xt, Yt, theta_o, theta_ot, alpha_t, Vott):

    Vot_re = Vott
    if Vot_re == 0:
        Vot_re = 0.00001
    else:
        Vot_re = Vott


    # Distance Between the vessels
    D_Btwn = math.sqrt(((Xt - Xo) ** 2) + ((Yt - Yo) ** 2))

    # DCPA Calculation
    inang_dcpa = theta_ot - alpha_t -180
    #print('inage :', inang_dcpa)
    ang_D = math.sin(math.radians( inang_dcpa))
    DCPA_1 = D_Btwn * ang_D
    DCPA_rnd = round(DCPA_1, 5)

    # TPCA Calculation
    top_fun = math.cos(math.radians(theta_ot - alpha_t -180 ))
    TCPA_1 = ((D_Btwn * top_fun) / Vot_re)
    TCPA_rnd = round(TCPA_1, 5)


    sudo_alpha_OT = (theta_o - theta_ot)
    alpha_OT = alpha_t - theta_o
    if alpha_OT < 0 :
        alpha_OT = alpha_OT + 360
    elif alpha_OT > 360 :
        alpha_OT = alpha_OT - 360
    else:
        pass

    result_3 = (DCPA_rnd, TCPA_rnd, alpha_OT, D_Btwn)
    return result_3




############################################################################################

# Calculation of d1 and d2
def d1_and_d2(alphaot, k):

    def d1_cal(alp_3):
        if alp_3 >= 0 and alp_3 < 67.5:
            res_d1 = 1.1 - ((0.2 *alphaot)/180)
            #print("Here the value of Alpha OT is ", alp_3, " which lies between 0 and 112.5")
            return (res_d1)
        elif alp_3 >= 67.5 and alp_3 < 112.5:
            res_d2 = 1.0 - ((0.6 *alphaot)/180)
            #print("Here the value of Alpha OT is ", alp_3, " which lies between 112.5 and 180")
            return (res_d2)
        elif alp_3 >= 112.5 and alp_3 < 247.5:
            res_d3 = 1.0 - ((0.6 *(360 - alphaot))/180)
            #print("Here the value of Alpha OT is ", alp_3, " which lies between 180 and 247.5")
            return (res_d3)
        elif alp_3 >= 247.5 and alp_3 <= 360:
            res_d4 = 1.1 - ((0.2 *(360 - alphaot))/180)
            #print("Here the value of Alpha OT is ", alp_3, " which lies between 247.5 and 360")
            return (res_d4)


    # def d1_cal(alp_3):
    #     if alp_3 >= 0 and alp_3 < 67.5:
    #         res_d1 = 8.0
    #         #print("Here the value of Alpha OT is ", alp_3, " which lies between 0 and 112.5")
    #         return (res_d1)
    #     elif alp_3 >= 67.5 and alp_3 < 112.5:
    #         res_d2 = 0.6
    #         #print("Here the value of Alpha OT is ", alp_3, " which lies between 112.5 and 180")
    #         return (res_d2)
    #     elif alp_3 >= 112.5 and alp_3 < 247.5:
    #         res_d3 = 0.3
    #         #print("Here the value of Alpha OT is ", alp_3, " which lies between 180 and 247.5")
    #         return (res_d3)
    #     elif alp_3 >= 247.5 and alp_3 <= 360:
    #         res_d4 = 0.5
    #         #print("Here the value of Alpha OT is ", alp_3, " which lies between 247.5 and 360")
    #         return (res_d4)


    d1_u = d1_cal(alphaot)
    d2_u = (k * d1_u)

    result_4 = (d1_u, d2_u)
    return result_4


############################################################################################

# Calculation of D1 D2 and t1 t2
def D1_D2_t1_t2(DCPA, L, VTOx, AlphaOT,d1,d2):

    if VTOx == 0:
        VTO = 0.00001
    else:
        VTO = VTOx

    L_NM = 0.000539957 * L  # The length of the ship is converted to NM

    D1 = round((12 * L_NM ), 3)  # D1 is taken 12 times the giveaway vessel length

    # D2 Calculation
    angle_inside_cos = AlphaOT - 19
    inside_pt1 = 1.7 * math.cos(math.radians(angle_inside_cos))
    inside_root_prt = 4.4 + (2.89 * math.cos(math.radians(angle_inside_cos))**2)
    inside_pt2 = math.sqrt(inside_root_prt)
    D2 = round((inside_pt1 + inside_pt2), 3)

    # t1 and t2 calculation

    def t1(DCPA_pas,D1,VTO):

        DCPA = abs(DCPA_pas)

        if (DCPA)<= D1:
            fun1_inside_rootz1 = abs(((D1)**2) - ((DCPA)**2))
            t1z = round(((math.sqrt(fun1_inside_rootz1)) / (2*VTO)), 3)
            return(t1z)
        elif (DCPA) > D1:
            fun_ontop1 = (((D1 - DCPA)))
            t1zz = round(((fun_ontop1) / (2*VTO)), 3)
            return(t1zz)

    def t2(DCPA,D2,VTO):

        if (DCPA) <= D2:
            fun2_inside_rootz2 = abs((((D2) ** 2 - (DCPA) ** 2)))
            t2z = round(((math.sqrt(fun2_inside_rootz2)) / (2*VTO)), 3)
            return(t2z)
        elif (DCPA) > D2:
            fun_ontop2 = (((D2 - DCPA)))
            t2zz = round(((fun_ontop2) / (2*VTO)), 3)
            return(t2zz)

    t1 = t1(DCPA,D1,VTO)
    t2 = t2(DCPA,D2,VTO)


    result_5 = (D1, D2, t1, t2)

    return result_5


############################################################################################

# Membership function for Time to the closest Point of Approach (TCPA)
def MF_TCPA(TCPA_NP, t1zz, t2zz):
    if t2zz < abs(TCPA_NP):
        u_TCPA1 = 0
        return u_TCPA1
    elif t1zz < abs(TCPA_NP) and abs(TCPA_NP) <= t2zz:
        u_TCPA = ((t2zz - abs(TCPA_NP)) / (t2zz - t1zz)) ** 2
        u_TCPA2 = round(u_TCPA,3)
        return u_TCPA2

    elif 0 <= abs(TCPA_NP) and abs(TCPA_NP)<= t1zz:
        u_TCPA3 = 1
        return u_TCPA3



############################################################################################

# Membership function for Relative Distance
def MF_Rel_dis(D1, D2, D):

    if D2 < D:
        Drf1 = 0
        return Drf1
    elif D1 <= D and D <= D2:
        Drf2 = round(((D2 - D) / (D2 - D1))**2,3)
        return Drf2
    elif D <= D1:
        Drf3 = 1
        return Drf3



############################################################################################

# Membership function for Relative bearing
def MF_realtive_bearing(alphaT):
    ang_alf = alphaT
    inside_angPT = (ang_alf - 19)
    indie_sqr_big = (440 / 289) + (math.cos(math.radians(inside_angPT))) ** 2
    u_alphaOT_f = round((0.5 * (math.cos(math.radians(inside_angPT)) + math.sqrt(indie_sqr_big)) - (5 / 17)), 3)
    return u_alphaOT_f


############################################################################################


def MF_DCPA(DCPA, d1, d2):

    if d2 < abs(DCPA):
        mf_dcpa1 = 0
        return mf_dcpa1
    elif d1 < abs(DCPA) and abs(DCPA)<= d2:
        iinnsin = (((math.pi) / (d2 - d1)) * ((abs(DCPA)) - ((d1 + d2) / 2)))
        mf_dcpa2 = round(0.5 - (0.5 * (math.sin(math.radians(iinnsin)))), 4)
        return mf_dcpa2
    elif abs(DCPA) <= d1:
        mf_dcpa3 = 1
        return mf_dcpa3

############################################################################################

# The memeber function for K

def MF_K(K_u, ang1, ang2):
    K_fg = K_u
    sin_ff = abs(math.sin(abs(math.radians(ang1 - ang2))))
    srt_inside_fun22 = (K_fg ** 2) + 1 + (2 * K_fg * sin_ff)
    down_fun221 = K_fg * (math.sqrt(srt_inside_fun22))
    u_kzs = round(((1) / (1 + (2 / down_fun221))), 3)
    return u_kzs


############################################################################################

# Function for Collision Risk Model is defined as follows :
# The Weights are set as 0.400, 0.367, 0.133, 0.067 and 0.033

def CRI_F(u_dcpa_fz, u_TCPA, Drf, u_alphaOT_f, u_kzs):
    UF = np.array([[u_dcpa_fz], [u_TCPA], [Drf], [u_alphaOT_f], [u_kzs]])
    WF = np.array([[float(0.33), float(0.297), float(0.203), float(0.067), float(0.003)]])
    CRF = WF.dot(UF)
    CRF_rn = round(CRF[0][0], 3)
    return CRF_rn

