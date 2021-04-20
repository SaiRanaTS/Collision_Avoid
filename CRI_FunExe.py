# This Module contains the function execution calls

import CRI_Functions


def CRI_call(Own_Ship_v, Trg_Ship_v, Own_Ship_Xpos, Own_Ship_Ypos, Trg_Ship_Xpos, Trg_Ship_Ypos, Own_Ship_ang,
             Trg_Ship_ang):
    ###################################################################################################
    #                             C R I    F U N C T I O N S
    ###################################################################################################

    # function test 1 - Intermidiate para

    Inter = CRI_Functions.Inter_Para(Own_Ship_v, Trg_Ship_v, Own_Ship_Xpos, Own_Ship_Ypos, Trg_Ship_Xpos, Trg_Ship_Ypos,
                                     Own_Ship_ang, Trg_Ship_ang)
    Vox = Inter[0]
    Voy = Inter[1]
    Vtx = Inter[2]
    Vty = Inter[3]
    Vx = Inter[4]
    Vy = Inter[5]
    Xot = Inter[6]
    Yot = Inter[7]

    ###################################################################################################

    # function test 2 - Relative Speed

    VTO = CRI_Functions.relative_Velo(Own_Ship_v, Trg_Ship_v, Own_Ship_ang, Trg_Ship_ang)

    # function test 3 - The Relative course and True course

    RC_TC = CRI_Functions.relative_Cor_TarS(Vx, Vy, Xot, Yot)
    theta_ot = RC_TC[0]
    alpha_t = RC_TC[1]


    ###################################################################################################

    # function test 4 - Relative Motion Parameters : DCPA TCPA and Alpha OT

    DTA = CRI_Functions.Relat_Motion_para(Own_Ship_Xpos, Own_Ship_Ypos, Trg_Ship_Xpos, Trg_Ship_Ypos, Own_Ship_ang, theta_ot,
                                          alpha_t, VTO)

    DCPA = DTA[0]
    TCPA = DTA[1]
    AlphaOT = DTA[2]
    D_btwnS = DTA[3]



    ###################################################################################################

    # function test 5 - d1 and d2
    # For this we need to fix a K avale between 1.5 and 2.0, for this case we fixed K =2.0

    k = 2
    d1_d2 = CRI_Functions.d1_and_d2(AlphaOT, k)
    d1 = d1_d2[0]
    d2 = d1_d2[1]

    ###################################################################################################
    # Function test 6 - D1 D2 and t1 t2
    L = 100  # Length of the giveway ship

    dts = CRI_Functions.D1_D2_t1_t2(DCPA, L, VTO, AlphaOT,d1,d2)
    D1_value = dts[0]
    D2_value = dts[1]
    t1_value = dts[2]
    t2_value = dts[3]

    ###################################################################################################
    #                 M E M B E R S H I P     F U N C T I O N S
    ###################################################################################################

    # Function test for MF TCPA

    MF_TC = CRI_Functions.MF_TCPA(TCPA, t1_value, t2_value)
    # print('---------------------------------------------')
    #print('The Membership Function for TCPA is : ', MF_TC)

    # Function test for MF Relative Distance

    MF_RD = CRI_Functions.MF_Rel_dis(D1_value, D2_value, D_btwnS)
    #print('---------------------------------------------')
    #print('The Membership Function for relative distance : ', MF_RD)

    MF_B = CRI_Functions.MF_realtive_bearing(AlphaOT)
    #print('---------------------------------------------')
    #print('The Membership Function for relative bearing is : ', MF_B)

    MF_DCPAz = CRI_Functions.MF_DCPA(DCPA, d1, d2)
    # print('---------------------------------------------')
    #print('The Membership Function for DCPA is : ', MF_DCPAz)

    MF_Kz = CRI_Functions.MF_K(k, Trg_Ship_ang, Own_Ship_ang)
    #print('---------------------------------------------')
    #print('The Membership Function for K is : ', MF_Kz)

    CRI_Index = CRI_Functions.CRI_F(MF_DCPAz, MF_TC, MF_RD, MF_B, MF_Kz)
    # print('---------------------------------------------')
    # print('The CRI Index is : ', CRI_Index )
    new_alpha_t =0
    new_dcpa_chk =0
    Data = (DCPA,CRI_Index,TCPA, d1,MF_DCPAz,AlphaOT,d1,d2,D_btwnS,theta_ot,alpha_t,MF_TC,t1_value,t2_value,new_alpha_t,new_dcpa_chk,Own_Ship_v,Trg_Ship_v,Own_Ship_ang,Trg_Ship_ang,Own_Ship_Xpos,Own_Ship_Ypos,Trg_Ship_Xpos,Trg_Ship_Ypos)
    return Data

