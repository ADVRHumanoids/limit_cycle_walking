#ifndef _COMPUTE_ZMP_H_
#define _COMPUTE_ZMP_H_


#include <Eigen/Dense>
#include <iostream>


inline Eigen::Vector3d computeFootZMP(Eigen::Matrix<double, 6, 1> FT_foot, 
                               Eigen::Vector3d sensor_T_foot)
{
    
    /**
    * @brief compute ZMP in a foot w.r.t. the foot
    * 
    * from an FT sensor. The formula used is based on:
    *      "Introduction to Humanoid Robots" by Shuuji Kajita et al., pag. 79-80
    *
    *              ZMPx = (-tau_y -fx*d)/fz
    *              ZMPy = (tau_x - fy*d)/fz
    *              ZMPz = -d
    *
    * where d is the height of the sensor w.r.t. the sole.
    * NOTE: The ZMP position is computed w.r.t. the sensor frame, then transformed into the foot frame.
    *
    * @param FT_foot wrench measured from the FT sensor
    * @param sensor_T_foot T of foot w.r.t. sensor
    * @return a vector with the ZMP position w.r.t. the foot frame
    */
        
    Eigen::Vector3d ZMP_sns;
    ZMP_sns.setZero();
    
    double d = fabs(sensor_T_foot(2)); /* height of the sensor w.r.t the sole frame*/

    /* ZMP w.r.t. the FT sensor frame (ankle)*/
    if ( FT_foot(2) > 0.0)
    {
        ZMP_sns(0) = (- FT_foot(4) - d * FT_foot(0))/ FT_foot[2];
        ZMP_sns(1) = (  FT_foot(3) - d * FT_foot(1))/ FT_foot[2];
        ZMP_sns(2) = - d;
    }
    
    /* ZMP w.r.t. the foot frame */
    Eigen::Vector3d ZMP_foot;
    ZMP_foot = ZMP_sns - sensor_T_foot;
    
    return ZMP_foot;
}
        
inline Eigen::Vector3d computeZMP(const Eigen::Vector3d ZMP_L,
                           const Eigen::Vector3d ZMP_R,
                           Eigen::Matrix<double, 6, 1> FT_foot_L,
                           Eigen::Matrix<double, 6, 1> FT_foot_R)     
{
    /**
    * @brief compute total ZMP given left and right ZMP, weighting it with the sensors of left and right foot
    
    * @param ZMP_L left foot ZMP
    * @param ZMP_R left foot ZMP
    * @param FT_foot wrench measured from the FT sensor
    * @return a vector with the ZMP position
     **/
    
    Eigen::Vector3d ZMP;
    ZMP.setZero();
    
    double Fz_ratio_l = FT_foot_L[2]/(FT_foot_L[2]+FT_foot_R[2]);
    double Fz_ratio_r = FT_foot_R[2]/(FT_foot_L[2]+FT_foot_R[2]);

    ZMP = Fz_ratio_l * ZMP_L + Fz_ratio_r * ZMP_R;
    
    return ZMP;
}
   
#endif