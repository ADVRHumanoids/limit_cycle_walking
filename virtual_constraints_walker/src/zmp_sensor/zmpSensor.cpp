#include <zmp_sensor/zmpSensor.h>

#define DEFAULT_Fzmin 10


zmpSensor::zmpSensor(XBot::ForceTorqueSensor::ConstPtr ftSensorL,
                     XBot::ForceTorqueSensor::ConstPtr ftSensorR,
                     Eigen::Affine3d w_T_Lfoot,
                     Eigen::Affine3d w_T_Rfoot,
                     Eigen::Affine3d w_T_Lankle,
                     Eigen::Affine3d w_T_Rankle,
                     const double dt) :
_ftSensorL(ftSensorL),
_ftSensorR(ftSensorR),
_dt(dt),
_Fz_min(DEFAULT_Fzmin),
_w_T_Lfoot(w_T_Lfoot),
_w_T_Rfoot(w_T_Rfoot),
_w_T_Lankle(w_T_Lankle),  
_w_T_Rankle(w_T_Rankle)
{
    _logger = XBot::MatLogger::getLogger("/tmp/zmp_sensor_log");
    
    L_T_ZMP_L.setZero();
    R_T_ZMP_R.setZero();
    _ZMP.setZero();
    
    /* get T feet w.r.t. ankle */
    _ankle_T_foot_L = _w_T_Lankle.inverse() * _w_T_Lfoot;
    _ankle_T_foot_R = _w_T_Rankle.inverse() * _w_T_Rfoot;
}

void zmpSensor::update(Eigen::Affine3d w_T_Lfoot, Eigen::Affine3d w_T_Rfoot)
{
    
    _w_T_Lfoot = w_T_Lfoot;
    _w_T_Rfoot = w_T_Rfoot;
    
    
    
    _ftSensorL->getWrench(_FT_foot_L);
    _ftSensorR->getWrench(_FT_foot_R);
    
    /* get ZMP w.r.t. foot */
    if (_FT_foot_L(2) >= _Fz_min)
    {
        L_T_ZMP_L = computeFootZMP(_FT_foot_L, _ankle_T_foot_L.translation());
    }
    
    if (_FT_foot_R(2) >= _Fz_min)
    {
        R_T_ZMP_R = computeFootZMP(_FT_foot_R, _ankle_T_foot_R.translation());
    }
    
    std::cout << "w_T_Lfoot: " << _w_T_Lfoot.translation().transpose() << std::endl;
    std::cout << "w_T_Rfoot: " << _w_T_Rfoot.translation().transpose()  << std::endl;
    std::cout << "ZMP_L: " <<L_T_ZMP_L.transpose() << std::endl;
    std::cout << "ZMP_R: " <<R_T_ZMP_R.transpose()  << std::endl;
    
    /* get ZMP w.r.t. world */
    Eigen::Vector3d w_T_ZMP_L = L_T_ZMP_L + _w_T_Lfoot.translation();
    Eigen::Vector3d w_T_ZMP_R = R_T_ZMP_R + _w_T_Rfoot.translation();
   
    /* get ZMP w.r.t. LEFT FOOT */
    
//     Eigen::Affine3d R_T_Lfoot = w_T_Rfoot.inverse() * w_T_Lfoot;
//     Eigen::Vector3d R_T_ZMP_L = L_T_ZMP_L + R_T_Lfoot.translation();
    
    _ZMP = computeZMP(w_T_ZMP_L, w_T_ZMP_R, _FT_foot_L, _FT_foot_R);
    
//     _ZMP = computeZMP(R_T_ZMP_L, R_T_ZMP_R, _FT_foot_L, _FT_foot_R);
    
    std::cout << "ZMP: " << _ZMP.transpose() << std::endl;
    std::cout << "------" << std::endl;
    
    // log 
    _logger->add("ZMP", _ZMP);
    _logger->add("ZMP_L", L_T_ZMP_L);
    _logger->add("ZMP_R", R_T_ZMP_R);
    _logger->add("FT_foot_L", _FT_foot_L);
    _logger->add("FT_foot_R", _FT_foot_R);
    
    
}

