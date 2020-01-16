#include <walker/sagittal_plane.h>

//void SagittalPlane::update(double q,
//                           double height_com,
//                           double q_min,
//                           double q_max,
//                           double theta,
//                           double step_duration,
//                           double& steep_coeff,
//                           Eigen::Vector3d inital_com,
//                           Eigen::Vector3d& final_com,
//                           Eigen::Affine3d initial_swing_foot,
//                           Eigen::Affine3d initial_stance_foot,
//                           Eigen::Affine3d& final_swing_foot,
//                           Eigen::Affine3d& final_stance_foot)
//{


//    computeStep(q_min,q_max, theta, step_duration, height_com, steep_coeff, inital_com, final_com, initial_swing_foot, initial_stance_foot, final_swing_foot);
//    computeCom(q, height_com);


//}

double SagittalPlane::computeCom(double q,
                  double height_com)
{
    /* compute com displacement */
    double delta_com = fabs(height_com) * tan(q);
    return delta_com;
}

bool SagittalPlane::computeStep(double q_min,
                                double q_max,
                                double theta,
                                double height_com,
                                Eigen::Vector3d inital_com,
                                Eigen::Affine3d initial_stance_foot,
                                Eigen::Vector3d& final_com,
                                Eigen::Affine3d& final_swing_foot)
{
    Eigen::Vector2d disp_com;
    Eigen::Rotation2Dd rot2(theta);

    /* total q angle in one step */
    double q_tot = q_max - q_min;

//    steep_coeff = (q_max - q_min)/ step_duration;

    /* displacement in the xy plane */
    disp_com << computeCom(q_tot, height_com), 0;

    std::cout << "disp_com_before_rot: " << disp_com.transpose() << std::endl;

    disp_com = rot2.toRotationMatrix() * disp_com;

    std::cout << "disp_com_after_rot: " << disp_com.transpose() << std::endl;

    /* compute com final pose */
    final_com.head(2) = inital_com.head(2) + disp_com;

    /* compute step final pose */
    final_swing_foot.translation() = - initial_stance_foot.translation()  + 2 * final_com;  // get final step given the displacement vector

    /*rotate, if needed, sole */
    final_swing_foot.linear() = (Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ())).toRotationMatrix();

    return true;
}

Eigen::Affine3d SagittalPlane::getFootGoal() const
{
    return _foot_goal;
}

Eigen::Vector3d SagittalPlane::getComGoal() const
{
    return _com_goal;
}
