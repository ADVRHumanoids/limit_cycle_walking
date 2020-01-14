#include <walker/sagittal_plane.h>

void SagittalPlane::update()
{




}

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
                                double step_duration,
                                double height_com,
                                double& steep_coeff,
                                Eigen::Vector3d inital_com,
                                Eigen::Vector3d& final_com,
                                Eigen::Affine3d initial_swing_foot,
                                Eigen::Affine3d initial_stance_foot,
                                Eigen::Affine3d& final_swing_foot,
                                Eigen::Affine3d& final_stance_foot,
                                Eigen::Affine3d& final_waist)
{
    Eigen::Vector2d disp_com;
    Eigen::Rotation2Dd rot2(theta);

    /* total q angle in one step */
    double q_tot = q_max - q_min;

    steep_coeff = (q_max - q_min)/ step_duration;

    /* displacement in the xy plane */
    double delta_com = fabs(height_com) * tan(q_tot);


    std::cout << "disp_com_before_rot: " << disp_com.transpose() << std::endl;

    disp_com = rot2.toRotationMatrix() * disp_com;

    std::cout << "disp_com_after_rot: " << disp_com.transpose() << std::endl;

    /* compute com final pose */
    final_com.head(2) = inital_com.head(2) + disp_com;

    /* compute step final pose */
    final_stance_foot = initial_stance_foot;
    final_swing_foot.translation() = - initial_stance_foot.translation()  + 2 * final_com;  // get final step given the displacement vector

    /*rotate, if needed, sole */
    final_swing_foot.linear() = (Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ())).toRotationMatrix();

    /* rotate, if needed, waist */
    final_waist.linear() = (Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ())).toRotationMatrix();
}

Eigen::Affine3d SagittalPlane::getFootStart() const
{
    return _foot_start;
}

Eigen::Affine3d SagittalPlane::getFootEnd() const
{
    return _foot_end;
}

Eigen::Vector3d SagittalPlane::getComStart() const
{
    return _com_start;
}

Eigen::Vector3d SagittalPlane::getComEnd() const
{
    return _com_end;
}
