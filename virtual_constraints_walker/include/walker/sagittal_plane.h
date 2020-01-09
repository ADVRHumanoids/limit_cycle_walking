#ifndef SAGITTAL_PLANE_H
#define SAGITTAL_PLANE_H

#include <XBotLogger/Logger.hpp>

class SagittalPlane {
public:

    SagittalPlane();

    void update(/* . q_fake . */);

    Eigen::Vector3d getDeltaComSag(){return _delta_com_sag;}
    Eigen::Affine3d getDeltaFoot(){return _delta_foot;}

private:

    void virtualConstraints(/* ... */);


    bool computeStep();


    Eigen::Vector3d _delta_com_sag;
    Eigen::Affine3d _delta_foot;





};

#endif // SAGITTAL_PLANE_H
