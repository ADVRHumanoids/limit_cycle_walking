#ifndef ENGINE_H
#define ENGINE_H

#include <string>
#include <cartesian_interface/CartesianInterface.h>
#include <robot/step_state.h>

#include <engine/lateral_plane.h>
#include <engine/sagittal_plane.h>

class Engine {
public:

    typedef std::shared_ptr<Engine> Ptr;

    struct Options
    {
        Options();

        double zmp_offset = 0.;
        double horizon_duration = 5.;
        double mpc_Q = 1000000.;
        double mpc_R = 1.;
        double double_stance_duration = 0.;

    };

    Engine(double dt, Options opt = Options());

    bool initialize(const mdof::StepState &state);

    bool computeCom(double time,
                    const mdof::StepState &state,
                    Eigen::Vector3d& delta_com);

    bool computeStep(double time,
                 const mdof::StepState &state,
                 Eigen::Vector3d& delta_foot_tot,
                 Eigen::Vector3d& delta_com_tot);

    void log(std::string name, XBot::MatLogger2::Ptr logger);

private:

    bool reset();

    /* dt inside Engine */
    double _dt;

    LateralPlane::Ptr _lat;
    SagittalPlane::Ptr _sag;

    const Options _opt;

};


#endif // ENGINE_H
