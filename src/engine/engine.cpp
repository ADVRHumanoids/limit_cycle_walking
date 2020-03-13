#include <engine/engine.h>

Engine::Options::Options()
{

}

Engine::Engine(double dt, Options opt) :
    _dt(dt),
    _opt(opt)
{

}

bool Engine::initialize(const mdof::StepState &state)
{
    /* TODO here only HEIGHT COM is used from state! */
    LateralPlane::Options _lat_opt;

    _lat_opt.Q << _opt.mpc_Q;
    _lat_opt.R << _opt.mpc_R;
    _lat_opt.horizon_duration = _opt.horizon_duration;
    /* this is the height of the com from ground (soles) (this should be CONSTANT) */
    _lat_opt.h = state.height_com;
    /* preview window resolution set as the control dt */
    _lat_opt.Ts = _dt;

    _lat = std::make_shared<LateralPlane>(_dt, _lat_opt);
    _sag = std::make_shared<SagittalPlane>(_dt);

    /* TODO check sanity */
    return true;
}

bool Engine::computeCom(double time,
                        const mdof::StepState &state,
                        Eigen::Vector3d& delta_com)
{   
    double q = state.q;
    double q_fake = state.q_fake;
    double q_min = state.q_min;
    double q_max = state.q_max;
    double step_duration = state.step_duration;
    bool disable_step = state.disable_step;
    double t_min = state.t_min;
    double t_max = state.t_max;

    /* for now it is not needed, but it will be if I put here the trajectory generation of step, which now is computed using cartesio */
    //    double step_clearing = state->step_clearance;
    double zmp_current = state.zmp_val_current;
    double zmp_next = state.zmp_val_next;
    /* height of com from ankle */
    double distance_ankle_com = state.distance_ankle_com;



    /* compute stepping motion */
    /* STILL TODO the update of sag with real q*/
    _sag->update(q_fake, distance_ankle_com);

    /* here can be put also middle_zmp and offset_zmp */
    if (!disable_step)
    {
        _lat->update(q, q_min, q_max, zmp_current, zmp_next, step_duration);

        delta_com(0) = _sag->getDeltaCom();
        delta_com(1) = _lat->getDeltaCom();
        delta_com(2) = 0;
    }
    else
    {
        _lat->update(time, t_min, t_max, zmp_current, zmp_next, step_duration);

        delta_com(0) = 0;
        delta_com(1) = _lat->getDeltaCom();
        delta_com(2) = 0;
    }
    return true;
}

bool Engine::computeStep(double time, const mdof::StepState &state, Eigen::Vector3d &delta_foot_tot, Eigen::Vector3d &delta_com_tot)
{

    double q_min = state.q_min;
    double q_max = state.q_max;

    //    bool disable_step = state.disable_step;
    /* height of com from ankle */
    double distance_ankle_com = state.distance_ankle_com;



    /* compute stepping motion */
    /* STILL TODO the update of sag with real q*/
    double delta_q = q_max - q_min;
    _sag->update(delta_q, distance_ankle_com);

    /* here can be put also middle_zmp and offset_zmp */
    //    if (!disable_step)
    //    {
    delta_foot_tot(0) = _sag->getDeltaFoot();
    delta_foot_tot(1) = 0;
    delta_foot_tot(2) = 0;
    //    }
    //    else
    //    {
    //        delta_foot_tot(0) = 0;
    //        delta_foot_tot(1) = 0;
    //        delta_foot_tot(2) = 0;
    //    }

    delta_com_tot(0) = _sag->getDeltaCom();
    delta_com_tot(1) = 0;
    delta_com_tot(2) = 0;

    return true;

}

void Engine::log(std::string name, XBot::MatLogger::Ptr logger)
{
    logger->add(name + "_dt", _dt);

    _lat->log("lat", logger);
    _sag->log("sag", logger);
}

