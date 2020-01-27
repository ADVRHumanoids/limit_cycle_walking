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
    _lat_opt.h = state.height_com;
    /* preview window resolution set as the control dt */
    _lat_opt.Ts = _dt;

    _lat = std::make_shared<LateralPlane>(_dt, _lat_opt);
    _sag = std::make_shared<SagittalPlane>(_dt);

    /* TODO check sanity */
    return true;

}

bool Engine::compute(double time,
                     const mdof::StepState &state,
                      Eigen::Vector3d& delta_com,
                      Eigen::Vector3d& delta_foot_tot)
{   
    double q = state.q;
    double q_min = state.q_min;
    double q_max = state.q_max;
    double step_duration = state.step_duration;

    /* for now it is not needed, but it will be if I put here the trajectory generation of step, which now is computed using cartesio */
//    double step_clearing = state->step_clearance;
    double zmp_current = state.zmp_val_current;
    double zmp_next = state.zmp_val_next;
    double height_com = state.height_com;



    /* compute stepping motion */
    /* STILL TODO the update of sag */
    _sag->update(q, q_min, q_max, height_com);

    /* here can be put also middle_zmp and offset_zmp */
    _lat->update(q, q_max, q_min, zmp_current, zmp_next, _opt.horizon_duration, step_duration);

    delta_com(0) = _sag->getDeltaCom();
    delta_com(1) = _lat->getDeltaCom();
    delta_com(2) = 0;

    delta_foot_tot(0) = _sag->getDeltaFootTot();
    delta_foot_tot(1) = 0;
    delta_foot_tot(2) = 0;

    return true;

}

