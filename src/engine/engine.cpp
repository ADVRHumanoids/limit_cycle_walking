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
    /* for sagittal */
    double q_sag = state.q_sag;
    double distance_ankle_com = state.distance_ankle_com;
    /* for lateral */
    double q_lat = state.q_lat;
    double q_lat_min = state.q_lat_min;
    double q_lat_max = state.q_lat_max;
    std::vector<Eigen::MatrixXd> zmp_vals = state.zmp_vals;
    Eigen::VectorXd durations = state.durations;

    /* compute stepping motion */
    /* STILL TODO the update of sag with real q*/ /* this was q_fake and now it's q_sag */
    _sag->update(q_sag, distance_ankle_com);
    _lat->update(q_lat, q_lat_min, q_lat_max, zmp_vals, durations);

    delta_com(0) = _sag->getDeltaCom();
    delta_com(1) = _lat->getDeltaCom();
    delta_com(2) = 0;

    return true;
}

bool Engine::computeStep(double time, const mdof::StepState &state, Eigen::Vector3d &delta_foot_tot, Eigen::Vector3d &delta_com_tot)
{

    double q_sag_min = state.q_sag_min;
    double q_sag_max = state.q_sag_max;

    /* height of com from ankle */
    double distance_ankle_com = state.distance_ankle_com;

    /* compute stepping motion */
    /* STILL TODO the update of sag with real q*/
    double delta_q = q_sag_max - q_sag_min;
    _sag->update(delta_q, distance_ankle_com);

    delta_foot_tot(0) = _sag->getDeltaFoot();
    delta_foot_tot(1) = 0;
    delta_foot_tot(2) = 0;


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

