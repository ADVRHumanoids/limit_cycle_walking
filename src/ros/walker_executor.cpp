#include <ros/walker_executor.h>

#include <RobotInterfaceROS/ConfigFromParam.h>

#include <cartesian_interface/ros/RosImpl.h>

WalkerExecutor::WalkerExecutor() :
    _nh("walker")
{
//    _feet_links = {"l_sole", "r_sole"};

    _logger = XBot::MatLogger::getLogger("/tmp/walker_log");

    _time = 0;
    _period = 0.01;
    _path_to_cfg = WALKER_CONFIG_PATH;

    /* init ros stuff */
    init_services();
    /* init model and robot */
    init_load_model_and_robot();

    init_load_robot_state();
    /* init cartesian interface */
    init_load_cartesian_interface();
    /* init walker */
    init_load_walker();
    homing();
    init_initialize_walker();
}

void WalkerExecutor::run()
{

    ros::spinOnce();

//    _robot->sense();
//    _model->syncFrom(*_robot, XBot::Sync::Position, XBot::Sync::MotorSide);
    /* update robot state */
    updateRobotState();

    /* update walker */
    _wlkr->update(_time, _state, _ref);

    /* set reference to ci */
    _ci->setPoseReferenceRaw("l_sole", _ref.world_T_foot[0]);
    _ci->setPoseReferenceRaw("r_sole", _ref.world_T_foot[1]);
    _ci->setComPositionReference(_ref.world_T_com);
    _ci->setPoseReferenceRaw("Waist", _ref.world_T_waist);

    /* update ci */
    if(!_ci->update(_time, _period))
    {
        return;
    }

    /* integrate solution */
    _model->getJointPosition(_q);
    _model->getJointVelocity(_qdot);
    _model->getJointAcceleration(_qddot);

    _q += _period * _qdot + 0.5 * std::pow(_period, 2) * _qddot;
    _qdot += _period * _qddot;

    /* set joint poition to model */
    _model->setJointPosition(_q);
    _model->update();

    /* update robot and send */
    _robot->setReferenceFrom(*_model, XBot::Sync::Position);
    _robot->move();

    _time += _period;

}

bool WalkerExecutor::homing()
{
    _wlkr->homing(_state, _ref);

    /* feet */
   _ci->setPoseReference("l_sole", _ref.world_T_foot[0]);
   _ci->setPoseReference("r_sole", _ref.world_T_foot[1]);
    /* com */
   _ci->setComPositionReference(_ref.world_T_com);

   XBot::Cartesian::State l_sole_state =_ci->getTaskState("l_sole");
   XBot::Cartesian::State r_sole_state =_ci->getTaskState("l_sole");
   XBot::Cartesian::State com_state =_ci->getTaskState("l_sole");

   while (l_sole_state != XBot::Cartesian::State::Online)
   {
       std::cout << "Task is reaching.." << std::endl;
   }

   while (r_sole_state != XBot::Cartesian::State::Online)
   {
       std::cout << "Task is reaching.." << std::endl;
   }

   while (com_state != XBot::Cartesian::State::Online)
   {
       std::cout << "Task is reaching.." << std::endl;
   }

    return true;
}

bool WalkerExecutor::updateRobotState()
{
    std::array<Eigen::Affine3d, 2> world_T_ankle;

    _ci->getPoseReferenceRaw("l_sole", _state.world_T_foot[0]);
    _ci->getPoseReferenceRaw("r_sole", _state.world_T_foot[1]);
    _ci->getComPositionReference(_state.world_T_com);
    _ci->getPoseReferenceRaw("Waist", _state.world_T_waist);
    _ci->getPoseReferenceRaw("l_sole", world_T_ankle[0]);
    _ci->getPoseReferenceRaw("r_sole", world_T_ankle[1]);

    _state.ankle_T_com[0].translation()= world_T_ankle[0].translation() - _state.world_T_com;
    _state.ankle_T_com[0].linear() = world_T_ankle[0].linear();
    _state.ankle_T_com[1].translation() = world_T_ankle[1].translation() - _state.world_T_com;
    _state.ankle_T_com[1].linear() = world_T_ankle[1].linear();

    return true;
}

void WalkerExecutor::init_load_model_and_robot()
{
    ros::NodeHandle nh("xbotcore");
    auto cfg = XBot::ConfigOptionsFromParamServer(nh);
    _robot = XBot::RobotInterface::getRobot(cfg);
    _model = XBot::ModelInterface::getModel(cfg);

    _robot->sense();

    _model->syncFrom(*_robot, XBot::Sync::Position, XBot::Sync::MotorSide);

    _q.resize(_model->getJointNum());
    _qdot = _q;

    _model->getJointPosition( _q );

    /* set FLOATING BASE? */
//    Eigen::Affine3d w_T_l;
//    w_T_l.setIdentity();
//    w_T_l.translation() = get_lfoot_pos();

//    Eigen::Affine3d w_T_pelvis;
//    _model->getFloatingBasePose(w_T_pelvis);

//    _model->setFloatingBasePose(w_T_pelvis*w_T_l.inverse());
    //    _model->update()
}

//void WalkerExecutor::init_homing()
//{
//    Eigen::VectorXd qhome;
//    _model->getRobotState("home", qhome);

//    std::cout << qhome.transpose() << std::endl;
//    _model->setJointPosition(qhome);
//    _model->update();

//    _robot->setReferenceFrom(*_model, XBot::Sync::Position);
//    _robot->move();

//}

void WalkerExecutor::init_load_robot_state()
{

    std::array<Eigen::Affine3d, 2> world_T_ankle;

    _model->getPose("l_sole", _state.world_T_foot[0]);
    _model->getPose("r_sole", _state.world_T_foot[1]);
    _model->getCOM(_state.world_T_com);
    _model->getPose("Waist", _state.world_T_waist);
    _model->getPose("l_ankle", world_T_ankle[0]);
    _model->getPose("r_ankle", world_T_ankle[1]);

    _state.ankle_T_com[0].translation()= world_T_ankle[0].translation() - _state.world_T_com;
    _state.ankle_T_com[0].linear() = world_T_ankle[0].linear();
    _state.ankle_T_com[1].translation() = world_T_ankle[1].translation() - _state.world_T_com;
    _state.ankle_T_com[1].linear() = world_T_ankle[1].linear();
}

void WalkerExecutor::init_load_cartesian_interface()
{
    std::cout << _path_to_cfg << std::endl;
    YAML::Node ik_yaml = YAML::LoadFile(_path_to_cfg + "comanPlus_VC_stack.yaml");

    XBot::Cartesian::ProblemDescription ik_problem(ik_yaml, _model);
    _ci = std::make_shared<XBot::Cartesian::CartesianInterfaceImpl>(_model, ik_problem);

//    XBot::Cartesian::RosImpl::Ptr ci;
//    ci = std::make_shared<XBot::Cartesian::RosImpl>();
    _ci->enableOtg(_period);
}


void WalkerExecutor::init_load_walker()
{
    //_walker_par = std::make_shared<Walker::Param>(_nh);
    YAML::Node walking_yaml = YAML::LoadFile(_path_to_cfg + "walking_config.yaml");
    auto wlkr_par = std::make_shared<Walker::Param>(walking_yaml);
    _wlkr = std::make_shared<Walker>(_period, wlkr_par);
}

void WalkerExecutor::init_initialize_walker()
{
    /* TODO */
    _wlkr->init(_state);
}


void WalkerExecutor::init_services()
{
    _run_walker_srv = _nh.advertiseService("run_walker", &WalkerExecutor::run_walker_service, this);
    _set_q_max_cmd_srv = _nh.advertiseService("set_qmax", &WalkerExecutor::set_qmax_service, this);
    _set_theta_cmd_srv = _nh.advertiseService("set_theta", &WalkerExecutor::set_theta_service, this);
    _set_cmd_srv = _nh.advertiseService("set_cmd", &WalkerExecutor::set_cmd_service, this);
}

bool WalkerExecutor::run_walker_service(std_srvs::SetBoolRequest &req,
                                         std_srvs::SetBoolResponse &res)
{
    if (req.data)
    {
        std::cout << "START received. Starting ..." << std::endl;
        _wlkr->start();
        res.message = "Walking started";
        res.success = true;
    }
    else
    {
        std::cout << "STOP received. Stopping ..." << std::endl;
        _wlkr->stop();
        res.message = "Walking stopped";
        res.success = true;
    }
    return true;
}

bool WalkerExecutor::set_qmax_service(limit_cycle_walking::SetCommandsRequest &req,
                                       limit_cycle_walking::SetCommandsResponse &res)
{
    if (!req.q_max.empty())
    {
        std::cout << "new 'q_max' received: " << std::endl;
        for (auto elem : req.q_max)
        {
            std::cout << elem << ", ";
        }
        std::cout << std::endl;
        _wlkr->setQMax(req.q_max);
        res.success = true;
    }
    else
    {
        std::cout << "Empty 'q_max' command." << std::endl;
        res.success = false;
    }
    return res.success;
}

bool WalkerExecutor::set_theta_service(limit_cycle_walking::SetCommandsRequest &req,
                                        limit_cycle_walking::SetCommandsResponse &res)
{
    if (!req.theta.empty())
    {
        std::cout << "new 'theta' received: " << std::endl;
        for (auto elem : req.theta)
        {
            std::cout << elem << ", ";
        }
        std::cout << std::endl;
        _wlkr->setTheta(req.theta);
        res.success = true;
    }
    else
    {
        std::cout << "Empty 'theta' command." << std::endl;
        res.success = false;
    }
    return res.success;
}

bool WalkerExecutor::set_cmd_service(limit_cycle_walking::SetCommandsRequest &req,
                                      limit_cycle_walking::SetCommandsResponse &res)
{
    //    cmd_all
}

bool WalkerExecutor::homing_service(std_srvs::SetBoolRequest& req,
                             std_srvs::SetBoolResponse& res)
{
    if (req.data)
    {
        if (_wlkr->getState() == Walker::State::Idle)
        {
            std::cout << "Homing permission granted. Homing..." << std::endl;
            homing();
            res.success = true;
        }
        else
        {
            std::cout << "Homing permission denied. You can issue 'homing only during State 'Idle'" << std::endl;
            res.success = false;
        }
    }

    return true;
}
