#include <ros/walker_executor.h>

#include <RobotInterfaceROS/ConfigFromParam.h>

#include <thread>

WalkerExecutor::WalkerExecutor() :
    _nh("walker")
{
//    _feet_links = {"l_sole", "r_sole"};

    /* Create logger */
    XBot::MatLogger2::Options logger_opt;
    logger_opt.default_buffer_size = 1e5;
    _logger = XBot::MatLogger2::MakeLogger("/tmp/walker_log", logger_opt);
    _logger->set_buffer_mode(XBot::VariableBuffer::Mode::circular_buffer);

    _time = 0;
    _period = 0.01;
    _set_flag = 0;
    _start_flag = 0;
    _stop_flag = 0;
    _path_to_cfg = WALKER_CONFIG_PATH;

    /* init ros stuff */
    init_services();
    /* init model and robot */
    init_load_model_and_robot();
    /* init cartesian interface */
    init_load_cartesian_interface();
    /* init walker */
    init_load_walker();
    /* get state of the robot */
    init_load_robot_state();
    /* homing of the robot */
    homing();
    /* initialize walker with current state of the robot */
    init_initialize_walker();
}

void WalkerExecutor::run()
{

    ros::spinOnce();

//    _robot->sense();

    /* update robot state */
    updateRobotState();

    /* update walker */
    _wlkr->update(_time, _state, _ref);

    /* set reference to ci */
    Eigen::Affine3d com;
    com.translation() = _ref.world_T_com;
    com.linear().setIdentity();

    _tasks["l_sole"]->setPoseReferenceRaw(_ref.world_T_foot[0]);
    _tasks["r_sole"]->setPoseReferenceRaw(_ref.world_T_foot[1]);
    _tasks["Com"]->setPoseReferenceRaw(com);
    _tasks["Waist"]->setPoseReferenceRaw(_ref.world_T_waist);
    _tasks["torso"]->setPoseReferenceRaw(_ref.world_T_torso);


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
    _model->setJointVelocity(_qdot);
    _model->update();

    /* update robot and send */
    _robot->setReferenceFrom(*_model, XBot::Sync::Position);
    _robot->move();

    _rsp->publishTransforms(ros::Time::now(), "ci");

    std::this_thread::sleep_for(std::chrono::duration<double>(_period));
    _time += _period;

    log(_logger);
    _set_flag = 0;
    _start_flag = 0;
    _stop_flag = 0;
}

bool WalkerExecutor::homing()
{
    _wlkr->homing(_state, _ref);

    double reaching_time = 1;

    /* feet */
   _tasks["l_sole"]->setPoseTarget(_ref.world_T_foot[0], reaching_time);
   _tasks["r_sole"]->setPoseTarget(_ref.world_T_foot[1], reaching_time);
   /* com */
   Eigen::Affine3d com;
   com.translation() = _ref.world_T_com;
   com.linear().setIdentity();
   _tasks["Com"]->setPoseTarget(com, reaching_time);

   std::cout << "Homing.." << std::endl;

   while (_tasks["l_sole"]->getTaskState() != XBot::Cartesian::State::Online &&
          _tasks["r_sole"]->getTaskState() != XBot::Cartesian::State::Online &&
          _tasks["Com"]->getTaskState() != XBot::Cartesian::State::Online)
   {
       /* update ci */
       _ci->update(_time, _period);

       /* integrate solution */
       _model->getJointPosition(_q);
       _model->getJointVelocity(_qdot);
       _model->getJointAcceleration(_qddot);

       _q += _period * _qdot + 0.5 * std::pow(_period, 2) * _qddot;
       _qdot += _period * _qddot;

       /* set joint poition to model */
       _model->setJointPosition(_q);
       _model->setJointVelocity(_qdot);
       _model->update();

       /* update robot and send */
       _robot->setReferenceFrom(*_model, XBot::Sync::Position);
       _robot->move();

       std::this_thread::sleep_for(std::chrono::duration<double>(_period));
       _time += _period;
   }

   updateRobotState();

   std::cout << "Homing pose reached. " << std::endl;
   /* update state */

   return true;
}

void WalkerExecutor::log(XBot::MatLogger2::Ptr logger)
{
    _state.log("state", logger);
    _ref.log("ref", logger);
    _wlkr->log("walker", logger);

    logger->add("q", _q);
    logger->add("q_dot", _qdot);
    logger->add("q_ddot", _qddot);
//    logger->add("l_ankle", _world_T_ankle[0].translation());
//    logger->add("r_ankle", _world_T_ankle[1].translation());
    logger->add("time", _time);
    logger->add("set_flag", _set_flag);
    logger->add("start_flag", _start_flag);
    logger->add("stop_flag", _stop_flag);
}

WalkerExecutor::~WalkerExecutor()
{
}

bool WalkerExecutor::updateRobotState()
{
    Eigen::Affine3d com;
    /* from reference */
    _tasks["l_sole"]->getPoseReferenceRaw(_state.world_T_foot[0]);
    _tasks["r_sole"]->getPoseReferenceRaw(_state.world_T_foot[1]);
    _tasks["Com"]->getPoseReferenceRaw(com);
    _tasks["Waist"]->getPoseReferenceRaw(_state.world_T_waist);
    _tasks["torso"]->getPoseReferenceRaw(_state.world_T_torso);

    _state.world_T_com = com.translation();
    /* from model */
//    std::array<Eigen::Affine3d, 2> world_T_ankle;
//    Eigen::Vector3d world_T_com;

    _model->getPose("l_ankle", _state.world_T_ankle[0]);
    _model->getPose("r_ankle", _state.world_T_ankle[1]);

//    _model->getCOM(world_T_com);

    _state.ankle_T_com[0].translation() = _state.world_T_ankle[0].translation() - _state.world_T_com;
    _state.ankle_T_com[0].linear() = _state.world_T_ankle[0].linear();
    _state.ankle_T_com[1].translation() = _state.world_T_ankle[1].translation() - _state.world_T_com;
    _state.ankle_T_com[1].linear() = _state.world_T_ankle[1].linear();

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

    _model->getJointPosition(_q);

    _rsp = std::make_shared<XBot::Cartesian::Utils::RobotStatePublisher>(_model);

//    XBot::JointNameMap joint_map;
//    _model->getJointPosition(joint_map);
//    std::cout << joint_map["RankleRoll"] << std::endl;


    /* set FLOATING BASE? */
//    Eigen::Affine3d w_T_l;
//    w_T_l.setIdentity();
//    w_T_l.translation() = get_lfoot_pos();

//    Eigen::Affine3d w_T_pelvis;
//    _model->getFloatingBasePose(w_T_pelvis);

//    _model->setFloatingBasePose(w_T_pelvis*w_T_l.inverse());
    //    _model->update()
}

//void WalkerExecutor::init_load_model_from_launchfile()
//{
//    // an option structure which is needed to make a model
//    XBot::ConfigOptions xbot_cfg;

//    // set the urdf and srdf path..
//    xbot_cfg.set_urdf_path(URDF_PATH);
//    xbot_cfg.set_srdf_path(SRDF_PATH);

//    // the following call is needed to generate some default joint IDs
//    xbot_cfg.generate_jidmap();

//    // some additional parameters..
//    xbot_cfg.set_parameter("is_model_floating_base", true);
//    xbot_cfg.set_parameter<std::string>("model_type", "RBDL");

//    // and we can make the model class
//    auto model = XBot::ModelInterface::getModel(xbot_cfg);


//    ros::NodeHandle nh("xbotcore");
//    auto cfg = XBot::ConfigOptionsFromParamServer(nh);

//    // and we can make the model class
//    auto model = XBot::ModelInterface::getModel(cfg);
//    _rsp = std::make_shared<XBot::Cartesian::Utils::RobotStatePublisher>(model);

//}


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
    _model->getPose("l_sole", _state.world_T_foot[0]);
    _model->getPose("r_sole", _state.world_T_foot[1]);
    _model->getCOM(_state.world_T_com);
    _model->getPose("Waist", _state.world_T_waist);
    _model->getPose("torso", _state.world_T_torso);
    _model->getPose("l_ankle", _state.world_T_ankle[0]);
    _model->getPose("r_ankle", _state.world_T_ankle[1]);


    _state.ankle_T_com[0].translation()= _state.world_T_ankle[0].translation() - _state.world_T_com;
    _state.ankle_T_com[0].linear() = _state.world_T_ankle[0].linear();
    _state.ankle_T_com[1].translation() = _state.world_T_ankle[1].translation() - _state.world_T_com;
    _state.ankle_T_com[1].linear() = _state.world_T_ankle[1].linear();

    _ci->reset(_time);
}

void WalkerExecutor::init_load_cartesian_interface()
{
    std::cout << _path_to_cfg << std::endl;
    YAML::Node ik_yaml = YAML::LoadFile(_path_to_cfg + "cartesio_stack.yaml");


    auto ctx = std::make_shared<XBot::Cartesian::Context>(
                std::make_shared<XBot::Cartesian::Parameters>(_period),
                _model);

    XBot::Cartesian::ProblemDescription ik_problem(ik_yaml, ctx);

    _ci = XBot::Cartesian::CartesianInterfaceImpl::MakeInstance("OpenSot",
                                                                ik_problem,
                                                                ctx);

    _ci->enableOtg(_period);
    std::cout << "Solver constructed successfully \n";

    for(auto tname : _ci->getTaskList())
    {
        _tasks[tname] = std::dynamic_pointer_cast<XBot::Cartesian::CartesianTask>(_ci->getTask(tname));
    }
}

void WalkerExecutor::init_load_walker()
{
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
    _run_walker_srv = _nh.advertiseService("run", &WalkerExecutor::run_service, this);
    _set_cmd_srv = _nh.advertiseService("set_cmd", &WalkerExecutor::set_cmd_service, this);
}

bool WalkerExecutor::run_service(std_srvs::SetBoolRequest &req,
                                 std_srvs::SetBoolResponse &res)
{
    if (req.data)
    {
        _start_flag = 1;
        std::cout << "START received. Starting ..." << std::endl;
        _wlkr->start();
        res.message = "Walking started";
        res.success = true;
    }
    else
    {
        _stop_flag = 1;
        std::cout << "STOP received. Stopping ..." << std::endl;
        _wlkr->stop();
        res.message = "Walking stopped";
        res.success = true;
    }
    return true;
}

bool WalkerExecutor::set_cmd_service(limit_cycle_walking::SetCommandsRequest &req,
                                      limit_cycle_walking::SetCommandsResponse &res)
{
    _set_flag = 1;

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

    if (!req.dist_feet.empty())
    {
        std::cout << "new 'dist_feet' received: " << std::endl;
        for (auto elem : req.dist_feet)
        {
            std::cout << elem << ", ";
        }
        std::cout << std::endl;
        _wlkr->setDistFeet(req.dist_feet);
        res.success = true;
    }
    else
    {
        std::cout << "Empty 'dist_feet' command." << std::endl;
        res.success = false;
    }

    if (!req.phi.empty())
    {
        std::cout << "new 'phi' received: " << std::endl;
        for (auto elem : req.phi)
        {
            std::cout << elem << ", ";
        }
        std::cout << std::endl;
        _wlkr->setPhi(req.phi);
        res.success = true;
    }
    else
    {
        std::cout << "Empty 'phi' command." << std::endl;
        res.success = false;
    }
    return res.success;
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
