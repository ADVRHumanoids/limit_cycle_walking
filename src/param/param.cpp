#include <param/param.h>

Walker::Param::Param()
{
    /* init parameters */
    _initial_lowering = -0.12;
    _first_side_step = 0;
    _lean_forward = 0;

    /* walking parameters */
    _step_clearance = 0.1;
    _step_duration = 2;
    _double_stance_duration = 0;
    _max_inclination = 0.1;

    /* mpc parameters */
    _mpc_Q = 1000000;
    _mpc_R = 1;
    _horizon_duration = 5;
    _zmp_offset = 0;

}


Walker::Param::Param(ros::NodeHandle nh) : Param()
{

    std::string pd_name = "/walking_parameters";
    std::string walking_parameters;

    if(!nh.hasParam(pd_name) ||
            !nh.getParam(pd_name,
                         walking_parameters))
    {
        throw std::runtime_error("problem_description '" + pd_name + "' parameter missing");
    }
    else
    {
        YAML::Node yaml_node = YAML::Load(walking_parameters);
        parseYAML(yaml_node);
    }
}

void Walker::Param::log(std::string name, XBot::MatLogger2::Ptr logger)
{
    logger->add(name + "_initial_lowering", _initial_lowering);
    logger->add(name + "_first_side_step", _first_side_step);
    logger->add(name + "_lean_forward", _lean_forward);
    logger->add(name + "_step_clearance", _step_clearance);
    logger->add(name + "_step_duration", _step_duration);
    logger->add(name + "_double_stance_duration", _double_stance_duration);
    logger->add(name + "_mpc_Q", _mpc_Q);
    logger->add(name + "_mpc_R", _mpc_R);
    logger->add(name + "_horizon_duration", _horizon_duration);
    logger->add(name + "_zmp_offset", _zmp_offset);
}

Walker::Param::Param(YAML::Node yaml_node) : Param()
{
    parseYAML(yaml_node);
}

bool Walker::Param::parseYAML(YAML::Node yaml_node)
{
    std::string init_par = "init_parameters";
//    std::vector<std::string> init_par_vec = {"initial_lowering", "first_step_side", "lean_forward"};

    std::string walk_par = "walking_parameters";
//     std::vector<std::string> walk_par_vec = {"step_clearance", "step_duration", "double_stance_duration", "max_inclination"};

     std::string mpc_par = "mpc_parameters";
//     std::vector<std::string> mpc_par_vec = {"Q", "R", "horizon_duration", "zmp_offset"};

    YAML::Node init_parameters = yaml_node[init_par];
    if(!init_parameters)
    {
        std::cout << "Missing node '"<< init_par << "' Using default values." << std::endl;
    }
    else
    {
        if(!init_parameters["initial_lowering"])
        {
            std::cout << "Missing in '"<< init_par << "': 'initial_lowering'. Using default --> " << _initial_lowering << std::endl;
        }
        else
        {
            _initial_lowering = init_parameters["initial_lowering"].as<double>();
        }

        if(!init_parameters["first_step_side"])
        {
            std::cout << "Missing in '"<< init_par << "': 'first_step_side'. Using default --> " << _first_side_step << std::endl;
        }
        else
        {
            std::string temp = init_parameters["first_step_side"].as<std::string>();
            if (temp == "Left")
                _first_side_step = 0;
            else if (temp == "Right")
                _first_side_step = 1;
            else std::cout << "unknown side starting command" << std::endl;
        }

        if(!init_parameters["lean_forward"])
        {
            std::cout << "Missing in '"<< init_par << "': 'lean_forward'. Using default --> " << _lean_forward << std::endl;
        }
        else
        {
            _lean_forward = init_parameters["lean_forward"].as<double>();
        }

    }

    YAML::Node walking_parameters = yaml_node["walking_parameters"];
    if(!walking_parameters)
    {
        std::cout << "Missing node 'walking_parameters'. Using default values." << std::endl;
    }
    else
    {
        if(!walking_parameters["step_clearance"])
        {
            std::cout << "Missing in '" << walk_par << "': 'step_clearance'. Using default --> " << _step_clearance << std::endl;
        }
        else
        {
            _step_clearance = walking_parameters["step_clearance"].as<double>();
        }

        if(!walking_parameters["step_duration"])
        {
            std::cout << "Missing in '" << walk_par << "': 'step_duration'. Using default --> " << _step_duration << std::endl;
        }
        else
        {
            _step_duration = walking_parameters["step_duration"].as<double>();
        }

        if(!walking_parameters["double_stance_duration"])
        {
            std::cout << "Missing in '" << walk_par << "': 'double_stance_duration'. Using default --> " << _double_stance_duration << std::endl;
        }
        else
        {
            _double_stance_duration = walking_parameters["double_stance_duration"].as<double>();
        }

        if(!walking_parameters["max_inclination"])
        {
            std::cout << "Missing in '" << walk_par << "': 'max_inclination'. Using default --> " << _max_inclination << std::endl;
        }
        else
        {
            _max_inclination = walking_parameters["max_inclination"].as<double>();
        }
    }

    YAML::Node mpc_parameters = yaml_node["mpc_parameters"];
    if(!mpc_parameters)
    {
        std::cout << "Missing node 'mpc_parameters'. Using default values." << std::endl;
    }
    else
    {
        if(!mpc_parameters["Q"])
        {
            std::cout << "Missing in '" << mpc_par << "': 'Q'. Using default --> " << _mpc_Q << std::endl;
        }
        else
        {
            _mpc_Q = mpc_parameters["Q"].as<double>();
        }

        if(!mpc_parameters["R"])
        {
            std::cout << "Missing in '" << mpc_par << "':'R'. Using default --> " << _mpc_R << std::endl;
        }
        else
        {
            _mpc_R = mpc_parameters["R"].as<double>();
        }

        if(!mpc_parameters["horizon_duration"])
        {
            std::cout << "Missing in '" << mpc_par << "':'horizon_duration'. Using default --> " << _horizon_duration << std::endl;
        }
        else
        {
            _horizon_duration = mpc_parameters["horizon_duration"].as<double>();
        }

        if(!mpc_parameters["zmp_offset"])
        {
            std::cout << "Missing in '" << mpc_par << "':'zmp_offset'. Using default --> " << _zmp_offset << std::endl;
        }
        else
        {
            _zmp_offset = mpc_parameters["zmp_offset"].as<double>();
        }
    }

    return true;
}
