#include <param/param.h>

Walker::Param::Param()
{
    /* init parameters */
    initial_lowering = -0.12;
    first_side_step = 0;
    lean_forward = 0;

    /* walking parameters */
    clearance_height = 0.1;
    step_duration = 2;
    double_stance_duration = 0;
    max_inclination = 0.1;

    /* mpc parameters */
    mpc_Q = 1000000;
    mpc_R = 1;
    horizon_duration = 5;
    zmp_offset = 0;

}


Walker::Param::Param(ros::NodeHandle nh)
{
    /* initialize default */
    Param();

    std::string first_side_step_string;
    std::string default_first_side_step_string = "Left";
    /* init parameters */
    initial_lowering = nh.param("initial_lowering", initial_lowering);
    first_side_step_string = nh.param("first_step_side", default_first_side_step_string);
    lean_forward = nh.param("lean_forward", lean_forward);

    /* walking parameters */
    clearance_height = nh.param("clearance_step", clearance_height);
    step_duration = nh.param("duration_step", step_duration);
    double_stance_duration = nh.param("double_stance_duration", double_stance_duration);
    max_inclination = nh.param("max_inclination", max_inclination);

    /* mpc parameters */
    mpc_Q = nh.param("mpc_Q", mpc_Q);
    mpc_R = nh.param("mpc_R", mpc_R);
    horizon_duration = nh.param("duration_horizon", horizon_duration);
    zmp_offset = nh.param("zmp_offset", zmp_offset);

    if (first_side_step_string == "Left")
        first_side_step = 0;
    else if (first_side_step_string == "Right")
        first_side_step = 1;
    else std::cout << "unknown side starting command" << std::endl;
}

Walker::Param::Param(YAML::Node yaml_node)
{
    /* initialize default */
    Param();

    std::string init_par = "init_parameters";
    std::string walk_par = "walking_parameters";
    std::string mpc_par = "mpc_parameters";

    YAML::Node init_parameters = yaml_node[init_par];
    if(!init_parameters)
    {
        std::cout << "Missing node '"<< init_par << "' Using default values." << std::endl;
    }
    else
    {
        if(!init_parameters["initial_lowering"])
        {
            std::cout << "Missing in '"<< init_par << "': 'initial_lowering'. Using default --> " << initial_lowering << std::endl;
        }
        if(!init_parameters["first_step_side"])
        {
            std::cout << "Missing in '"<< init_par << "': 'first_step_side'. Using default --> " << first_side_step << std::endl;
        }
        if(!init_parameters["lean_forward"])
        {
            std::cout << "Missing in '"<< init_par << "': 'lean_forward'. Using default --> " << lean_forward << std::endl;
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
            std::cout << "Missing in '" << walk_par << "': 'step_clearance'. Using default --> " << clearance_height << std::endl;
        }
        if(!walking_parameters["step_duration"])
        {
            std::cout << "Missing in '" << walk_par << "': 'first_step_side'. Using default --> " << step_duration << std::endl;
        }
        if(!walking_parameters["double_stance_duration"])
        {
            std::cout << "Missing in '" << walk_par << "': 'double_stance_duration'. Using default --> " << double_stance_duration << std::endl;
        }
        if(!walking_parameters["max_inclination"])
        {
            std::cout << "Missing in '" << walk_par << "': 'max_inclination'. Using default --> " << max_inclination << std::endl;
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
            std::cout << "Missing in '" << mpc_par << "': 'Q'. Using default --> " << mpc_Q << std::endl;
        }
        if(!mpc_parameters["R"])
        {
            std::cout << "Missing in '" << mpc_par << "':'R'. Using default --> " << mpc_R << std::endl;
        }
        if(!mpc_parameters["horizon_duration"])
        {
            std::cout << "Missing in '" << mpc_par << "':'horizon_duration'. Using default --> " << horizon_duration << std::endl;
        }
        if(!mpc_parameters["zmp_offset"])
        {
            std::cout << "Missing in '" << mpc_par << "':'zmp_offset'. Using default --> " << zmp_offset << std::endl;
        }
    }

}
