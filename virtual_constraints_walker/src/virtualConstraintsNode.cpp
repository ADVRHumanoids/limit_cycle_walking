#include <virtualConstraintsNode.h>
#include <atomic>


#include <boost/geometry.hpp>
#include <boost/polygon/polygon.hpp>
// #include <boost/geometry/geometries/polygon.hpp>
// #include <boost/geometry/geometries/adapted/boost_tuple.hpp>
#include <boost/geometry/geometries/adapted/boost_tuple.hpp>



#include <XmlRpcValue.h>

#define runOnlyOnce ([] { \
    static std::atomic<bool> first_time(true); \
    return first_time.exchange(false); } ())
    
 
    
virtualConstraintsNode::virtualConstraintsNode()
    {
//         initialize_cmd_fake_q1(); //TODO
        
        std::string this_node_name = ros::this_node::getName();
        _logger = XBot::MatLogger::getLogger("/tmp/" + this_node_name);
        ros::NodeHandle n;
        
        _step_counter = 0;
        get_param_ros(); //initial parameters from ros
        
        _initial_pose = _current_pose_ROS;
        _initial_step_y = _current_pose_ROS.get_sole(robot_interface::Side::Left).coeff(1);
        

        _poly_com.set_com_initial_pose(_current_pose_ROS.get_com());
        
        _q1_state = sense_q1();
        
        _terrain_heigth =  _current_pose_ROS.get_sole(_current_side).coeff(2);
        
        
//      prepare subscriber node
        _q1_sub = n.subscribe("/q1", 10, &virtualConstraintsNode::q1_callback, this); /*subscribe to /q1 topic*/
        
//      prepare advertiser node
        _com_pub = n.advertise<geometry_msgs::PoseStamped>("/cartesian/com/reference", 10); /*publish to /cartesian/com/reference*/
        
        _sole_pubs[robot_interface::Side::Left] = n.advertise<geometry_msgs::PoseStamped>("/cartesian/l_sole/reference", 10); /*publish to /l_sole/reference*/
        _sole_pubs[robot_interface::Side::Right] = n.advertise<geometry_msgs::PoseStamped>("/cartesian/r_sole/reference", 10); /*publish to /r_sole/reference*/
        

        
  
    }

bool virtualConstraintsNode::get_param_ros()
    {
        ros::NodeHandle nh_priv("~");
        int max_steps, delay_impact_scenario;
        double clearance_heigth, duration, drop, indentation_zmp, double_stance, start_time, lean_forward, slope_delay_impact, max_inclination, mpc_Q, mpc_R, lat_step, threshold_delay;
        std::vector<double> thresholds_impact_right(2), thresholds_impact_left(2);
        
        bool real_impacts, walking_forward; 
        
        std::string first_side;

        /*default parameters*/
        double default_drop = -0.12;
        double default_clearance_heigth = 0.1;
        double default_duration = 2;
        std::string default_first_side = "Left";
        int default_max_steps = 10;
        double default_indentation_zmp = 0;
        double default_double_stance = 0;
        double default_start_time = 1;
        double default_lean_forward = 0;
        std::vector<double> default_thresholds_impact_right = {50, 100};
        std::vector<double> default_thresholds_impact_left = {50, 100};
        bool default_real_impacts = 0;
        double default_slope_delay_impact;
        bool default_walking_forward = 0;
        double default_max_inclination = 0.1;
        double default_mpc_Q = 1000000;
        double default_mpc_R = 1;
        double default_lat_step = 0;
        double default_threshold_delay = 0;
        int default_delay_impact_scenario = 0;
        
        drop = nh_priv.param("initial_crouch", default_drop);
        max_steps = nh_priv.param("max_steps", default_max_steps);
        duration = nh_priv.param("duration_step", default_duration);
        clearance_heigth = nh_priv.param("clearance_step", default_clearance_heigth);
        first_side = nh_priv.param("first_step_side", default_first_side);
        indentation_zmp = nh_priv.param("indent_zmp", default_indentation_zmp);
        double_stance = nh_priv.param("double_stance_duration", default_double_stance);
        start_time = nh_priv.param("start_time", default_start_time);
        lean_forward = nh_priv.param("lean_forward", default_lean_forward);
        thresholds_impact_right = nh_priv.param("force_sensor_threshold_right", default_thresholds_impact_right);
        thresholds_impact_left = nh_priv.param("force_sensor_threshold_left", default_thresholds_impact_left);
        real_impacts = nh_priv.param("real_impacts", default_real_impacts);
        slope_delay_impact = nh_priv.param("slope_delay_impact", default_slope_delay_impact); // 0 is max Steepness
        walking_forward = nh_priv.param("walking_forward", default_walking_forward); 
        max_inclination = nh_priv.param("max_inclination", default_max_inclination); 
        mpc_Q = nh_priv.param("mpc_Q", default_mpc_Q); 
        mpc_R = nh_priv.param("mpc_R", default_mpc_R); 
        lat_step = nh_priv.param("lateral_step", default_lat_step); 
        threshold_delay = nh_priv.param("threshold_delay", default_threshold_delay);
        delay_impact_scenario = nh_priv.param("delay_impact_scenario", default_delay_impact_scenario);
        
        _initial_param.set_crouch(drop);
        _initial_param.set_max_steps(max_steps);
        _initial_param.set_duration_step(duration);
        _initial_param.set_clearance_step(clearance_heigth);
        _initial_param.set_indent_zmp(indentation_zmp);
        _initial_param.set_double_stance(double_stance);
        _initial_param.set_start_time(start_time);
        _initial_param.set_lean_forward(lean_forward);
        _initial_param.set_threshold_impact_right(thresholds_impact_right);
        _initial_param.set_threshold_impact_left(thresholds_impact_left);
        _initial_param.set_switch_real_impact(real_impacts);
        _initial_param.set_slope_delay_impact(slope_delay_impact);
        _initial_param.set_walking_forward(walking_forward);
        _initial_param.set_max_inclination(max_inclination);
        _initial_param.set_MPC_Q(mpc_Q);
        _initial_param.set_MPC_R(mpc_R);
        _initial_param.set_lateral_step(lat_step);
        _initial_param.set_threshold_delay(threshold_delay);
        _initial_param.set_delay_impact_scenario(delay_impact_scenario);
        
        if (first_side == "Left")
                _initial_param.set_first_step_side(robot_interface::Side::Left);
        else if (first_side == "Right")
                _initial_param.set_first_step_side(robot_interface::Side::Right);
        else std::cout << "unknown side starting command" << std::endl;
    }
            
// double virtualConstraintsNode::getTime()
//     {
//         double time;
//         if (_starting_time == 0)
//         {
//             time = 0;
//         }
//         else
//         {
//             time = ros::Time::now().toSec() - _starting_time;
//         }
//         
//         return time;
//     }

Eigen::Vector3d virtualConstraintsNode::straighten_up_goal()
    {      
        Eigen::Vector3d straight_com = _current_pose_ROS.get_com();
        
        straight_com(0) = _current_pose_ROS.get_sole(_current_side).coeff(0) + _initial_param.get_lean_forward(); /*TODO*/
//         straight_com(1) = _current_pose_ROS.get_sole(_current_side).coeff(1);       /*TODO TOCHANGE*/ /* - _current_pose_ROS.get_sole(_current_side).coeff(1)/3.2 */
        straight_com(2) = _initial_param.get_crouch();
        /*TODO PUT DEFAULT POSITION*/
//         _poly_step.set_data_step( _current_pose_ROS.get_sole(_current_side), _current_pose_ROS.get_sole(_current_side), straight_com, straight_com, 0, 0, getTime(), getTime()+2);
        _bezi_step.set_data_step(_current_pose_ROS.get_sole(_current_side), _current_pose_ROS.get_sole(_current_side), 0);
        _poly_com.set_com_initial_pose(straight_com);  
        return straight_com;
    }
    
Eigen::Affine3d virtualConstraintsNode::l_sole_orientation_goal()
    {      
        Eigen::Affine3d sole_o = _current_pose_ROS.get_sole_tot(robot_interface::Side::Left);
        
        Eigen::Quaterniond goal_orientation;
        
        goal_orientation.x() = 0;
        goal_orientation.y() = 0;
        goal_orientation.z() = 0;
        goal_orientation.w() = 1;
        
        sole_o.linear() = goal_orientation.normalized().toRotationMatrix();
        return sole_o;
    }

Eigen::Affine3d virtualConstraintsNode::r_sole_orientation_goal()
    {      
        Eigen::Affine3d sole_o = _current_pose_ROS.get_sole_tot(robot_interface::Side::Right);
        
        Eigen::Quaterniond goal_orientation;
        
        goal_orientation.x() = 0;
        goal_orientation.y() = 0;
        goal_orientation.z() = 0;
        goal_orientation.w() = 1;
        
        sole_o.linear() = goal_orientation.normalized().toRotationMatrix();
        return sole_o;
    }
    
int virtualConstraintsNode::straighten_up_action() /*if I just setted a publisher it would be harder to define when the action was completed*/
    {
        actionlib::SimpleActionClient<cartesian_interface::ReachPoseAction> ac_com("cartesian/com/reach", true); /*without /goal!!*/
        actionlib::SimpleActionClient<cartesian_interface::ReachPoseAction> ac_r_sole("cartesian/r_sole/reach", true); /*without /goal!!*/
        actionlib::SimpleActionClient<cartesian_interface::ReachPoseAction> ac_l_sole("cartesian/l_sole/reach", true); /*without /goal!!*/
      
        ROS_INFO("Waiting for action server to start.");
        // wait for the action server to start
        ac_com.waitForServer(); //will wait for infinite time
        ac_r_sole.waitForServer();
        ac_l_sole.waitForServer();
        ROS_INFO("Action server started, sending goal.");
        
        float cmd_duration_time;
        cmd_duration_time = 1; //15;
        
        cartesian_interface::ReachPoseGoal goal_com;
        cartesian_interface::ReachPoseGoal goal_l_step, goal_r_step;
        
        // send a goal to the action
        geometry_msgs::Pose cmd_initial_l_sole, cmd_initial_r_sole;
        
        Eigen::Vector4d orientation_sole;
        
        geometry_msgs::Pose q_l;
        geometry_msgs::Pose q_r;
        
        tf::poseEigenToMsg(l_sole_orientation_goal(), q_l);
        tf::poseEigenToMsg(r_sole_orientation_goal(), q_r);
        
        goal_r_step.frames.push_back(q_r); /*wants geometry_msgs::Pose*/
        goal_r_step.time.push_back(cmd_duration_time); 
        
        goal_l_step.frames.push_back(q_l); /*wants geometry_msgs::Pose*/
        goal_l_step.time.push_back(cmd_duration_time);      
        
        ac_r_sole.sendGoal(goal_r_step);
        ac_l_sole.sendGoal(goal_l_step);
        
        geometry_msgs::Pose cmd_initial_com;
        
        tf::pointEigenToMsg(straighten_up_goal(), cmd_initial_com.position);
   
        goal_com.frames.push_back(cmd_initial_com); /*wants geometry_msgs::Pose*/
        goal_com.time.push_back(cmd_duration_time);

        ac_com.sendGoal(goal_com);
    
        //wait for the action to return
        bool finished_before_timeout = ac_com.waitForResult(ros::Duration(5.0));

        if (finished_before_timeout)
        {
            actionlib::SimpleClientGoalState state = ac_com.getState();
            ROS_INFO("Action finished: %s", state.toString().c_str());
        }
        else
            ROS_INFO("Action did not finish before the time out.");

        /* fill initial pose with pose after straighten_up_action */
        _current_pose_ROS.sense();
        _initial_pose = _current_pose_ROS;
        
        std::cout << "initial pose. CoM -> " << _initial_pose.get_com().transpose() << std::endl; 
        std::cout << "Left sole -> " << _initial_pose.get_sole(robot_interface::Side::Left).transpose() << std::endl;
        std::cout << "right sole -> " << _initial_pose.get_sole(robot_interface::Side::Right).transpose() << std::endl;
        
        _initial_q1 = sense_q1();
        std::cout << "First_q1: " << _initial_q1 << std::endl;
        _initial_height = fabs(_current_pose_ROS.get_com().coeff(2) - _current_pose_ROS.get_sole(_current_side).coeff(2)); //TODO
        //exit
        return 0;
}
    
void virtualConstraintsNode::q1_callback(const std_msgs::Float64 msg_rcv) //this is called by ros
    {
       _q1_cmd = msg_rcv.data;
       _check_received = true;
    }

double virtualConstraintsNode::get_q1()
    {       
        return _q1_cmd;
        ROS_INFO("%f", _q1_state);
        
    }
    
Eigen::MatrixXd virtualConstraintsNode::get_supportPolygon() 

    //TODO refactor this a little, add always foot surface + support polygon
    //TODO shitty code
    //TODO porco dio
    {
        _current_pose_ROS.sense();
        double size_foot_x = 0.21;
        double size_foot_y = 0.11;
        std::map<robot_interface::Side, Eigen::Vector3d> sole;
        std::map<robot_interface::Side, Eigen::Vector2d> center_sole;
        typedef boost::geometry::model::point<double, 2, boost::geometry::cs::cartesian> point_t;
        typedef boost::geometry::model::polygon<point_t> polygon;
        std::map<robot_interface::Side, polygon> poly, hull;
        polygon full_poly;
        polygon full_hull;


        sole[robot_interface::Side::Left] = _current_pose_ROS.get_sole(robot_interface::Side::Left);
        sole[robot_interface::Side::Right] = _current_pose_ROS.get_sole(robot_interface::Side::Right);
        
        std::map<robot_interface::Side, Eigen::Matrix<double, 4,2>> vertex_poly;
        
        vertex_poly[robot_interface::Side::Left].row(0) << sole[robot_interface::Side::Left](0) + size_foot_x/2, sole[robot_interface::Side::Left](1) + size_foot_y/2;
        vertex_poly[robot_interface::Side::Left].row(1) << sole[robot_interface::Side::Left](0) + size_foot_x/2, sole[robot_interface::Side::Left](1) - size_foot_y/2;
        vertex_poly[robot_interface::Side::Left].row(2) << sole[robot_interface::Side::Left](0) - size_foot_x/2, sole[robot_interface::Side::Left](1) + size_foot_y/2;
        vertex_poly[robot_interface::Side::Left].row(3) << sole[robot_interface::Side::Left](0) - size_foot_x/2, sole[robot_interface::Side::Left](1) - size_foot_y/2;
        
        vertex_poly[robot_interface::Side::Right].row(0) << sole[robot_interface::Side::Right](0) + size_foot_x/2, sole[robot_interface::Side::Right](1) + size_foot_y/2;
        vertex_poly[robot_interface::Side::Right].row(1) << sole[robot_interface::Side::Right](0) + size_foot_x/2, sole[robot_interface::Side::Right](1) - size_foot_y/2;
        vertex_poly[robot_interface::Side::Right].row(2) << sole[robot_interface::Side::Right](0) - size_foot_x/2, sole[robot_interface::Side::Right](1) + size_foot_y/2;
        vertex_poly[robot_interface::Side::Right].row(3) << sole[robot_interface::Side::Right](0) - size_foot_x/2, sole[robot_interface::Side::Right](1) - size_foot_y/2;
       
        
        center_sole[robot_interface::Side::Left] = _current_pose_ROS.get_sole(robot_interface::Side::Left).head(2);
        center_sole[robot_interface::Side::Right] = _current_pose_ROS.get_sole(robot_interface::Side::Right).head(2);
        
        //generate hull for lfoot
        for (int i = 0; i < 4; i++)
        {
            boost::geometry::append(poly[robot_interface::Side::Left], point_t(vertex_poly[robot_interface::Side::Left].coeff(i,0), vertex_poly[robot_interface::Side::Left].coeff(i,1)));
        }
        boost::geometry::convex_hull(poly[robot_interface::Side::Left], hull[robot_interface::Side::Left]);
        
        //generate hull for rfoot
        for (int i = 0; i < 4; i++)
        {
            boost::geometry::append(poly[robot_interface::Side::Right], point_t(vertex_poly[robot_interface::Side::Right].coeff(i,0), vertex_poly[robot_interface::Side::Right].coeff(i,1)));
        }
        boost::geometry::convex_hull(poly[robot_interface::Side::Right], hull[robot_interface::Side::Right]);    
        
   
        //generate hull for double support
        if (_current_side == robot_interface::Side::Double)
        {
            Eigen::Matrix<double,8,2> temp_poly;
            temp_poly << vertex_poly[robot_interface::Side::Left], vertex_poly[robot_interface::Side::Right];
            
            for (int i = 0; i < 8; i++)
            {
                boost::geometry::append(full_poly, point_t(temp_poly(i,0), temp_poly(i,1)));
            }
            boost::geometry::convex_hull(full_poly, full_hull);
        }
        else
        {
            full_hull = hull[_current_side];
        }
        
        
        /*---------------------------------------------------------------------------*/

        
        
        Eigen::MatrixXd full_hull_point(boost::geometry::num_points(full_hull),2);
        
        Eigen::MatrixXd left_hull_point(boost::geometry::num_points(hull[robot_interface::Side::Left]),2);
        Eigen::MatrixXd right_hull_point(boost::geometry::num_points(hull[robot_interface::Side::Left]),2);
        
      
        int full_vertex_hull_count = 0;
        for(auto it = boost::begin(boost::geometry::exterior_ring(full_hull)); it != boost::end(boost::geometry::exterior_ring(full_hull)); ++it)
    {
        double x = boost::geometry::get<0>(*it);
        double y = boost::geometry::get<1>(*it);
        full_hull_point.row(full_vertex_hull_count) << x,y;
        full_vertex_hull_count++;
    }
        int left_vertex_hull_count = 0;
        for(auto it = boost::begin(boost::geometry::exterior_ring(hull[robot_interface::Side::Left])); it != boost::end(boost::geometry::exterior_ring(hull[robot_interface::Side::Left])); ++it)
    {
        double x = boost::geometry::get<0>(*it);
        double y = boost::geometry::get<1>(*it);
        left_hull_point.row(left_vertex_hull_count) << x,y;
        left_vertex_hull_count++;
    }
        int right_vertex_hull_count = 0;
        for(auto it = boost::begin(boost::geometry::exterior_ring(hull[robot_interface::Side::Right])); it != boost::end(boost::geometry::exterior_ring(hull[robot_interface::Side::Right])); ++it)
    {
        double x = boost::geometry::get<0>(*it);
        double y = boost::geometry::get<1>(*it);
        right_hull_point.row(right_vertex_hull_count) << x,y;
        right_vertex_hull_count++;
    }
//         _logger->add("full_hull", full_hull_point); /*TODO this is wrong, changing size*/
        _logger->add("left_hull", left_hull_point);
        _logger->add("right_hull", right_hull_point);
        
        _logger->add("L_sole_center", center_sole[robot_interface::Side::Left]);
        _logger->add("R_sole_center", center_sole[robot_interface::Side::Right]);
        

        
        Eigen::VectorXd sp(Eigen::Map<Eigen::VectorXd>(full_hull_point.data(), full_hull_point.size()));
        
        Eigen::VectorXd vsp(20);
        vsp.setZero();
        vsp << sp;
        
        _logger->add("support_polygon", vsp);
        
        return full_hull_point;
    }

double virtualConstraintsNode::sense_qlat()
    {
        double q1_lat, q2_lat;
        _current_pose_ROS.sense(); 
        if (_current_side == robot_interface::Side::Double)
        {
            Eigen::Vector3d left_ankle_to_com, right_ankle_to_com;
            
            left_ankle_to_com = _current_pose_ROS.get_distance_ankle_to_com(robot_interface::Side::Left);
            right_ankle_to_com = _current_pose_ROS.get_distance_ankle_to_com(robot_interface::Side::Right);
            
            q1_lat = atan(left_ankle_to_com(1)/left_ankle_to_com(2));
            q2_lat = atan(right_ankle_to_com(1)/right_ankle_to_com(2));
        }
        else
        {
            Eigen::Vector3d swing_ankle_to_com, stance_ankle_to_com;
            robot_interface::Side other_side = (robot_interface::Side)(1 - static_cast<int>(_current_side));
            
            swing_ankle_to_com = _current_pose_ROS.get_distance_ankle_to_com(_current_side);
            stance_ankle_to_com = _current_pose_ROS.get_distance_ankle_to_com(other_side);
            
            q1_lat = atan(stance_ankle_to_com(1)/stance_ankle_to_com(2));
            q2_lat = atan(swing_ankle_to_com(1)/swing_ankle_to_com(2));
        }
        
        _logger->add("q_lateral_stance", q1_lat);
        _logger->add("q_lateral_swing", q2_lat);
        
        
        _logger->add("q_lateral_left", _current_pose_ROS.get_distance_ankle_to_com(robot_interface::Side::Left).coeff(1)/_current_pose_ROS.get_distance_ankle_to_com(robot_interface::Side::Left).coeff(2));
        _logger->add("q_lateral_right", _current_pose_ROS.get_distance_ankle_to_com(robot_interface::Side::Right).coeff(1)/_current_pose_ROS.get_distance_ankle_to_com(robot_interface::Side::Right).coeff(2));
        
        return q1_lat;
    }


double virtualConstraintsNode::sense_q1()
    {   
        _current_pose_ROS.sense();
        double q1, q2;
        Eigen::Vector3d swing_ankle_to_com, stance_ankle_to_com;
        
        if (_current_side == robot_interface::Side::Double)
        {
            Eigen::Vector3d left_ankle_to_com, right_ankle_to_com;
            
            left_ankle_to_com = _current_pose_ROS.get_distance_ankle_to_com(robot_interface::Side::Left);
            right_ankle_to_com = _current_pose_ROS.get_distance_ankle_to_com(robot_interface::Side::Right);
              
        q1 = atan(left_ankle_to_com(0)/left_ankle_to_com(2));
        q2 = atan(right_ankle_to_com(0)/right_ankle_to_com(2));
        
        }
        else
        {
        Eigen::Vector3d swing_ankle_to_com, stance_ankle_to_com;
        
        robot_interface::Side other_side = (robot_interface::Side)(1 - static_cast<int>(_current_side));
        swing_ankle_to_com = _current_pose_ROS.get_distance_ankle_to_com(_current_side);
        stance_ankle_to_com = _current_pose_ROS.get_distance_ankle_to_com(other_side);
            
        q1 = atan(stance_ankle_to_com(0)/stance_ankle_to_com(2));
        q2 = atan(swing_ankle_to_com(0)/swing_ankle_to_com(2));
        
        }
        
        _logger->add("left_foot_to_com", _current_pose_ROS.get_distance_ankle_to_com(robot_interface::Side::Left).coeff(2));
        _logger->add("right_foot_to_com", _current_pose_ROS.get_distance_ankle_to_com(robot_interface::Side::Right).coeff(2));
        
        _logger->add("q_sagittal_stance", q1); //TODO it's not sagittal in the double stance
        _logger->add("q_sagittal_swing", q2);
        
        _logger->add("q_sagittal_left", _current_pose_ROS.get_distance_ankle_to_com(robot_interface::Side::Left).coeff(0)/_current_pose_ROS.get_distance_ankle_to_com(robot_interface::Side::Left).coeff(2));
        _logger->add("q_sagittal_right", _current_pose_ROS.get_distance_ankle_to_com(robot_interface::Side::Right).coeff(0)/_current_pose_ROS.get_distance_ankle_to_com(robot_interface::Side::Right).coeff(2));
   
        return q1;
    }  

void virtualConstraintsNode::update_pose(Eigen::Vector3d *current_pose, Eigen::Vector3d update) 
    {   
//       // update current_pose with update
        *current_pose = *current_pose + update;
    }
    
// Eigen::Vector3d virtualConstraintsNode::calc_com(double q1)
//     {
//         Eigen::Vector3d com_to_ankle_distance;
//         Eigen::Vector3d delta_com;
//         
//         com_to_ankle_distance = _current_pose_ROS.get_distance_ankle_to_com(_current_side); /*if take out minus the robot steps back*/
//         delta_com << - 2* com_to_ankle_distance.z() * tan(q1), 0, 0; /*calc x com distance from given angle q1*/
//         
//           if (_step_counter == 0)
//             delta_com  << (- com_to_ankle_distance.z() * tan(q1)), 0, 0; //+ _current_pose_ROS.get_com().coeff(0), 0, 0; /*calc x com distance from given angle q1*/
//  
//         _logger->add("delta_com", delta_com);
//         return delta_com;
//     }

    
// Eigen::Vector3d virtualConstraintsNode::calc_step(double q1)
//     {
//         Eigen::Vector3d com_to_ankle_distance;
//         Eigen::Vector3d delta_step;
//         
//         com_to_ankle_distance = _current_pose_ROS.get_distance_ankle_to_com(_current_side); /*if take out minus the robot steps back*/
//         delta_step << - 4* com_to_ankle_distance.z() * tan(q1), 0, 0; /*calc step distance given q1*/  //lenght_leg * sin(q1)
//             
//             
//         if (_step_counter == 0)
//             delta_step << (- 2* com_to_ankle_distance.z() * tan(q1)), 0, 0;
// 
//         _logger->add("delta_step", delta_step);
//         return delta_step;
//     }

void virtualConstraintsNode::left_sole_phase()
{
    Eigen::Matrix<double, 6 ,1> ft_left = _current_pose_ROS.get_ft_sole(robot_interface::Side::Left);
  
    switch (_current_phase_left) {
        case Phase::LAND :
            if (fabs(ft_left.coeff(2)) <= _initial_param.get_threshold_impact_left().at(0)) //threshold_min
            {
                _previous_phase_left = _current_phase_left;
                _current_phase_left = Phase::FLIGHT;
            }
            break;
        case Phase::FLIGHT : 
            if (fabs(ft_left.coeff(2)) >= _initial_param.get_threshold_impact_left().at(1)) //threshold_max
            {
                _previous_phase_left = _current_phase_left;
                _current_phase_left = Phase::LAND;
            }
            break;
        default : 
            throw std::runtime_error(std::string("Phase not recognized (left sole)!"));
            break;
    }
}

void virtualConstraintsNode::right_sole_phase()
{
    Eigen::Matrix<double, 6 ,1> ft_right = _current_pose_ROS.get_ft_sole(robot_interface::Side::Right);

    switch (_current_phase_right) {
        case Phase::LAND :
            if (fabs(ft_right.coeff(2)) <= _initial_param.get_threshold_impact_right().at(0)) //threshold_min
            {
                _previous_phase_right = _current_phase_right;
                _current_phase_right = Phase::FLIGHT;
                
            }
            break;
        case Phase::FLIGHT : 
            if (fabs(ft_right.coeff(2)) >= _initial_param.get_threshold_impact_right().at(1)) //threshold_max
            {
                _previous_phase_right = _current_phase_right;
                _current_phase_right = Phase::LAND;
            }
            break;
        default : 
            throw std::runtime_error(std::string("Phase not recognized (right sole)!"));
            break;
    }
}

bool virtualConstraintsNode::real_impacts()
{       
        right_sole_phase();
        left_sole_phase();
        
//         std::cout << "previous left phase: "<<_previous_phase_left << " and current left phase: " << _current_phase_left << std::endl;
//         std::cout << "previous right phase: "<<_previous_phase_right << " and current right phase: " << _current_phase_right << std::endl;
        
        // left foot
        if (_previous_phase_left == Phase::FLIGHT &&  _current_phase_left == Phase::LAND)
        {
            std::cout << "LEFT impact detected" << std::endl;
            _previous_phase_left = _current_phase_left;
            return true;
        }
        
        // right foot
        if (_previous_phase_right == Phase::FLIGHT &&  _current_phase_right == Phase::LAND)
        {
            std::cout << "RIGHT impact detected" << std::endl;
            _previous_phase_right = _current_phase_right;
            return true;
        }
        
        
        return false;
}

bool virtualConstraintsNode::fake_impacts()
{
    double cond;
    
    if (_initial_param.get_walking_forward())
    {
        cond = fabs(_current_pose_ROS.get_sole(_current_side).coeff(0) - _initial_pose.get_sole(_current_side).coeff(0)) >  0.05;
    }
    else
    {
        cond = _internal_time > (_start_walk + 0.2)  && _impact_cond > 0.2;
    }
    
    if (fabs(fabs(_current_pose_ROS.get_sole(_current_side).coeff(2)) - fabs(_terrain_heigth)) <= 1e-4  && cond)
    {
        return true;
    }
    
    return false;
}

bool virtualConstraintsNode::yet_another_impact()
{
    if (    fabs(fabs(_current_pose_ROS.get_sole(_current_side).coeff(2)) - fabs(_terrain_heigth)) <= 1e-4
            && _internal_time > (_start_walk)
            && _impact_cond > _initial_param.get_duration_step()
       )
    {
        return true;
    }
    return false;
}

bool virtualConstraintsNode::impact_detector()
{
    if (_initial_param.get_switch_real_impact())
    {
        real_impacts();
    }
    else
    {
        fake_impacts();
//         yet_another_impact();
    }
}

int virtualConstraintsNode::impact_routine()                
    {
            
            if (impact_detector())
            {
                _event = Event::IMPACT; // event impact detected for core()
                _current_pose_ROS.sense();
                
//                 get_supportPolygon();
                

                robot_interface::Side last_side = _current_side;
//                 std::cout << "Last side: " << last_side << std::endl;
               _initial_pose = _current_pose_ROS;
                _current_side = robot_interface::Side::Double;
                
//                 get_supportPolygon();
                
                
                
                std::cout << "Impact! Current side: " << _current_side << std::endl;
                
                _current_pose_ROS.get_sole(_current_side);
                // ----------------------------------------------------
                _poly_com.set_com_initial_pose(_current_pose_ROS.get_com());          //ADDED
                // ----------------------------------------------------
                

//                 {
                    if (last_side == robot_interface::Side::Left)
                        _current_side = robot_interface::Side::Right;
                    else if (last_side == robot_interface::Side::Right)
                        _current_side = robot_interface::Side::Left;
                    else ROS_INFO("wrong side");
                    
                    std::cout << "State changed. Current side: " << _current_side << std::endl;
                
//                     std::cout << "Return 1" << std::endl;
                    return 1;
//                 }
            }
            else
            {
                return 0;
            }
    }

bool virtualConstraintsNode::new_q1()
    {
        bool flag_q1 = false;
        double last_q1_step = 0;
        last_q1_step = _q1_step;
        _q1_step = get_q1();
        
        if (fabs(last_q1_step - _q1_step) >= 1e-10) /* TODO this is very sad*/ /*it's a problem of initialization*/
        {
            flag_q1 = true;
        }

        return flag_q1;
    }
    
void virtualConstraintsNode::first_q1()
    {
        ROS_INFO("waiting for command...");
        std::cout << "Initial state: " << _current_side << std::endl;
        while (!_check_received)
        {
            _current_pose_ROS.sense();
            
            if (_check_received)
            {
                _q1_state = _q1_cmd;
            }
                
        }
        ROS_INFO("command received! q1 = %f", _q1_state);
    }
    
    
void virtualConstraintsNode::exe(double time)
{   
    if (_init_completed == 0)
        ST_init(time);
    
    
    _internal_time = time - _starting_time;
        
        
    _impact_cond = _internal_time - _reset_time;

     
    if (_internal_time >= _start_walk && runOnlyOnce)
    {
        _event = Event::START;
    };
    
    if (_cycleCounter == 2 && runOnlyOnce)
    {
//         _impact_cond = 0;
        _event = Event::STOP;
    };
        

        if (impact_routine())
        {    
            _reset_time = _internal_time;
        }
        
//         std::cout << "com sensed now: " << _current_pose_ROS.get_com().transpose() << std::endl;
        core(_internal_time);
        commander(_internal_time);
}

void virtualConstraintsNode::send(std::string type, Eigen::Vector3d command)
    {
        geometry_msgs::PoseStamped cmd;
        tf::pointEigenToMsg(command, cmd.pose.position);
        
        if (type == "right")
        {
            _sole_pubs[robot_interface::Side::Right].publish(cmd);
        }
        else if (type == "left")
        {
            _sole_pubs[robot_interface::Side::Left].publish(cmd);
        }
        else if (type == "com")
        {
            _com_pub.publish(cmd);  /*TODO make a map here, so there is only one publisher and you decide to whom*/
        }
        else std::cout << "you're trying to send the command to a non existent task" << std::endl;
    }

void virtualConstraintsNode::send_com(Eigen::Vector3d com_command)
    {
        geometry_msgs::PoseStamped cmd_com;
        tf::pointEigenToMsg(com_command, cmd_com.pose.position);
        
        _com_pub.publish(cmd_com);
    }
void virtualConstraintsNode::send_step(Eigen::Vector3d foot_command)
    {
        geometry_msgs::PoseStamped cmd_sole;
        tf::pointEigenToMsg(foot_command, cmd_sole.pose.position);
        
        _sole_pubs[_current_side].publish(cmd_sole);
    }

Eigen::Vector3d virtualConstraintsNode::compute_swing_trajectory(const Eigen::Vector3d& start, 
                                                                 const Eigen::Vector3d& end,
                                                                 double clearance,
                                                                 double t_start, 
                                                                 double t_end, 
                                                                 double time, 
                                                                 std::string plane,
                                                                 Eigen::Vector3d* vel,
                                                                 Eigen::Vector3d* acc
                                                                )
{
    Eigen::Vector3d ret;

    
    double dx0 = 0;
    double ddx0 = 0;
    
    double dxf = 0;
    double ddxf = 0;
    
    double dx; 
    double ddx;
    
    double beta = 1; //2
    double tau = std::min(std::max((time - t_start)/(t_end - t_start), 0.0), 1.0);
    double alpha = compute_swing_trajectory_normalized_plane(dx0, ddx0, dxf, ddxf, time_warp(tau, beta), &dx, &ddx);
    
    if (plane == "xy" || plane == "yx")
    {
        ret.head<2>() = (1-alpha)*start.head<2>() + alpha*end.head<2>();
        ret.z() = start.z() + compute_swing_trajectory_normalized_clearing(end.z()/clearance, time_warp(tau, beta))*clearance; /*porcodio*/
     }
     else if (plane == "yz" || plane == "zy")
     {
         ret.tail<2>() = (1-alpha)*start.tail<2>() + alpha*end.tail<2>();
         ret.x() = start.x() + compute_swing_trajectory_normalized_clearing(end.x()/clearance, time_warp(tau, beta))*clearance; /*porcodio*/
     }
     else if (plane == "xz" || plane == "zx")
     {
        ret[0] = (1-alpha)*start[0] + alpha*end[0];
        ret[2] = (1-alpha)*start[2] + alpha*end[2];
        ret.y() = start.y() + compute_swing_trajectory_normalized_clearing(end.y()/clearance, time_warp(tau, beta))*clearance; /*porcodio*/
     }
     
     _logger->add("alpha", alpha);
     _logger->add("traj_pos", ret);
     _logger->add("traj_vel", dx);
     _logger->add("traj_acc", ddx);
    return ret;
}

double virtualConstraintsNode::compute_swing_trajectory_normalized_plane(double dx0, double ddx0, 
                                                                         double dxf, double ddxf, 
                                                                         double tau, 
                                                                         double* __dx, double* __ddx)
{
    
    double x, dx, ddx;
    FifthOrderPlanning(0, dx0, ddx0, 1, dxf, ddxf, 0, 1, tau, x, dx, ddx);

    if(__dx) *__dx = dx;
    if(__ddx) *__ddx = ddx; 
    
    return x;
}

double virtualConstraintsNode::compute_swing_trajectory_normalized_clearing(double final_height, double tau, double* dx, double* ddx)
{
    
    
    double x = std::pow(tau, 3)*std::pow(1-tau, 3);
    double x_max = 1./64.;
    x = x/x_max;
    
//     if(dx) *dx = powdx.dot(avec);
//     if(ddx) *ddx = powddx.dot(avec);
    
    return x;
    
}

double virtualConstraintsNode::time_warp(double tau, double beta)
{
    return 1.0 - std::pow(1.0 - tau, beta);
}

void virtualConstraintsNode::FifthOrderPlanning(double x0, double dx0, double ddx0,  //initial position
                                                double xf, double dxf, double ddxf,  //final position
                                                double start_time, double end_time, 
                                                double time, double& x, double& dx, double& ddx 
                                                )
{
    Eigen::Matrix6d A;
    A << 1.0000,         0,         0,         0,         0,         0,
              0,    1.0000,         0,         0,         0,         0,
              0,         0,    0.5000,         0,         0,         0,
       -10.0000,   -6.0000,   -1.5000,   10.0000,   -4.0000,    0.5000,
        15.0000,    8.0000,    1.5000,  -15.0000,    7.0000,   -1.0000,
        -6.0000,   -3.0000,   -0.5000,    6.0000,   -3.0000,    0.5000;
    
    double alpha = (end_time-start_time);
    alpha = std::max(1e-6, alpha);
    double tau = (time - start_time)/alpha; // d/dt = d/d(alpha*tau)
    tau = std::max(0.0, tau);
    tau = std::min(tau, 1.0);
        
    Eigen::Vector6d b;
    b << x0, dx0*alpha, ddx0*std::pow(alpha,2.0), xf, dxf*alpha, ddxf*std::pow(alpha,2.0);
    
    Eigen::Vector6d coeffs = A*b;
    
    Eigen::Vector6d t_v, dt_v, ddt_v;
    for(int i = 0; i < 6; i++)
    {
        t_v(i) = std::pow(tau, i);
        dt_v(i) = i > 0 ? std::pow(tau, i-1)*i : 0;
        ddt_v(i) = i > 1 ? std::pow(tau, i-2)*i*(i-1) : 0;
        
    }
    
    x = coeffs.dot(t_v);
    dx = coeffs.dot(dt_v)/alpha;
    ddx = coeffs.dot(ddt_v)/(alpha*alpha);
}

double virtualConstraintsNode::getPt( double n1 , double n2 , double perc )
{
    double diff = n2 - n1;

    return n1 + ( diff * perc );
} 

double virtualConstraintsNode::getBezierCurve(Eigen::VectorXd coeff_vec, double tau)
{ 
    double x = 0;
    int n_points  = coeff_vec.size();
    Eigen::VectorXd clone_vec = coeff_vec;

    
    for (int temp_n_points = (n_points-1); temp_n_points >= 1; temp_n_points--)
    {
        for (int i = 0; i < temp_n_points; i++)
        {
            clone_vec(i) = getPt(clone_vec.coeff(i), clone_vec.coeff(i+1), tau);
        }
    }

    x = clone_vec.coeff(0);
    _logger->add("traj_bezier_x", x);
    
    return x;
    
    
}

double virtualConstraintsNode::getBezierCurve(Eigen::VectorXd coeff_vec, Eigen::VectorXd coeff_vec_t, double tau)
{ 
    double t = 0;
    double p = 0;
    
    int n_points  = coeff_vec.size();
    
    Eigen::VectorXd clone_vec = coeff_vec;
    Eigen::VectorXd clone_vec_t = coeff_vec_t;
    
    
    
    for (int temp_n_points = (n_points-1); temp_n_points >= 1; temp_n_points--)
    {
        for (int i = 0; i < temp_n_points; i++)
        {
            clone_vec(i) = getPt(clone_vec.coeff(i), clone_vec.coeff(i+1), tau);
            clone_vec_t(i) = getPt(clone_vec_t.coeff(i), clone_vec_t.coeff(i+1), tau);
        }
    }

    p = clone_vec.coeff(0);
    t = clone_vec_t.coeff(0);
    
    
    _logger->add("traj_bezier_p", p);
    _logger->add("traj_bezier_t", t);
   
    _logger->add("tau", tau);
    
    return p;
    
    
}

void virtualConstraintsNode::lSpline(Eigen::VectorXd times, Eigen::VectorXd y, double dt, Eigen::VectorXd &times_vec, Eigen::VectorXd &Y)
{

    int n = times.size();
    double N = 0;
    
    
    Eigen::VectorXi N_chunks(n-1);
    
    for(int i=0; i<n-1; i++)
    { 
        N = N + (times.coeff(i+1)-times.coeff(i))/dt;
        N_chunks(i) = round((times.coeff(i+1)-times.coeff(i))/dt);   ; //round((times.coeff(i+1)-times.coeff(i))/dt);      
    }
    
//     std::cout << "N: " << N << std::endl;
//     std::cout << "N_progress: " << N_chunks.transpose() << std::endl;

    Y.resize(N+1,1);
    Y.setZero();
    
    times_vec.setLinSpaced(N, dt, dt*N);

//     std::cout << "times_vec_init" << times_vec(0) << std::endl;
//     std::cout << "times_vec: " << times_vec.transpose() << std::endl;
     
    int idx = 0;
    for(int i=0; i<n-1; i++)
    {
        Eigen::VectorXd temp_vec(N_chunks(i)+1);
        temp_vec.setLinSpaced(N_chunks(i)+1, y.coeff(i), y.coeff(i+1));

//         std::cout << "temp_vec.size(): " << temp_vec.size()-1 << std::endl;
        
        Y.segment(idx, temp_vec.size()-1) = temp_vec.segment(1,temp_vec.size()-1);
        idx += (temp_vec.size()-1);
        
//         std::cout << "idx: " << idx << std::endl;
    }

//     std::cout << "Y: " << Y.transpose() << std::endl;
    for (int i = 0; i < times_vec.size(); i++)
    {
        _logger->add("comy_traj_planned", times_vec(i));
        _logger->add("times_traj_planned", Y(i));
    }
}

void virtualConstraintsNode::generate_zmp(double y_start, double t_start, double double_stance, int num_points, double dt, Eigen::VectorXd& zmp_t, Eigen::VectorXd& zmp_y)
{
//         int num_points = _initial_param.get_max_steps();
        double t_end = t_start + _step_duration*num_points;
        //TODO qui potrei mettere anche un t_windows_end
        Eigen::VectorXd y, times;
        
        y.resize(num_points*2,1);
        times.resize(num_points*2,1);
            
        int myswitch = 1;
        int i = 0;
        int j = 0;
        for (i = 0; i<num_points; i++)
        {
            times(j) = t_start + _step_duration* i + double_stance*(i+1);    
            times(j+1) = t_start + _step_duration* (i+1) + double_stance*(i+1);
            
            y(j) = myswitch * y_start;
            y(j+1) = myswitch * y_start;
            myswitch = -1 * myswitch;
            j = j+2;
        }
        
        
        Eigen::VectorXd y_tot(y.size() + 3);
        Eigen::VectorXd times_tot(times.size() + 3);
        
        y_tot << 0, 0, y, 0;
        times_tot << 0, t_start, times, t_end + double_stance*(i+1);

//         std::cout << "y_tot: " << y_tot.transpose() << std::endl;
//         std::cout << "times_tot: " << times_tot.transpose() << std::endl;
        
        lSpline(times_tot, y_tot, dt, zmp_t, zmp_y);
    }

void virtualConstraintsNode::zmp_window(Eigen::VectorXd zmp_t, Eigen::VectorXd zmp_y, double window_start, double window_end, Eigen::VectorXd &zmp_window_t, Eigen::VectorXd &zmp_window_y)
    {

        
        int window_size = round((window_end - window_start))/_MpC_lat->_Ts + 1;

        zmp_window_t.setLinSpaced(window_size, window_start, window_end);
        
        int i = 0;
        while (i < zmp_t.size() && zmp_t(i) < window_start)
        {
            i++;
        }
//         
        int j = 0;
        while (j < zmp_t.size() && zmp_t(j) < window_end)
        {
            j++;
        }
        
        zmp_window_y.resize(window_size,1);

        zmp_window_y.setZero();
        zmp_window_y.segment(0, j-i) = (zmp_y).segment(i, j-i);

    }
    
Eigen::Vector3d virtualConstraintsNode::lateral_com(double time)
{
        time = time - _start_walk;
        double dt = 0.01; //TODO take it out from here
        
        double entered_forward = 0;
        double reset_lateral = 0;
        
        // this is needed to synchro pre impacts
//         if (_event == Event::IMPACT && _step_counter <= _initial_param.get_max_steps())  // jump in time, going to closer planned impact
//         {
//             entered_forward = 1;  
//             _shift_time = time - _planned_impacts(_step_counter) - dt; //_planned_impacts(ceil((time - _start_walk)/_initial_param.get_duration_step()))
//             
//             _period_delay = 0;
// 
//                 
//         }
//         // -----------------------------------------------------------------------------------------------------------------------------
        
//          _logger->add("shifted", entered_forward);
         
        double window_start = time - _shift_time;
//         
//         _entered_delay = 0;
//         bool entered_period_delay = 0;
//         bool entered_right = 0;
//         bool entered_left = 0;
//         
//         if (time > _planned_impacts(_step_counter) + _shift_time) // if it entered inside delay
//         {
//             
//             _entered_delay = 1;
//             _period_delay = _internal_time - _planned_impacts(_step_counter) + _shift_time; // HOW MUCH TIME IT IS STAYING HERE
//             
//             
//             
//             
//             // // --------------------------------------------------------------
//             // // ---------------SAVE THE FALL ---------------------------------
//             // // --------------------------------------------------------------
//             if (_period_delay >= _initial_param.get_threshold_delay())
//             {
//                 entered_period_delay = 1;
// //                 // if I want  the lateral stepping immediately
//                 if (_current_side == robot_interface::Side::Right)
//                 {
//                     entered_right = 1;
//                     _lateral_step = - _initial_param.get_lateral_step();
//                 }
//                 else if (_current_side == robot_interface::Side::Left)
//                 {
//                     entered_left = 1;
//                     _lateral_step = _initial_param.get_lateral_step();
//                 }                        
//             }
// 
//             if (_event == Event::IMPACT)
//             {
//                 Eigen::Vector3d reset_pose;
//                 reset_pose = _poly_step.get_foot_final_pose();
//                 
//                 if (_current_side == robot_interface::Side::Left)
//                 {
//                     reset_pose(1) = _initial_step_y;
//                 }
//                 else if (_current_side == robot_interface::Side::Right)
//                 {
//                     reset_pose(1) = - _initial_step_y;
//                 }
//                 
//                 _poly_step.set_foot_final_pose(reset_pose);
//                 _lateral_step = 0;
//             }
//             
//             
// //                 std::cout << "UPDATING:" << std::endl;
// //                 std::cout << "final pose: " << _poly_step.get_foot_final_pose().transpose() << std::endl;
// 
//                 Eigen::Vector3d newpose = _poly_step.get_foot_final_pose();
//                 newpose[1] += _lateral_step;
//                 
//                 if (newpose[1] > (_initial_step_y + _lateral_step))
//                 {
//                     newpose[1] = _initial_step_y + _lateral_step;
//                 }
//                 
//                 if (newpose[1] < (- _initial_step_y + _lateral_step))
//                 {
//                     newpose[1] = -_initial_step_y + _lateral_step;
//                 }
//                 
//                 _poly_step.set_foot_final_pose(newpose);
//                 
// //                 std::cout << "NEW final pose: " << _poly_step.get_foot_final_pose().transpose() << std::endl;
// //                 std::cout << std::endl;
//             
// 
//        
//             if (_initial_param.get_delay_impact_scenario() == 0 || _initial_param.get_delay_impact_scenario() == 1) // RAMP or STAY FIXED
//             {
//                 if (_step_counter % 2 == 0)
//                 {
//                             window_start = time - (_planned_impacts(_step_counter) + _shift_time); 
//                             zmp_window(_zmp_t_fake_right, _zmp_y_fake_right, window_start, _MpC_lat->_window_length + window_start, _zmp_window_t, _zmp_window_y);
// //                             _logger->add("window_tot", _zmp_window_y);
//                             if (_period_delay >= _initial_param.get_threshold_delay())
//                             {
//                                 zmp_window(_zmp_t_fake_right_lat, _zmp_y_fake_right_lat, window_start, _MpC_lat->_window_length + window_start, _zmp_window_t, _zmp_window_y);
//                             }
//                 }
//                 else
//                 {
//                             window_start = time - (_planned_impacts(_step_counter) + _shift_time);
//                             zmp_window(_zmp_t_fake_left, _zmp_y_fake_left, window_start, _MpC_lat->_window_length + window_start, _zmp_window_t, _zmp_window_y);
// //                             _logger->add("window_tot", _zmp_window_y);
//                             if (_period_delay >= _initial_param.get_threshold_delay())
//                             {
//                                 zmp_window(_zmp_t_fake_left_lat, _zmp_y_fake_left_lat, window_start, _MpC_lat->_window_length + window_start, _zmp_window_t, _zmp_window_y);
//                             }
//                 }
//             }
// //             else if (_initial_param.get_delay_impact_scenario() == 2) // ??????????????
// //             {
// //                 window_start = time - _period_delay;
// //                 zmp_window(_zmp_t, _zmp_y, window_start, _MpC_lat->_window_length + window_start, _zmp_window_t, _zmp_window_y);
// //             }
// 
//          // MOVE COM LATERALLY WHEN STEPS LATERALLY   
// //             if (_period_delay >= _initial_param.get_threshold_delay()) // this shift a little bit the zmp for the lateral step
// //             {
// //                     window_start = time - _period_delay;
// //                     zmp_window(_zmp_t_lat, _zmp_y_lat, window_start, _MpC_lat->_window_length + window_start, _zmp_window_t, _zmp_window_y);  
// //             }
//             
//         _logger->add("zmp_ref", _zmp_window_y.coeff(0));
//         _logger->add("window_tot", _zmp_window_y);
//         }
//         else // if it's not entered inside delay
//         {
                    zmp_window(_zmp_t, _zmp_y, window_start, _MpC_lat->_window_length + window_start, _zmp_window_t, _zmp_window_y);
//                     _logger->add("zmp_ref", _zmp_window_y.coeff(0));
//                     _logger->add("window_tot", _zmp_window_y);
//         }
            
//         _logger->add("delayed", _entered_delay);
//         _logger->add("period_delay", _period_delay);
//         _logger->add("entered_period_delay", entered_period_delay);
//         _logger->add("entered_left", entered_left);
//         _logger->add("entered_right", entered_right);
//         _logger->add("lateral_step", _lateral_step);
//         _logger->add("current_side", static_cast<int>(_current_side));
//         _logger->add("event", static_cast<int>(_event));
//         _logger->add("reset_lateral", reset_lateral);
        
        _u = _MpC_lat->_K_fb * _com_y + _MpC_lat->_K_prev * _zmp_window_y;
        
        _MpC_lat->_integrator->integrate(_com_y, _u, dt, _com_y);
        
//         _logger->add("com", _com_y);
//         _logger->add("u", _u);
//         _logger->add("zmp", _MpC_lat->_C_zmp*_com_y);
//         _logger->add("zmp_ref", _zmp_window_y.coeff(0));
        
        return _com_y;    
}

void virtualConstraintsNode::commander(double time)
{
    if (_current_state == State::IDLE)
    {
        ////do nothing just logging
        Eigen::Vector3d com_trajectory, foot_trajectory;
        com_trajectory = _current_pose_ROS.get_com();
        foot_trajectory = _current_pose_ROS.get_sole(robot_interface::Side::Left);
        
        _logger->add("com_trajectory", com_trajectory);
        _logger->add("foot_trajectory", foot_trajectory);

        _logger->add("time", _internal_time); // TODO wrong dimension wrt to logger foot and com
        
        _logger->add("ft_left", _current_pose_ROS.get_ft_sole(robot_interface::Side::Left));
        _logger->add("ft_right", _current_pose_ROS.get_ft_sole(robot_interface::Side::Right));

        _logger->add("landed_left", static_cast<int>(_current_phase_left));
        _logger->add("landed_right",  static_cast<int>(_current_phase_right));
               
        _logger->add("zmp_ref", _zmp_window_y.coeff(0));
        _logger->add("window_tot", _zmp_window_y);
        
        _logger->add("com", _com_y);
        _logger->add("u", _u);
        _logger->add("zmp", _MpC_lat->_C_zmp*_com_y);
    }
    else
    {
        //// send com sagittal
        Eigen::Vector3d com_trajectory;
    
        if (_initial_param.get_walking_forward())
        {
            com_trajectory = compute_swing_trajectory(_poly_com.get_com_initial_pose(), _poly_com.get_com_final_pose(), 0, _poly_com.get_starTime(), _poly_com.get_endTime(), time, "xy");
        }
        
        com_trajectory(1) = lateral_com(time).coeff(0);   
        
        std::cout << "time now: " << time << std::endl;
//         std::cout << "initial com send to commander " << _poly_com.get_com_initial_pose().transpose() << std::endl;
//          std::cout << "final com send to commander " << _poly_com.get_com_final_pose().transpose() << std::endl;
         std::cout << "start time " << _poly_com.get_starTime() << std::endl;
         std::cout << "end time " << _poly_com.get_endTime() << std::endl;
//         std::cout << "COMMANDED TRAJ COM " << com_trajectory.transpose() << std::endl;
        
       
        //// send foot
        

        Eigen::Vector3d foot_trajectory;
        
        if (_initial_param.get_walking_forward())
        {
            foot_trajectory = compute_swing_trajectory(_poly_step.get_foot_initial_pose(), _poly_step.get_foot_final_pose(), _poly_step.get_step_clearing(), _poly_step.get_starTime(), _poly_step.get_endTime(), time, "xy");
        }
        else
        {
            Eigen::Vector3d keep_position = _poly_step.get_foot_final_pose();
            
            
            keep_position(0) = _poly_step.get_foot_initial_pose().coeff(0);
//             keep_position(1) = _poly_step.get_foot_initial_pose().coeff(1) + _lateral_step;
            keep_position(2) = _poly_step.get_foot_initial_pose().coeff(2);
             _poly_step.set_foot_final_pose(keep_position);
             
//             std::cout << "initial step: " << _poly_step.get_foot_initial_pose().transpose() << std::endl;
//             std::cout << "final step updated: " << _poly_step.get_foot_final_pose().transpose() << std::endl;
            foot_trajectory = compute_swing_trajectory(_poly_step.get_foot_initial_pose(), _poly_step.get_foot_final_pose(), _poly_step.get_step_clearing(), _poly_step.get_starTime(), _poly_step.get_endTime(), time, "xy");
        }
       
        _logger->add("com_trajectory", com_trajectory);
        _logger->add("foot_trajectory", foot_trajectory);
        
        _logger->add("time", _internal_time); // TODO wrong dimension wrt to logger foot and com
        
        _logger->add("ft_left", _current_pose_ROS.get_ft_sole(robot_interface::Side::Left));
        _logger->add("ft_right", _current_pose_ROS.get_ft_sole(robot_interface::Side::Right));

        _logger->add("landed_left", static_cast<int>(_current_phase_left));
        _logger->add("landed_right",  static_cast<int>(_current_phase_right));
    
        _logger->add("zmp_ref", _zmp_window_y.coeff(0));
        _logger->add("window_tot", _zmp_window_y);
        
        _logger->add("com", _com_y);
        _logger->add("u", _u);
        _logger->add("zmp", _MpC_lat->_C_zmp*_com_y);
        
//         std::cout << "foot_trajectory " << foot_trajectory.transpose() << std::endl;
        send_com(com_trajectory);
        send_step(foot_trajectory);
    }
    
    //burn impact event
    if (_event == Event::IMPACT)
    {
        _event = Event::EMPTY;
        _step_counter++;
    }
}
    
    

bool virtualConstraintsNode::ST_idle(double time)
{
    
    return 1;
};

bool virtualConstraintsNode::ST_halfStep(double time)
{
//     std::cout << "ENTERED HALF STEP PLANNER at time: " << time << std::endl;
    
    _initial_com_position = _current_pose_ROS.get_com();
    _initial_sole_position = _current_pose_ROS.get_sole(_current_side);
    
    _final_sole_position = _initial_sole_position;
    _final_com_position = _initial_com_position;
    
    if (_current_side == robot_interface::Side::Left)
    {
        _final_sole_position[1] = _initial_step_y;
    }
    else if (_current_side == robot_interface::Side::Right)
    {
        _final_sole_position[1] = - _initial_step_y;
    }
   

    Eigen::Vector3d delta_com;
    delta_com << (- _current_pose_ROS.get_distance_ankle_to_com(_current_side).z() * tan(_q_max)), 0, 0;
    _final_com_position += delta_com;

    Eigen::Vector3d delta_step;
    delta_step << (- 2* _current_pose_ROS.get_distance_ankle_to_com(_current_side).z() * tan(_q_max)), 0, 0;
    _final_sole_position += delta_step;

    _poly_step.set_foot_initial_pose(_initial_sole_position);
    _poly_step.set_foot_final_pose(_final_sole_position);
    // ---------------------------------------------
    _poly_step.set_starTime(time);
    _poly_step.set_endTime(time + _initial_param.get_duration_step());
    _poly_step.set_step_clearing(_initial_param.get_clearance_step());
    // ---------------------------------------------
    _poly_com.set_com_initial_pose(_initial_com_position);
    _poly_com.set_com_final_pose(_final_com_position);
    
    _poly_com.set_starTime(time);
    _poly_com.set_endTime(time + _initial_param.get_duration_step());
    
    return 1;
};
       
bool virtualConstraintsNode::ST_fullStep(double time)
{
    
//     std::cout << "ENTERED FULL STEP PLANNER" << std::endl;
    _initial_com_position = _current_pose_ROS.get_com();

     
    _initial_sole_position = _current_pose_ROS.get_sole(_current_side);
    
    _final_sole_position = _initial_sole_position;
    _final_com_position = _initial_com_position;
    
    if (_current_side == robot_interface::Side::Left)
    {
        _final_sole_position[1] = _initial_step_y;
    }
    else if (_current_side == robot_interface::Side::Right)
    {
        _final_sole_position[1] = - _initial_step_y;
    }
    
    Eigen::Vector3d delta_com;
    delta_com << (- 2* _current_pose_ROS.get_distance_ankle_to_com(_current_side).z() * tan(_q_max)), 0, 0; // _lateral_step
    _final_com_position += delta_com;
        
    Eigen::Vector3d delta_step;
    delta_step << (- 4* _current_pose_ROS.get_distance_ankle_to_com(_current_side).z() * tan(_q_max)), 0, 0; //_lateral_step
    _final_sole_position += delta_step;

    _poly_step.set_foot_initial_pose(_initial_sole_position);
    _poly_step.set_foot_final_pose(_final_sole_position);
    // ---------------------------------------------
    _poly_step.set_starTime(time);
    _poly_step.set_endTime(time + _initial_param.get_duration_step());
    _poly_step.set_step_clearing(_initial_param.get_clearance_step());
    // ---------------------------------------------
    _poly_com.set_com_initial_pose(_initial_com_position);
    _poly_com.set_com_final_pose(_final_com_position);
    
    _poly_com.set_starTime(time);
    _poly_com.set_endTime(time + _initial_param.get_duration_step());


    return 1;
};

    
void virtualConstraintsNode::planner(double time) 
{  
    switch (_current_state)
    {
            case State::STARTING :
                ST_halfStep(time);
                break;
        
            case State::WALK :
                ST_fullStep(time);
                break;
        
            case State::STOPPING :
                 ST_halfStep(time);
                 break;
                 
            case State::IDLE :
                ST_idle(time);
                break;
            
            default : throw std::runtime_error(std::string("State not recognized"));
    }
};


void virtualConstraintsNode::core(double time)
{
//     std::cout << "Entering core with event: " << _event << " during state: " << _current_state << std::endl;
    
    
    switch (_event)
    {    
        case Event::IMPACT :
            switch (_current_state)
            {
                
                case State::IDLE :
                    throw std::runtime_error(std::string("Something wrong. Impact during IDLE"));
                    break;
                   
                case State::WALK :
                    planner(time);
                    break;
                    
                case State::STARTING :
                    _current_state = State::WALK;
                    planner(time);
                    break;
                    
                case State::STOPPING :
                    _current_state = State::IDLE;
                    planner(time);
                    break;
            }
            
            _step_counter++;
//             _event = Event::EMPTY; //burn event
            break;
        
        case Event::START :
            switch (_current_state)
            {

                case State::IDLE :
                    _current_state = State::STARTING;
                    planner(time + _t_before_first_step);
                    break;
                    
                default : std::cout << "Ignored starting event. Already WALKING" << std::endl;
                    break;
            }
//             _event = Event::EMPTY; //burn event
            break;
         
        case Event::STOP :
            switch (_current_state)
            {
                case State::IDLE :
                    std::cout << "Ignored stopping event. Already in IDLE" << std::endl;
                    break;
                   
                case State::WALK :
                    _current_state = State::STOPPING;
                    planner(time);
                    break;
                    
                case State::STARTING :
                    _current_state = State::STOPPING;
                    planner(time);
                    break;
                    
                case State::STOPPING :
                    std::cout << "Ignored stopping event. Already STOPPING" << std::endl;
                    break;
            }
//             _event = Event::EMPTY; //burn event
            break;
            
        case Event::EMPTY :
            break;
             
        default : 
            throw std::runtime_error(std::string("Event not recognized"));
            break;
    }
        
}

bool virtualConstraintsNode::ST_init(double time) 
{
    double clearance = _initial_param.get_clearance_step();
    _reset_condition = 0; 
    

    _current_side = _initial_param.get_first_step_side();
    std::cout << "First step: " << _current_side << std::endl;
    _other_side = (robot_interface::Side)(1 - static_cast<int>(_current_side));

    _initial_com_position = _current_pose_ROS.get_com();
    _initial_sole_position = _current_pose_ROS.get_sole(_current_side);
    _final_sole_position = _initial_sole_position;
    _final_com_position = _initial_com_position;

    planner(time);
    Eigen::Vector3d foot_trajectory;
    Eigen::Vector3d fake_pose;
    fake_pose.setZero();
    
    // just to run it once, heat up the process
    foot_trajectory = compute_swing_trajectory(fake_pose, fake_pose, 0, 0, 0, time, "xy");
    /* comy */

    double Ts = 0.01; //window resolution
    double T = 5; //window length for MpC
    double dt = 0.01; //rate of ros
    _t_before_first_step = 0.5; // preparation time in the first step for the com to swing laterally before stepping 
    
    _start_walk = _initial_param.get_start_time(); //TODO _start_walk and _initial_param.get_start_time are the same thing
    _q_min = sense_q1(); // min angle of inclination
    _q_max = _initial_param.get_max_inclination(); // max angle of inclination

    _step_duration = _initial_param.get_duration_step();

    _steep_coeff = _q_max/_step_duration;

    std::cout << "Start walk time: " << _start_walk <<  std::endl;
    std::cout << "Max angle of inclination: " << _q_max <<  std::endl;
    std::cout << "Step duration: " << _step_duration <<  std::endl;
    std::cout << "Double stance: " <<  _initial_param.get_double_stance() << std::endl;
    std::cout << "Steepness: " << _steep_coeff <<  std::endl;
    std::cout << "Real impacts: " << _initial_param.get_switch_real_impact() <<  std::endl;
    
    switch (_initial_param.get_delay_impact_scenario())
    {
        case 0 :
            std::cout << "delay adjustment: RAMP TO 0" << std::endl;
            break;
        case 1 :
            std::cout << "delay adjustment: KEEP CONSTANT ZMP" << std::endl;
            break;
        case 2 :
            std::cout << "delay adjustment: DON'T UPDATE, STOP WINDOW (does not work well in theory)" << std::endl;
            break;
        default : 
            std::cout << "delay adjustment: WRONG scenario" << std::endl;
            break;
    }
    _com_y << _initial_com_position(1), 0, 0; //com trajectory used by mpc: pos, vel, acc

    int sign_first_stance_step = (_current_pose_ROS.get_sole(_other_side).coeff(1) > 0) - (_current_pose_ROS.get_sole(_other_side).coeff(1) < 0);

    double first_stance_step = _current_pose_ROS.get_sole(_other_side).coeff(1) - sign_first_stance_step * _initial_param.get_indent_zmp();


    // generate zmp given start walk and first stance step
    generate_zmp(first_stance_step, _t_before_first_step, _initial_param.get_double_stance(), _initial_param.get_max_steps(), dt, _zmp_t, _zmp_y); //TODO once filled, I shouldn't be able to modify them
//     // -----------------------------------------------------
    // generate zmp when lateral step is needed
    double first_stance_step_lat = _current_pose_ROS.get_sole(_other_side).coeff(1) - sign_first_stance_step * (_initial_param.get_indent_zmp() - _initial_param.get_lateral_step());;
    generate_zmp(first_stance_step_lat, _t_before_first_step, _initial_param.get_double_stance(), _initial_param.get_max_steps(), dt, _zmp_t_lat, _zmp_y_lat); //TODO once filled, I shouldn't be able to modify them
//     // -----------------------------------------------------
    
    
    if (_initial_param.get_delay_impact_scenario() == 0) // generate different ZMP with ramp to 0
    {
        Eigen::VectorXd point_t(3);
        Eigen::VectorXd point_y_right(3), point_y_left(3);
        
        point_t << 0, _initial_param.get_slope_delay_impact(), 10;
        point_y_right << first_stance_step, 0, 0;
        
        lSpline(point_t, point_y_right, dt, _zmp_t_fake_right, _zmp_y_fake_right);
        _zmp_t_fake_left = _zmp_t_fake_right;
        _zmp_y_fake_left = -_zmp_y_fake_right;
        
            for (int i = 0; i < (_zmp_y_fake_right).size(); i++)
        {
            _logger->add("zmp_y_fake_right", (_zmp_y_fake_right)(i));
        }
            for (int i = 0; i < (_zmp_y_fake_left).size(); i++)
        {
            _logger->add("zmp_y_fake_left", (_zmp_y_fake_left)(i));
        }
    }
    else if (_initial_param.get_delay_impact_scenario() == 1)  // generate different ZMP keeping the ZMP constant
    {  
        Eigen::VectorXd point_t(2);
        Eigen::VectorXd point_y_right(2), point_y_left(2);
        
        point_t << 0, 10; //TODO horizon for the delay
        point_y_right << -first_stance_step, -first_stance_step;
        
        lSpline(point_t, point_y_right, dt, _zmp_t_fake_right, _zmp_y_fake_right);
        _zmp_t_fake_left = _zmp_t_fake_right;
        _zmp_y_fake_left = -_zmp_y_fake_right;
        
            for (int i = 0; i < (_zmp_y_fake_right).size(); i++)
        {
            _logger->add("zmp_y_fake_right", (_zmp_y_fake_right)(i));
        }
            for (int i = 0; i < (_zmp_y_fake_left).size(); i++)
        {
            _logger->add("zmp_y_fake_left", (_zmp_y_fake_left)(i));
        }
        
//         if lateral step

        point_t << 0, 10; //TODO horizon for the delay
        point_y_right << -first_stance_step - _initial_param.get_lateral_step(), -first_stance_step - _initial_param.get_lateral_step();
        lSpline(point_t, point_y_right, dt, _zmp_t_fake_right_lat, _zmp_y_fake_right_lat);
        _zmp_t_fake_left_lat = _zmp_t_fake_right_lat;
        _zmp_y_fake_left_lat  = -_zmp_y_fake_right_lat;
    }
    else if (_initial_param.get_delay_impact_scenario() == 2)  // keep ZMP generated the same
    {
        // do nothing
    }
//     // -----------------------------------------------------
    
    
    // get impact position in time
    _planned_impacts.resize(_initial_param.get_max_steps(),1);
    
    for (int i = 1; i <= _initial_param.get_max_steps(); i++)
    {
        _planned_impacts(i-1) = _initial_param.get_start_time() +  _initial_param.get_duration_step() * i;
    }
    
    for (int i = 0; i < (_zmp_t).size(); i++)
    {
        _logger->add("zmp_t", (_zmp_t)(i));
        _logger->add("zmp_y", (_zmp_y)(i));
    }

    Eigen::MatrixXd Q(1,1);
    Eigen::MatrixXd R(1,1);
    
    Q << _initial_param.get_MPC_Q(); //1000000
    R << _initial_param.get_MPC_R();
    
    
    
    
    _MpC_lat = std::make_shared<item_MpC>(_initial_height, Ts, T, Q, R);
    
    zmp_window(_zmp_t, _zmp_y, 0, _MpC_lat->_window_length + 0, _zmp_window_t, _zmp_window_y);
    
    _zmp_window_y.setZero();
    _zmp_window_t.setZero();
    _com_y.setZero();
    _u.setZero();
    
        
    _lateral_step = 0;
//     _lateral_step_left = 0;
//     _lateral_step_right = 0;
    _init_completed = 1;
    
    std::cout << "Initialization complete." << std::endl;
        
    _starting_time = time;

    return 1;
                
};
