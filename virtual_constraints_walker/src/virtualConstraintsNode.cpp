#include <virtualConstraintsNode.h>
#include <atomic>


#include <boost/geometry.hpp>
#include <boost/polygon/polygon.hpp>
// #include <boost/geometry/geometries/polygon.hpp>
// #include <boost/geometry/geometries/adapted/boost_tuple.hpp>
#include <boost/geometry/geometries/adapted/boost_tuple.hpp>



#include <XmlRpcValue.h>

#define initialized ([] { \
    static std::atomic<bool> first_time(true); \
    return first_time.exchange(false); } ())
    

virtualConstraintsNode::virtualConstraintsNode()
    {
        
//         initialize_cmd_fake_q1(); //TODO
        
        
        std::string this_node_name = ros::this_node::getName();
        _logger = XBot::MatLogger::getLogger("/tmp/" + this_node_name);
        ros::NodeHandle n;
        
        _step_counter = 0;
        get_param_ros();
        
        _initial_pose = _current_pose_ROS;  
        _step.set_data_step( _current_pose_ROS.get_sole(_current_side), _current_pose_ROS.get_sole(_current_side), _current_pose_ROS.get_com(), _current_pose_ROS.get_com(), 0,0, getTime(),getTime()+2);
        
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
        int max_steps;
        double clearance_heigth, duration, drop;
        std::string first_side;

        /*default parameters*/
        double default_drop = -0.12;
        double default_clearance_heigth = 0.1;
        double default_duration = 2;
        std::string default_first_side = "Left";
        int default_max_steps = 10;
        
        drop = nh_priv.param("initial_crouch", default_drop);
        max_steps = nh_priv.param("max_steps", default_max_steps);
        duration = nh_priv.param("duration_step", default_duration);
        clearance_heigth = nh_priv.param("clearance_step", default_clearance_heigth);
        first_side = nh_priv.param("first_step_side", default_first_side);
        
        _initial_param.set_crouch(drop);
        _initial_param.set_max_steps(max_steps);
        _initial_param.set_duration_step(duration);
        _initial_param.set_clearance_step(clearance_heigth);
        
        if (first_side == "Left")
                _initial_param.set_first_step_side(robot_interface::Side::Left);
        else if (first_side == "Right")
                _initial_param.set_first_step_side(robot_interface::Side::Right);
        else std::cout << "unknown side starting command" << std::endl;
    }
            
double virtualConstraintsNode::getTime()
    {
        double time = ros::Time::now().toSec() - _starting_time;
        return time;
    }

Eigen::Vector3d virtualConstraintsNode::straighten_up_goal()
    {      
        Eigen::Vector3d straight_com = _current_pose_ROS.get_com();
        
        straight_com(0) = _current_pose_ROS.get_sole(_current_side).coeff(0); /*TODO*/
//         straight_com(1) = _current_pose_ROS.get_sole(_current_side).coeff(1) ;       /*TODO TOCHANGE*/ /* - _current_pose_ROS.get_sole(_current_side).coeff(1)/3.2 */
        straight_com(2) = _initial_param.get_crouch();
        /*TODO PUT DEFAULT POSITION*/
        _step.set_data_step( _current_pose_ROS.get_sole(_current_side), _current_pose_ROS.get_sole(_current_side), straight_com, straight_com, 0, 0, getTime(), getTime()+2);
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

bool virtualConstraintsNode::impact_detected()                
    {
//
            if (fabs(fabs(_current_pose_ROS.get_sole(_current_side).coeff(2)) - fabs(_terrain_heigth)) <= 1e-3 &&  
                fabs(_current_pose_ROS.get_sole(_current_side).coeff(0) - _initial_pose.get_sole(_current_side).coeff(0))>  0.1)
            {

                _current_pose_ROS.sense();
                
//                 get_supportPolygon();
                

                robot_interface::Side last_side = _current_side;
//                 std::cout << "Last side: " << last_side << std::endl;
               _initial_pose = _current_pose_ROS;
               _step_counter++;
                _current_side = robot_interface::Side::Double;
                
//                 get_supportPolygon();
                
                
                
                std::cout << "Impact! Current state: " << _current_side << std::endl;
                _current_pose_ROS.get_sole(_current_side);
                
                // ----------------------------------------------------
//                 Eigen::Vector3d current_com = _current_pose_ROS.get_com();
//                 current_com(2) = _current_pose_ROS.get_sole(robot_interface::Side(1 - static_cast<int>(last_side))).coeff(2) + _initial_height;
                _step.set_com_initial_pose(_current_pose_ROS.get_com());          //ADDED
                // ----------------------------------------------------
                
                if (_step_counter < _initial_param.get_max_steps())
                {
                    if (last_side == robot_interface::Side::Left)
                        _current_side = robot_interface::Side::Right;
                    else if (last_side == robot_interface::Side::Right)
                        _current_side = robot_interface::Side::Left;
                    else ROS_INFO("wrong side");
                    
                std::cout << "State changed. Current side: " << _current_side << std::endl;
                

                return 1;
                }
                else 
                {
                    return 0;
                }
                
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
    
// double virtualConstraintsNode::lat_controller(double starting_time, double phase = 0)
//     {
// 
//     }

// void virtualConstraintsNode::update_com()
//     {
//         Eigen::Vector3d initial_com_position;
//         Eigen::Vector3d final_com_position;
//         Eigen::Vector3d delta_com;
//         
//         double com_clearing;
//         
//         _current_pose_ROS.sense();
//         
//         initial_com_position = _current_pose_ROS.get_com();
//         final_com_position = _current_pose_ROS.get_com();
// 
//         delta_com = calc_com(_q1_state);      
// 
//         update_pose(&final_com_position, delta_com);    /*incline*/
//         
//         double starTime = getTime();
//         double endTime = starTime + _initial_param.get_duration_step();
//         
//         _step.set_com_initial_pose(initial_com_position);
//         _step.set_com_final_pose(final_com_position);
//         _step.set_com_clearing(com_clearing);
//         _step.set_starTime(starTime);
//         _step.set_endTime(endTime);
//     }
    
// void virtualConstraintsNode::update_step()
//     {
//         Eigen::Vector3d initial_sole_position;
//         Eigen::Vector3d final_sole_position;
//         Eigen::Vector3d delta_step;
//         
//         _current_pose_ROS.sense();
//         
//         initial_sole_position = _current_pose_ROS.get_sole(_current_side);
//         final_sole_position = _current_pose_ROS.get_sole(_current_side);
//         
//         delta_step = calc_step(_q1_state);       
//         
//         update_pose(&final_sole_position, delta_step);  /*step*/
//         
//         
//         double step_clearing = _initial_param.get_clearance_step();
//         double starTime = getTime();
//         double endTime = starTime + _initial_param.get_duration_step();
//         
//         _step.set_foot_initial_pose(initial_sole_position);
//         _step.set_foot_final_pose(final_sole_position);
//         _step.set_step_clearing(step_clearing);
//         _step.set_starTime(starTime);
//         _step.set_endTime(endTime);
//     }
void virtualConstraintsNode::fakeCOM()
    {
        _current_pose_ROS.sense();
        
        if (initialized) //initialized
        {   
            _step.get_com_initial_pose() = _current_pose_ROS.get_com();
//             std::cout << "starting com: " << _current_pose_ROS.get_com().transpose() << std::endl;
        }
        
        
        double q1_fake = get_q1();
        
        Eigen::Vector3d com_trajectory;
        robot_interface::Side other_side = (robot_interface::Side)(1 - static_cast<int>(_current_side));
        Eigen::Vector3d com_to_ankle_distance = _current_pose_ROS.get_distance_ankle_to_com(other_side);
        Eigen::Vector3d delta_com;
        delta_com << - com_to_ankle_distance.z() * tan(q1_fake), 0, 0;
        
        com_trajectory = _step.get_com_initial_pose() + delta_com;
        
//         std::cout << "q1_fake: " << get_q1() << std::endl;
//         std::cout << "---------------------------------------" << std::endl;
//         std::cout << "com_to_ankle_distance: " << com_to_ankle_distance.z() << std::endl;
//         std::cout << "com_initial_pose: " << _step.get_com_initial_pose().transpose() << std::endl;
//         std::cout << "delta_com: " << delta_com.transpose() << std::endl;
//         std::cout << "com_trajectory: " << com_trajectory.transpose() << std::endl;
//         std::cout << "current COM: " << _current_pose_ROS.get_com().transpose() << std::endl;
        
        _logger->add("com_trajectory", com_trajectory);
        _logger->add("delta_com", delta_com);
        _logger->add("heigth_com", com_to_ankle_distance.z());
        _logger->add("q1_fake", q1_fake);

        send_com(com_trajectory);
    }
    
void virtualConstraintsNode::run() 
    {
        

        if (initialized) //initialized
        {    

//                 start timer
//                 set_q1(_initial_q1);             
//                 first_q1();
                
                _q_min = sense_q1();
                _q_max = 0.3;
                                
                _current_side = _initial_param.get_first_step_side();
                std::cout << "State changed. First side: " << _current_side << std::endl;

                Eigen::Vector3d com_to_ankle_distance;
                Eigen::Vector3d delta_step;
                com_to_ankle_distance = _current_pose_ROS.get_distance_ankle_to_com(_current_side);
                delta_step << (- 2* com_to_ankle_distance.z() * tan(_q_max)), 0, 0;
                
                Eigen::Vector3d initial_sole_position;
                Eigen::Vector3d final_sole_position;

                initial_sole_position = _current_pose_ROS.get_sole(_current_side);
                final_sole_position = _current_pose_ROS.get_sole(_current_side);
                
                update_pose(&final_sole_position, delta_step);  /*step*/
               
                
                _step.set_foot_initial_pose(initial_sole_position);
                _step.set_foot_final_pose(final_sole_position);
                
        }
        

        Eigen::Vector3d pointsBezier_z;
        Eigen::Vector2d pointsBezier_x;        
        double clearance = 0.1;
        
        pointsBezier_z << 0, clearance*2, 0;
        pointsBezier_x << 0, 1;
        
//         sense_qlat();
        double q1 = sense_q1();
        
//         std::cout << q1 << std::endl;

        double tau = (q1 - _q_min) / (_q_max - _q_min);
        
   
// //                 double tau = (ros::Time::now().toSec() - _step.get_starTime()) / (_step.get_endTime() - _step.get_starTime());


        double trajectory_z = getBezierCurve(pointsBezier_z, tau);
        double trajectory_x = getBezierCurve(pointsBezier_x, tau);
        
        _logger->add("traj_bezier_z", trajectory_z);
        _logger->add("traj_bezier_x", trajectory_x);
        _logger->add("tau", tau);
        
        _logger->add("time", getTime());
    
        Eigen::Vector3d foot_trajectory = _step.get_foot_initial_pose();
        foot_trajectory(0) = (1 - trajectory_x) * _step.get_foot_initial_pose().coeff(0) + trajectory_x * _step.get_foot_final_pose().coeff(0);
        foot_trajectory(2) = _step.get_foot_initial_pose().coeff(2) + trajectory_z;
        
        
        _logger->add("q1", q1);

        _logger->add("foot_trajectory", foot_trajectory);
        send_step(foot_trajectory);

    }

bool virtualConstraintsNode::impact()
    {
        // // ---------------------------------impact----------------------------------------------------------
        if (impact_detected())
        {
            _q_min = sense_q1();
            
//             std::cout << "q_min_new: " << _q_min << std::endl;
            
            Eigen::Vector3d com_to_ankle_distance;
            Eigen::Vector3d delta_step;
            com_to_ankle_distance = _current_pose_ROS.get_distance_ankle_to_com(_current_side);
            delta_step << (- 4* com_to_ankle_distance.z() * tan(_q_max)), 0, 0;

            Eigen::Vector3d initial_sole_position;
            Eigen::Vector3d final_sole_position;
            initial_sole_position = _current_pose_ROS.get_sole(_current_side);
            final_sole_position = _current_pose_ROS.get_sole(_current_side);
            update_pose(&final_sole_position, delta_step);  /*step*/

            _step.set_foot_initial_pose(initial_sole_position);
            _step.set_foot_final_pose(final_sole_position);
            
//             std::cout << "current_com:" << _current_pose_ROS.get_com().transpose() << std::endl;
//             std::cout << "initial foot pose: " << _step.get_foot_initial_pose().transpose() << std::endl;
//             std::cout << "final foot pose: " <<_step.get_foot_final_pose().transpose() << std::endl;
            
            return 1;
        }
        else
            return 0;
// // -------------------------------------------------------------------------------------------------    
    
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
    
    double beta = 2; //2
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

void virtualConstraintsNode::FifthOrderPlanning(double x0, double dx0, double ddx0,
                                                double xf, double dxf, double ddxf,
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
    
    return x;
}

void virtualConstraintsNode::lSpline(Eigen::VectorXd times, Eigen::VectorXd y, double dt, Eigen::VectorXd &X, Eigen::VectorXd &Y)
{
    int j=0;
    dt = 1.0/100;
    std::cout << "dt: " << dt << std::endl;
    double n = times.size();
    std::cout << "n: " << n << std::endl;
    double N = 0;
        
    for(int i=0; i<n-1; i++)
    { 
            N = N + (times.coeff(i+1)-times.coeff(i))/dt;
    }
    
    std::cout << "N: " << N << std::endl;
    
    X.resize(N,1);
    Y.resize(N,1);
    
    for(int i=0; i<n-1; i++)
    {
        double yn,xn;
        for(xn= times.coeff(i); xn < times.coeff(i+1); xn = xn+dt)
        {
            yn=(y.coeff(i+1)-y.coeff(i))*(xn-times.coeff(i))/(times.coeff(i+1)-times.coeff(i))+y.coeff(i);
            Y[j] = yn;
            X[j] = xn;
            j++;
            
            _logger->add("X", yn);
            _logger->add("Y", xn);
        }
    }

}

void virtualConstraintsNode::traj_zmp(double y_start, double t_start, double T)
    {

        
        double step_duration = 0.3;
        double dt;
        dt = 1.0/100;
        
        double T_tot = T + t_start;
        
        double frac_part;
        int step_number = floor(T/step_duration);
        frac_part = T/step_duration - step_number;
        
        std::cout << "step_number: " << step_number << std::endl;
        std::cout << "frac_part: " << frac_part << std::endl;
        
        Eigen::VectorXd y, times;
        
        if (frac_part == 0)
        {
            y.resize(step_number*2 + 1,1);
            times.resize(step_number*2 + 1,1);
        }
        else
        {
            y.resize(step_number*2 + 2,1); 
            times.resize(step_number*2 + 2,1);
        }
        
        int size_y = y.size();
        int size_times = times.size();
        
        std::cout << "size_y: " << size_y << std::endl;
        std::cout << "size_times: " << size_times << std::endl;
        
        times(0) = t_start;
        y(0) = y_start;
        int myswitch = 1;
        int j = 1;
        for (int i = 1; i<=step_number; i++)
        {
            
            times(j) = t_start + step_duration*(floor(t_start/step_duration) + i);
            times(j+1) = t_start + step_duration*(floor(t_start/step_duration) + i);
            
            y(j) = myswitch * y_start;
            y(j+1) = myswitch * -1 * y_start;
            myswitch = -1 * myswitch;
            j = j+2;
            
        }
        
        if (frac_part != 0)
        {
            times(step_number*2 + 1) = T_tot;
                
            if (step_number % 2)
            {
                y(step_number*2 + 1) = -y_start;
            }
            else
            {
                y(step_number*2 + 1) = y_start;
            }
        }
        
        std::cout << "y:" <<  y.transpose() << std::endl;
        std::cout << "times:" <<  times.transpose() << std::endl;
        


        Eigen::VectorXd X;
        Eigen::VectorXd Y;
        
        lSpline(times, y, dt, X, Y);
    }

// void virtualConstraintsNode::initialize_cmd_fake_q1()
//     {
//     ros::NodeHandle n_cmd;
//     
//     _q1_pub = n_cmd.advertise<std_msgs::Float64>("/q1", 10);
// 
//     }

// void virtualConstraintsNode::cmd_fake_q1()
//     {
//     _q1_fake = _initial_q1 + _reset_condition + 0.1*(getTime());
//     _q1_pub.publish(_q1_fake);
//     }
// 
// void virtualConstraintsNode::set_q1(double cmd_q1)
//     {
//     _q1_pub.publish(cmd_q1);
//     }



Eigen::VectorXd virtualConstraintsNode::zmp_traj(Eigen::VectorXd times)
{
    int m = 1;
    int N = times.size();
    Eigen::VectorXd zmp_traj(N);
    
    for (int i = 0; i<N; i++)
    {
    zmp_traj(i) = cos(m*3.14*times(i)) >0;
    
    _logger->add("zmp_traj",zmp_traj(i));
    }
    return zmp_traj;
}






// double virtualConstraintsNode::lat_oscillator_com(double starting_time, double phase = 0)
//     {
//         _current_pose_ROS.sense();
// //         double delay = 0; //2* 3.14/(2*_initial_param.get_duration_step());
//         double delay = 3.14/(2*_initial_param.get_duration_step());
//         double t = getTime() - starting_time + delay;
//         
//         _reducer = 0.8;
//         double amp = _current_pose_ROS.get_distance_l_to_r_foot().coeff(1)/2 * _reducer;
//         double period = 2*3.14*t/(2*_initial_param.get_duration_step());
//        
//         double oscillator = amp * sin(period + phase);
//         return oscillator;
//     }



// int virtualConstraintsNode::initial_shift_action() /*if I just setted a publisher it would be harder to define when the action was completed*/
//     {
//         actionlib::SimpleActionClient<cartesian_interface::ReachPoseAction> ac_com("cartesian/com/reach", true); /*without /goal!!*/
//         actionlib::SimpleActionClient<cartesian_interface::ReachPoseAction> ac_step("cartesian/l_sole/reach", true); /*without /goal!!*/
//         double q1 = 0.1; /*STARTING ANGLE!!!!!*/
//         ROS_INFO("Waiting for action server to start.");
//         // wait for the action server to start
//         ac_com.waitForServer();
//         ac_step.waitForServer(); //will wait for infinite time
//         ROS_INFO("Action server started, sending goal.");
//         // send a goal to the action
//         cartesian_interface::ReachPoseGoal goal_com, goal_step;
// //      TODO there is a bug in cartesian/com/reach, adding waypoints won't work     
//         geometry_msgs::Pose cmd_shift_pos, cmd_shift_foot;
//         
//         Eigen::Vector3d pose_com = _current_pose_ROS.get_com();
//         pose_com(0) = (_current_pose_ROS.get_distance_ankle_to_com(_current_side).coeff(2) * tan(q1)) + _current_pose_ROS.get_com().coeff(0);
//         tf::pointEigenToMsg(pose_com, cmd_shift_pos.position);
//         
//         Eigen::Vector3d pose_step =  _current_pose_ROS.get_sole(_current_side);
//         pose_step(0) = (2 * _current_pose_ROS.get_distance_ankle_to_com(_current_side).coeff(2) * tan(q1)) + _current_pose_ROS.get_sole(_current_side).coeff(0);
//         tf::pointEigenToMsg(pose_step, cmd_shift_foot.position);
//         
//         float cmd_initial_time;
//         cmd_initial_time = 1;
// 
//         
//         
//         goal_com.frames.push_back(cmd_shift_pos); /*wants geometry_msgs::Pose*/
//         goal_com.time.push_back(cmd_initial_time);
//         
//         goal_step.frames.push_back(cmd_shift_foot); /*wants geometry_msgs::Pose*/
//         goal_step.time.push_back(cmd_initial_time);
//         
//         ac_com.sendGoal(goal_com);
//         ac_step.sendGoal(goal_step);
//     
//         //wait for the action to return
//         bool finished_before_timeout = ac_step.waitForResult(ros::Duration(5.0));
// 
//         if (finished_before_timeout)
//         {
//             actionlib::SimpleClientGoalState state = ac_step.getState();
//             ROS_INFO("Action finished: %s", state.toString().c_str());
//         }
//         else
//             ROS_INFO("Action did not finish before the time out.");
// 
//         /* fill initial pose with pose after straighten_up_action */
// //         _current_pose_ROS.sense();
//         _initial_pose = _current_pose_ROS; 
// //         sense_q1();
//         //exit
//         return 0;
// }