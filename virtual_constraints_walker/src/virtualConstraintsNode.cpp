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
        double time = ros::Time::now().toSec();
        return time;
    }

Eigen::Vector3d virtualConstraintsNode::straighten_up_goal()
    {      
        Eigen::Vector3d straight_com = _current_pose_ROS.get_com();
        
//         straight_com(0) =  _current_pose_ROS.get_sole(_current_side).coeff(0);
//         straight_com(1) = _current_pose_ROS.get_sole(_current_side).coeff(1) ;       /*TODO TOCHANGE*/ /* - _current_pose_ROS.get_sole(_current_side).coeff(1)/3.2 */
        straight_com(2) = _initial_param.get_crouch();
        /*TODO PUT DEFAULT POSITION*/
        _step.set_data_step( _current_pose_ROS.get_sole(_current_side), _current_pose_ROS.get_sole(_current_side), straight_com, straight_com, 0, 0, getTime(), getTime()+2);
        return straight_com;
    }
    
int virtualConstraintsNode::straighten_up_action() /*if I just setted a publisher it would be harder to define when the action was completed*/
    {
        actionlib::SimpleActionClient<cartesian_interface::ReachPoseAction> ac_com("cartesian/com/reach", true); /*without /goal!!*/
        ROS_INFO("Waiting for action server to start.");
        // wait for the action server to start
        ac_com.waitForServer(); //will wait for infinite time
        ROS_INFO("Action server started, sending goal.");
        // send a goal to the action
        cartesian_interface::ReachPoseGoal goal; 
        geometry_msgs::Pose cmd_initial;
        
        tf::pointEigenToMsg(this->straighten_up_goal(), cmd_initial.position);
        float cmd_duration_time;
        cmd_duration_time = 1; //15;
   
        goal.frames.push_back(cmd_initial); /*wants geometry_msgs::Pose*/
        goal.time.push_back(cmd_duration_time);

        ac_com.sendGoal(goal);
    
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
        sense_q1();
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
        
        for (int i = 0; i < 4; i++)
        {
            boost::geometry::append(poly[robot_interface::Side::Left], point_t(vertex_poly[robot_interface::Side::Left].coeff(i,0), vertex_poly[robot_interface::Side::Left].coeff(i,1)));
        }
        boost::geometry::convex_hull(poly[robot_interface::Side::Left], hull[robot_interface::Side::Left]);
        
        
        for (int i = 0; i < 4; i++)
        {
            boost::geometry::append(poly[robot_interface::Side::Right], point_t(vertex_poly[robot_interface::Side::Right].coeff(i,0), vertex_poly[robot_interface::Side::Right].coeff(i,1)));
        }
        boost::geometry::convex_hull(poly[robot_interface::Side::Right], hull[robot_interface::Side::Right]);    
        
   
 
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
        _logger->add("full_polygon_hull", full_hull_point); /*TODO this is wrong, changing size*/
        _logger->add("left_polygon_hull", left_hull_point);
        _logger->add("right_polygon_hull", right_hull_point);
        
        _logger->add("L_sole_center", center_sole[robot_interface::Side::Left]);
        _logger->add("R_sole_center", center_sole[robot_interface::Side::Right]);
        
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

void virtualConstraintsNode::update_position(Eigen::Vector3d *current_pose, Eigen::Vector3d update) 
    {   
//       // update current_pose with update
        *current_pose = *current_pose + update;
    }
    
void virtualConstraintsNode::calc_step(double q1, Eigen::Vector3d *delta_com,  Eigen::Vector3d *delta_step)
    {
        Eigen::Vector3d com_to_ankle_distance;
        
        com_to_ankle_distance = _current_pose_ROS.get_distance_ankle_to_com(_current_side); /*if take out minus the robot steps back*/

        /* virtual constraints - very simple */
        *delta_com << - 2* com_to_ankle_distance.z() * tan(q1), 0, 0; /*calc x com distance from given angle q1*/
        *delta_step << - 4* com_to_ankle_distance.z() * tan(q1), 0, 0; /*calc step distance given q1*/  //lenght_leg * sin(q1)
            
            
        if (_step_counter == 0)
        {
            *delta_com  << (- com_to_ankle_distance.z() * tan(q1)); //+ _current_pose_ROS.get_com().coeff(0), 0, 0; /*calc x com distance from given angle q1*/
            *delta_step << (- 2* com_to_ankle_distance.z() * tan(q1)); //+ _current_pose_ROS.get_l_sole().coeff(0), 0, 0; /*calc step distance given q1*/  //lenght_leg * sin(q1)
        }
                    
        _logger->add("delta_com", *delta_com);
        _logger->add("delta_step", *delta_step);
    }

bool virtualConstraintsNode::impact_detected()                
    {
//
            if (fabs(fabs(_current_pose_ROS.get_sole(_current_side).coeff(2)) - fabs(_terrain_heigth)) <= 1e-5 &&  
                fabs( _current_pose_ROS.get_sole(_current_side).coeff(0) -  _initial_pose.get_sole(_current_side).coeff(0))>  0.1)
            {

                _current_pose_ROS.sense();
                
//                 get_supportPolygon();
                

                robot_interface::Side last_side = _current_side;
               _initial_pose = _current_pose_ROS;
               _step_counter++;
                _current_side = robot_interface::Side::Double;
                
                get_supportPolygon();
                
                std::cout << "Impact! Current state: " << _current_side << std::endl;
                _current_pose_ROS.get_sole(_current_side);

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
    
double virtualConstraintsNode::lat_oscillator_com(double starting_time, double phase = 0)
    {
        _current_pose_ROS.sense();
        double delay = - 2* 3.14/(2*_initial_param.get_duration_step());
        
        double t = getTime() - starting_time + delay;
        
        bool flag;
        double amp = _current_pose_ROS.get_distance_l_to_r_foot().coeff(1)/2;
        double period = 2*3.14*t/(2*_initial_param.get_duration_step());
       
        double oscillator = amp * sin(period + phase);
        return oscillator;
    }
    
void virtualConstraintsNode::update_step()
    {
        Eigen::Vector3d initial_com_position, initial_sole_position;
        Eigen::Vector3d final_com_position, final_sole_position;
        Eigen::Vector3d delta_com, delta_step;
        
        double com_clearing;
        
        _current_pose_ROS.sense();
        
        initial_com_position = _current_pose_ROS.get_com();
        
        final_com_position = _current_pose_ROS.get_com();

//         if (_step_counter == 0)
//         {
//             final_com_position(1) = 0;
//         }
        
        initial_sole_position = _current_pose_ROS.get_sole(_current_side);
        final_sole_position = _current_pose_ROS.get_sole(_current_side);

        calc_step(_q1_state, &delta_com, &delta_step);
        com_clearing = lateral_com();
        
//         delta_com[1] = delta_com_y;
        
        update_position(&final_com_position, delta_com);    /*incline*/
        update_position(&final_sole_position, delta_step);  /*step*/
        
        
        double step_clearing = _initial_param.get_clearance_step();
        double starTime = getTime();
        double endTime = starTime + _initial_param.get_duration_step();
        
        _step.set_data_step(initial_sole_position, final_sole_position, initial_com_position, final_com_position, step_clearing, com_clearing, starTime, endTime);
    }

double virtualConstraintsNode::lateral_com()
    {
        _current_pose_ROS.sense();
        double qlat = sense_qlat();ros::NodeHandle n;
        double delta_com_y;
        
        Eigen::Vector3d ankle_to_com_distance;

        ankle_to_com_distance = _current_pose_ROS.get_distance_ankle_to_com(_current_side);

        /* virtual constraints - very simple */
        if (_current_side == robot_interface::Side::Double)
        {
            delta_com_y = 0;
        }
        delta_com_y = ankle_to_com_distance.z() * tan(qlat); /*calc x com distance from given angle q1*/
  
        if (_step_counter == 0)
        {
            delta_com_y  = 0; 
        }    
        
        _logger->add("delta_com_lateral", delta_com_y);
        return delta_com_y;
    }
    
void virtualConstraintsNode::run() 
    {
        if (initialized) //initialized
        {    
                first_q1();
                _starting_time = getTime();
        }
        
        if (fabs(lat_oscillator_com(_starting_time) - fabs(_current_pose_ROS.get_sole(robot_interface::Side::Left).coeff(1))) <= 1e-5)
        {
            if (initialized)
            {
                _current_side = _initial_param.get_first_step_side();
                std::cout << "State changed. First side: " << _current_side << std::endl;
                update_step();
            }
        }
        
        sense_qlat();
        sense_q1();
        
        
        
        if (impact_detected())
        {
            update_step();
        }
        
        Eigen::Vector3d foot_trajectory, com_trajectory;
// // // // //         _step.get_com_clearing() // old lateral swing
        foot_trajectory = compute_swing_trajectory(_step.get_foot_initial_pose(), _step.get_foot_final_pose(), _step.get_step_clearing(), _step.get_starTime(), _step.get_endTime(), ros::Time::now().toSec(), "xy");
        com_trajectory = compute_swing_trajectory(_step.get_com_initial_pose(), _step.get_com_final_pose(), 0, _step.get_starTime(), _step.get_endTime(), ros::Time::now().toSec(), "xz");

        com_trajectory[1] = lat_oscillator_com(_starting_time);


        _step.log(_logger);
        
//         _logger->add("output_oscillator", lat_oscillator_com());
        _logger->add("com_trajectory", com_trajectory);
        _logger->add("foot_trajectory", foot_trajectory);
        
        send_step(foot_trajectory, com_trajectory);
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
void virtualConstraintsNode::send_step(Eigen::Vector3d foot_command, Eigen::Vector3d com_command)
    {
        geometry_msgs::PoseStamped cmd_com, cmd_sole;
        tf::pointEigenToMsg(com_command, cmd_com.pose.position);
        tf::pointEigenToMsg(foot_command, cmd_sole.pose.position);
        
        _com_pub.publish(cmd_com);
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
    
    double tau = std::min(std::max((time - t_start)/(t_end - t_start), 0.0), 1.0);
    double alpha = compute_swing_trajectory_normalized_plane(time_warp(tau, 2.0));
    
    if (plane == "xy" || plane == "yx")
    {
        ret.head<2>() = (1-alpha)*start.head<2>() + alpha*end.head<2>();
        ret.z() = start.z() + compute_swing_trajectory_normalized_clearing(end.z()/clearance, time_warp(tau, 2.0))*clearance; /*porcodio*/
     }
     else if (plane == "yz" || plane == "zy")
     {
         ret.tail<2>() = (1-alpha)*start.tail<2>() + alpha*end.tail<2>();
         ret.x() = start.x() + compute_swing_trajectory_normalized_clearing(end.x()/clearance, time_warp(tau, 2.0))*clearance; /*porcodio*/
     }
     else if (plane == "xz" || plane == "zx")
     {
        ret[0] = (1-alpha)*start[0] + alpha*end[0];
        ret[2] = (1-alpha)*start[2] + alpha*end[2];
        ret.y() = start.y() + compute_swing_trajectory_normalized_clearing(end.y()/clearance, time_warp(tau, 2.0))*clearance; /*porcodio*/
     }
    return ret;
}

double virtualConstraintsNode::compute_swing_trajectory_normalized_plane(double tau, double* __dx, double* __ddx)
{
    double x, dx, ddx;
    XBot::Utils::FifthOrderPlanning(0, 0, 0, 1, 0, 1, tau, x, dx, ddx);

//     if(__dx) *__dx = dx;
//     if(__ddx) *__ddx = ddx;
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