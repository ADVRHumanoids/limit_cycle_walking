#include <ros/ros.h>
#include <footStabilizer/footStabilizer.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <eigen_conversions/eigen_msg.h>
#include <nav_msgs/Odometry.h>
#include <RobotInterfaceROS/ConfigFromParam.h>
#include <XBotInterface/RobotInterface.h>
#include <visualization_msgs/Marker.h>
#include <cartesian_interface/ros/RosImpl.h>


#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>

#include <dynamic_reconfigure/server.h>
#include <virtual_constraints_walker/gainsConfig.h>

class item_switch {
    public:
        
    item_switch() {_switch = 0;};
    
    bool get_value () {return _switch;};
    bool cmd_switch(std_srvs::SetBoolRequest& req, std_srvs::SetBoolResponse& res)
    {
        if (req.data)
        {
            res.message = "Start controller...";
            res.success = true;
            _switch = 1;
            std::cout << "Controller started" << std::endl;
        }
        else
        {
            res.message = "Stop controller...";
            res.success = false;
            _switch = 0;
            std::cout << "Controller stopped" << std::endl;
        }
        return true;
    }
    
    private:
        bool _switch = 0;
            
    };
class item_gains {
    
    public:
    
        item_gains() 
        {
            _kp.setZero(); 
            _kd.setZero(); 
            _old_kp.setZero();
            _old_kd.setZero();
            flag_called = 0;
            
        };
        
        Eigen::Vector3d getKp() 
        {
            return _kp;

        };
        Eigen::Vector3d getKd() 
        {
            return _kd;

        };
        
        bool areChanged()
        {
            if (flag_called)
            {
                flag_called = 0;
                return true;
            }
            else
            {
                return false;
            }
        }
        
        void callback(compute_cp::gainsConfig &config, uint32_t level) 
        {    
            std::cout << "Gains changed." << std::endl;
            _kp(0) = config.kp_x;
            _kp(1) = config.kp_y;
            std::cout << "Kp = " << _kp.transpose() << std::endl;
            _kd(0) = config.kd_x;
            _kd(1) = config.kd_y;
            std::cout << "Kd = " << _kd.transpose() << std::endl;
            flag_called = 1;
        }
    
    private:
        Eigen::Vector3d _old_kp, _old_kd;
        Eigen::Vector3d _kp, _kd;
        
        bool flag_called;
};

class item_fb {
        
    public:
        
        item_fb() 
        {
            _pos.matrix().setZero(); 
            _vel.setZero(); 
            _check_1 = 0;
            _check_2 = 0;
        }

        Eigen::Affine3d getPose() {return _pos;};
        Eigen::Matrix<double, 6, 1> getVelocity() {return _vel;};
        
        void pos_callback(const geometry_msgs::PoseStamped msg_rcv)
        {
            tf::poseMsgToEigen(msg_rcv.pose, _pos);
            _check_1 = 1;
        }
        
        void vel_callback(const geometry_msgs::TwistStamped msg_rcv)
        {
            tf::twistMsgToEigen(msg_rcv.twist, _vel);
            _check_2 = 1;
        }
        
        bool isReady() {return (_check_1 && _check_2);};
    private:
        Eigen::Affine3d _pos;
        Eigen::Matrix<double, 6, 1> _vel;
        bool _check_1, _check_2;
};

class item_com_odom {
        
    public:
        
        item_com_odom() 
        {
            _pos.setZero(); 
            _vel.setZero(); 
            _check_1 = 0;
        }

        Eigen::Vector3d getPosition() {return _pos;};
        Eigen::Vector3d getVelocity() {return _vel;};
        
        void odom_callback(const nav_msgs::Odometry msg_rcv)
        {
            Eigen::Matrix<double, 6, 1> vel;
            tf::pointMsgToEigen(msg_rcv.pose.pose.position, _pos);
            tf::twistMsgToEigen(msg_rcv.twist.twist, vel);
            _vel[0] = vel[0];
            _vel[1] = vel[1];
            _vel[2] = vel[2];
            
            _check_1 = 1;
        }

        
        bool isReady() {return (_check_1);};
    private:
        Eigen::Vector3d _pos;
        Eigen::Vector3d _vel;
        bool _check_1;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "compute_cp");
    ros::NodeHandle nh("xbotcore");
    
    auto cfg = XBot::ConfigOptionsFromParamServer(nh);
    auto robot = XBot::RobotInterface::getRobot(cfg);
    auto model = XBot::ModelInterface::getModel(cfg);
    
    /* DYNAMIC RECONFIGURE ITEMS */
    dynamic_reconfigure::Server<compute_cp::gainsConfig> server;
    dynamic_reconfigure::Server<compute_cp::gainsConfig>::CallbackType f;
    
    item_gains gains;
    f = boost::bind(&item_gains::callback, &gains, _1, _2);
    server.setCallback(f);
    
    /* ROS STUFF */
    ros::Rate loop_rate(100); //TODO set it from the robot
    double dt = 0.01;
   
    item_fb fb;
   
    item_com_odom com;
    
    item_switch switcher;
    
    /* BUILD STABILIZER */
    footStabilizer stab(dt);
   
    
    XBot::JointNameMap mapCogimon;
    
    ros::Publisher _cp_pub, _com_pub, _vis_pub, _com_odom_pub; 
    ros::Subscriber _com_sub, _fb_pos_sub, _fb_vel_sub;
    
    ros::Subscriber _com_odom_sub;
    
    ros::ServiceServer _switch_srv;
    
    
    XBot::Cartesian::RosImpl::Ptr ci;
    
    /* parameter server */
    bool default_com_sensing = 0;
    bool default_use_cartesio = 0;
    bool default_use_instantaneous_cp = 0;
    
    bool use_odom;
    bool use_cartesio;
    bool use_instantaneous_cp;
    
    ros::NodeHandle nh_priv("~");
    use_odom = nh_priv.param("using_com_from_odometry", default_com_sensing);
    use_cartesio = nh_priv.param("using_cartesio", default_use_cartesio);
    use_instantaneous_cp = nh_priv.param("using_instantaneous_cp", default_use_instantaneous_cp);
    
    /* SEND TOPICS FOR COM CP, ADVERTISE CP MARKER */
    
    /* using com odom */
    _com_odom_pub = nh.advertise<nav_msgs::Odometry>("/compute_cp/com_odom", 10);
    
    /* using com fake */
    _com_pub = nh.advertise<nav_msgs::Odometry>("/compute_cp/com_real", 10);
    _vis_pub = nh.advertise<visualization_msgs::Marker>( "cp", 0 );
    
    _cp_pub = nh.advertise<geometry_msgs::PointStamped>("/compute_cp/cp_measured", 10);
//     ros::topic::waitForMessage<geometry_msgs::PoseStamped>("/xbotcore/floating_base_position");
    
    /* GET FLOATING BASE TO UPDATE MODEL */
    _fb_pos_sub = nh.subscribe("/xbotcore/floating_base_position", 10, &item_fb::pos_callback, &fb);
    _fb_vel_sub = nh.subscribe("/xbotcore/floating_base_velocity", 10, &item_fb::vel_callback, &fb);
    
    /* GET TOPICS FROM ODOMETRY */
    _com_odom_sub = nh.subscribe("/SERoW/CoM/odom", 10, &item_com_odom::odom_callback, &com);
    
    /* SET SWITCH TOPIC FOR CONTROLLER */
    _switch_srv = nh.advertiseService("/compute_cp/controller_switch", &item_switch::cmd_switch, &switcher);
    
    
    /* INSTANTIATE CARTESIO OBJECT */
    if (use_cartesio)
    {   
        ci = std::make_shared<XBot::Cartesian::RosImpl>();
    }
    
    Eigen::Affine3d base_T_l_sole, base_T_r_sole;
    
    /* INITIALIZING */
    std::cout << "INITIALIZING ..." << std::endl;
    
    
    /* SET RECONFIGURABLE GAINS */
    stab.setKp(gains.getKp());
    stab.setKd(gains.getKd());
    std::cout << "Kp: " << gains.getKp().transpose() << std::endl;
    std::cout << "Kd: " << gains.getKd().transpose() << std::endl;
    
    if (use_cartesio)
    {
        std::cout << "using Cartesio to send commands" << std::endl;
    }
    else
    {
        std::cout << "using XBotCore to send commands" << std::endl;
    }
    
    if (use_odom)
    {
        std::cout << "using CoM from odometry SERoW" << std::endl;
    }
    else
    {
        std::cout << "using fake CoM from gazebo" << std::endl;
    }
    
    if (use_instantaneous_cp)
    {
        std::cout << "using instantaneous CP" << std::endl;
    }
    else
    {
        std::cout << "using CP" << std::endl;
    }
    
    while (!fb.isReady())
    {
        ros::spinOnce();
        fb.getPose();
        fb.getVelocity();
    }
    
    if (use_odom)
    {
        while (!com.isReady())
        {
            ros::spinOnce();
            com.getPosition();
            com.getVelocity();
        }
    }
    Eigen::Vector3d com_pos, com_vel;
    Eigen::Vector3d com_pos_odom, com_vel_odom;
    Eigen::Vector3d cp, cp_inst;
    
    
    
    com_pos.setZero();
    com_vel.setZero();
    cp.setZero();
    cp_inst.setZero();
    
    /* first run for INITIALIZATION (so that cp and previous cp are the same)*/
    robot->sense();
    model->syncFrom(*robot);
    
    model->setFloatingBaseState(fb.getPose(), fb.getVelocity());
    model->update();
        

    
    model->getJointPosition(mapCogimon);
    model->getCOM(com_pos);
    model->getCOMVelocity(com_vel);
    
    
    if (use_odom == 0)
    {
        /* UPDATING STABILIZER WITH COM FAKE (actual com of the robot from gazebo) */
        if (!use_instantaneous_cp)
        {
            /* cp */
            stab.update(com_pos, com_vel);

            Eigen::Vector3d initial_cp = stab.getCP();
            
            stab.setCPRef(initial_cp);
            stab.update(com_pos, com_vel);
        }
        else if (use_instantaneous_cp)
        {
            
            /* instantaneous cp */
            stab.update(com_pos[2], com_vel);
            
            Eigen::Vector3d initial_inst_cp = stab.getInstantaneousCP();
            
            stab.setInstantaneousCPRef(initial_inst_cp);
            stab.update(com_pos[2], com_vel);
        }
        
    }    
    else if (use_odom == 1)
    {
        /* UPDATING STABILIZER WITH COM ODOM (estimated com by serow) */
        if (!use_instantaneous_cp)
        {
            /* cp */
            stab.update(com.getPosition(), com.getVelocity());
            Eigen::Vector3d initial_cp = stab.getCP();
            
            stab.setCPRef(initial_cp);
            stab.update(com.getPosition(), com.getVelocity());
        }
        else if (use_instantaneous_cp)
        {
            /* instantaneous cp */
            stab.update((com.getPosition())[2], com.getVelocity());
            Eigen::Vector3d initial_inst_cp = stab.getInstantaneousCP();
            
            stab.setInstantaneousCPRef(initial_inst_cp);
            stab.update((com.getPosition())[2], com.getVelocity());
        }
        
    }
    
    /* GET STARTING POSITION OF ANKLES */
    if (use_cartesio)
    {
        
        ci->getPoseReference("l_sole", base_T_l_sole);
        ci->getPoseReference("r_sole", base_T_r_sole);

        /* get L pitch angle */
        double angle_l_pitch_cmd = (((Eigen::Matrix3d)base_T_l_sole.linear()).eulerAngles(2,1,0))[0];
        /* get L roll angle */
        double angle_l_roll_cmd = (((Eigen::Matrix3d)base_T_l_sole.linear()).eulerAngles(2,1,0))[1];

        
        /* get R pitch angle */
        double angle_r_pitch_cmd = (((Eigen::Matrix3d)base_T_r_sole.linear()).eulerAngles(2,1,0))[0];
        /* get R roll angle */
        double angle_r_roll_cmd = (((Eigen::Matrix3d)base_T_r_sole.linear()).eulerAngles(2,1,0))[1];
        
//         std::cout << base_T_l_sole.matrix() << std::endl;
//         std::cout << base_T_r_sole.matrix() << std::endl;
        
//         std::cout << "angle_cmd_pitch: " << angle_cmd << std::endl;
    }
    /* ---------------------------------- */
    std::cout << "DONE!" << std::endl;
    /* -------------------------------------------------------------------- */
    

    
    bool flag = 1;
    
    while (ros::ok())
    {
        ros::spinOnce();
        
        /* UPDATE PARAMETERS FROM DYNAMIC RECONFIGURE */
        
        robot->sense();
        model->syncFrom(*robot);
        
        model->setFloatingBaseState(fb.getPose(), fb.getVelocity());
        model->update();
        
        /* UPDATE GAINS DYNAMICALLY */
        if (gains.areChanged())
        {
            stab.setKp(gains.getKp());
            stab.setKd(gains.getKd());
        }

        model->getCOM(com_pos);
        model->getCOMVelocity(com_vel);        
        
        if (use_odom == 0)
        {
            /* UPDATING STABILIZER WITH COM FAKE */
            
            if (!use_instantaneous_cp)
            {
                /* compute with cp */
                stab.update(com_pos, com_vel);
            } else if (use_instantaneous_cp)
            {
                /* compute with istantaneous cp */
                stab.update(com_pos[2], com_vel);
            }
        }
        else if (use_odom == 1)
        {
            /* UPDATING STABILIZER WITH COM ODOM */
            if (!use_instantaneous_cp)
            {
                /* compute with cp */
                stab.update(com.getPosition(), com.getVelocity());
            } else if (use_instantaneous_cp)
            {
                /* compute with cp */
                stab.update((com.getPosition())[2], com.getVelocity());
            }
        }
        
        /* CALCULATE CP */
       
        if (!use_instantaneous_cp)
        {
            /* cp */
            cp = stab.getCP();
        }
        else if (use_instantaneous_cp)
        {
            /* instantaneous cp */
            cp_inst = stab.getInstantaneousCP();
        }
        
        /* CALCULATE STAB ANKLES and send it */
        if (switcher.get_value())
        {
            Eigen::Vector3d pos_ankle = stab.getTheta();
            
            if (!use_cartesio)
            {
                mapCogimon["LAnklePitch"] += pos_ankle[0];
                mapCogimon["RAnklePitch"] += pos_ankle[0];
//                 std::cout << "angle:" << pos_ankle[0] << std::endl;
//                 std::cout << "angle in degrees: " << pos_ankle[0] / M_PI  * 180 << std::endl;
                robot->setPositionReference(mapCogimon);
                robot->move();
            }
            else
            {
                
                /* initial angle */
                base_T_l_sole.linear() *= (Eigen::AngleAxisd(pos_ankle[0], Eigen::Vector3d::UnitY())).toRotationMatrix();
                base_T_r_sole.linear() *= (Eigen::AngleAxisd(pos_ankle[0], Eigen::Vector3d::UnitY())).toRotationMatrix();
                
                base_T_l_sole.linear() *= (Eigen::AngleAxisd(pos_ankle[1], Eigen::Vector3d::UnitX())).toRotationMatrix();
                base_T_r_sole.linear() *= (Eigen::AngleAxisd(pos_ankle[1], Eigen::Vector3d::UnitX())).toRotationMatrix();
                
                ci->setPoseReference("l_sole", base_T_l_sole);
                ci->setPoseReference("r_sole", base_T_r_sole);
                
//                 std::cout << "-----------------" << std::endl;
//                 std::cout << base_T_l_sole.matrix() << std::endl; 
            }
        }
        
//         Eigen::Affine3d T_sole_obs;
//         model->getPose("l_sole", T_sole_obs);
//         Eigen::Matrix3d rotMat = T_sole_obs.linear();
//         Eigen::Vector3d ea = rotMat.eulerAngles(2,1,0);
//         
//         std::cout << "angle read from model: " << ea[1] << std::endl;
        
        /* ------------------- visualization and topics ------------------------ */
        
        /* for odom com */
            
//         std::cout << "odom com pos:" << com.getPosition().transpose() << std::endl;
//         std::cout << "odom com vel:" << com.getVelocity().transpose() << std::endl;        

        /* for fake com */
//         
// //         std::cout << "fb_pos: " << fb.getPose().matrix() << std::endl; 
// //         std::cout << "fb_vel: " << fb.getVelocity().transpose() << std::endl;
//         
//         std::cout << "pos ankle: " << pos_ankle.transpose() << std::endl;
//         std::cout << "com position: " << com_pos.transpose() << std::endl;
//         std::cout << "com velocity: " << com_vel.transpose() << std::endl;
//         std::cout << "cp: " << cp.transpose() << std::endl;
//         std::cout << "------------------------" << std::endl;

        /* PUBLISH CP */
        geometry_msgs::PointStamped cmd_cp;
        tf::pointEigenToMsg(cp, cmd_cp.point);
        _cp_pub.publish(cmd_cp);
        
        /* PUBLISH COM FAKE */
        nav_msgs::Odometry cmd_com;
        cmd_com.pose.pose.position.x = com_pos[0];
        cmd_com.pose.pose.position.y = com_pos[1];
        cmd_com.pose.pose.position.z = com_pos[2];
//         
        cmd_com.twist.twist.linear.x = com_vel[0];
        cmd_com.twist.twist.linear.y = com_vel[1];
        cmd_com.twist.twist.linear.z = com_vel[2];

        _com_pub.publish(cmd_com);
        
        /* PUBLISH COM ODOM */
        nav_msgs::Odometry cmd_com_odom;
        cmd_com_odom.pose.pose.position.x = com.getPosition()[0];
        cmd_com_odom.pose.pose.position.y = com.getPosition()[1];
        cmd_com_odom.pose.pose.position.z = com.getPosition()[2];
//         
        cmd_com_odom.twist.twist.linear.x = com.getVelocity()[0];
        cmd_com_odom.twist.twist.linear.y = com.getVelocity()[1];
        cmd_com_odom.twist.twist.linear.z = com.getVelocity()[2];
        
        _com_odom_pub.publish(cmd_com_odom);
        
        
        /* ADVERTISE CP */
        visualization_msgs::Marker CP_marker;
        std::string child_frame_id = "world_gazebo";

        Eigen::Vector3d cp_vis(cp);
        
        CP_marker.header.frame_id = child_frame_id;
        CP_marker.header.stamp = ros::Time::now();
        CP_marker.ns = "CP";
        CP_marker.id = 0;
        CP_marker.type = visualization_msgs::Marker::SPHERE;
        CP_marker.action = visualization_msgs::Marker::ADD;

        CP_marker.pose.orientation.x = 0.0;
        CP_marker.pose.orientation.y = 0.0;
        CP_marker.pose.orientation.z = 0.0;
        CP_marker.pose.orientation.w = 1.0;
        CP_marker.pose.position.x = cp[0];
        CP_marker.pose.position.y = cp[1];
        CP_marker.pose.position.z = cp[2];

        CP_marker.color.a = 1.0;
        CP_marker.color.r = 0.0;
        CP_marker.color.g = 1.0;
        CP_marker.color.b = 1.0;

        CP_marker.scale.x = 0.02;
        CP_marker.scale.y = 0.02;
        CP_marker.scale.z = 0.02;

        _vis_pub.publish(CP_marker);
        
        loop_rate.sleep();
    }
    
}