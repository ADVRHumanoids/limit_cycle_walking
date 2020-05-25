#ifndef _FOOTSTEPSPAWNER_VIZ_H_
#define _FOOTSTEPSPAWNER_VIZ_H_

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <robot/robot_state.h>
#include <XBotInterface/RobotInterface.h>

class FootstepsSpawner
{
public:

    typedef std::shared_ptr<FootstepsSpawner> Ptr;
    FootstepsSpawner(XBot::ModelInterface::Ptr model, ros::NodeHandle nh/*ros::NodeHandle nh*/);

    void initializeMarkers();
    void spawnMarkers(mdof::RobotState state, ros::Time t);
    visualization_msgs::Marker generateMarker(Eigen::Affine3d pose, ros::Time t, std::array<float, 4> color = {1.0, 0.0, 1.0, 0.0});
private:

    int _step_number;
    bool _initial_pose;

    ros::NodeHandle _nh;
    std::string _prefix;

    std::vector<urdf::LinkSharedPtr> _soles;

    XBot::ModelInterface::ConstPtr _model;

    ros::Publisher _footstep_pub;

    boost::shared_ptr<urdf::Box> _mesh;

    visualization_msgs::MarkerArray _markers;

};

#endif
