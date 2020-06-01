#include <ros/footsteps_viz.h>

FootstepsSpawner::FootstepsSpawner(XBot::ModelInterface::Ptr model, ros::NodeHandle nh/*ros::NodeHandle nh*/) :
    _step_number(0),
    _initial_pose(false),
    _nh(nh),
    _prefix("ci/"),
    _model(model)
{
}

void FootstepsSpawner::initializeMarkers()
{


    std::vector<urdf::LinkSharedPtr> links;
    _model->getUrdf().getLinks(links);

    _footstep_pub = _nh.advertise<visualization_msgs::MarkerArray>( "il_porco", 0 );

    _mesh = boost::static_pointer_cast<urdf::Box>(_model->getUrdf().getLink("LFoot")->collision->geometry);



}

void FootstepsSpawner::spawnMarkers(mdof::RobotState state, ros::Time t)
{
    if (!_initial_pose)
    {
        if (state.feet_contact[0] && state.feet_contact[1])
        {
            _markers.markers.push_back(generateMarker(state.world_T_foot[0], t, {1.0, 0.0, 0.0, 1.0}));
            _markers.markers.push_back(generateMarker(state.world_T_foot[1], t, {1.0, 0.0, 0.0, 1.0}));
            _initial_pose = true;
        }
    }

    if (state.impact[0])
    {
        _markers.markers.push_back(generateMarker(state.world_T_foot[0], t));
    }
    else if (state.impact[1])
    {
        _markers.markers.push_back(generateMarker(state.world_T_foot[1], t, {1.0, 1.0, 0.0, 0.0}));
    }



    _footstep_pub.publish(_markers);


}

visualization_msgs::Marker FootstepsSpawner::generateMarker(Eigen::Affine3d pose, ros::Time t, std::array<float, 4> color)
{
    visualization_msgs::Marker marker;
    std::string bl = "world"; // _model->getFloatingBaseLink(bl);

    marker.header.frame_id = _prefix+bl;
    marker.header.stamp = t;
    marker.ns = "foostep";
    marker.id = _step_number++;

    marker.action = visualization_msgs::Marker::ADD;

    Eigen::Quaterniond q(pose.linear());

    marker.pose.position.x = pose.translation().x();
    marker.pose.position.y = pose.translation().y();
    marker.pose.position.z = pose.translation().z();

    marker.pose.orientation.w = q.w();
    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();

    marker.color.a = color[0];
    marker.color.r = color[1];
    marker.color.g = color[2];
    marker.color.b = color[3];

    marker.type = visualization_msgs::Marker::CUBE;

    marker.scale.x = _mesh->dim.x;
    marker.scale.y = _mesh->dim.y;
    marker.scale.z = _mesh->dim.z;

    return marker;

}


