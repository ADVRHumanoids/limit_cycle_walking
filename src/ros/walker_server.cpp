#include <ros/walker_executor.h>
#include <RobotInterfaceROS/ConfigFromParam.h>

using namespace XBot::Cartesian;

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "walker_server_node");

    WalkerExecutor exec;

    while(ros::ok())
    {
        exec.run();
    }
}
