#include <mainNode.h>


mainNode::mainNode(int argc, char **argv, const char *node_name) 
{
    ros::init(argc, argv, node_name);
}