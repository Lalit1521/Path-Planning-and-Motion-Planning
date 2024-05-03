#include "lalit_sridharan_roslab7/rrt.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rrt");
    ros::NodeHandle nh;
    RRT rrt(nh);
    ros::spin();
    return 0;
}