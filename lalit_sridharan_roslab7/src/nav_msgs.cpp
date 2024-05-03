#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_broadcaster.h>


using namespace std::chrono_literals;
class OccupancyGrid_Publisher
{
  public:
    OccupancyGrid_Publisher()
    {
      og_pub = n.advertise<nav_msgs::OccupancyGrid>("custom_occupancy_grid", 10);
      og_timer = n.createTimer(ros::Duration(0.5), boost::bind(&OccupancyGrid_Publisher::og_callback, this));
    }
  private:

    void og_callback()
    {

      auto occupancy_grid_msg = nav_msgs::OccupancyGrid();
      std::vector<signed char> og_array(30);
      og_array[0] = 100;
      og_array[1] = 100;
      og_array[5] = 100;
      og_array[6] = 50;
      og_array[7] = 50;
      og_array[8] = 50;
      og_array[20] = 50;
      og_array[21] = 50;
     /* for(int i=0;i<10;i++){
        if (i%2 == 0)
        {
            og_array[i] = 100;
        }
        else
        {
            og_array[i] =  0 ;
        }
      }*/
      
      occupancy_grid_msg.header.stamp = ros::Time().now();
      occupancy_grid_msg.header.frame_id = "map_frame";

      occupancy_grid_msg.info.resolution = 0.5;

      occupancy_grid_msg.info.width = 5;
      occupancy_grid_msg.info.height = 6;

      occupancy_grid_msg.info.origin.position.x = -2.0;
      occupancy_grid_msg.info.origin.position.y = 2.0;
      occupancy_grid_msg.info.origin.position.z = 0.0;
      occupancy_grid_msg.info.origin.orientation.x = 0.0;
      occupancy_grid_msg.info.origin.orientation.y = 0.0;
      occupancy_grid_msg.info.origin.orientation.z = 0.0;
      occupancy_grid_msg.info.origin.orientation.w = 1.0;
      occupancy_grid_msg.data = og_array;
      static tf::TransformBroadcaster br;
      tf::Transform transform;
      transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
      tf::Quaternion tf_quaternion;
      tf_quaternion.setX(0.0);
      tf_quaternion.setY(0.0);
      tf_quaternion.setZ(0.0);
      tf_quaternion.setW(occupancy_grid_msg.info.origin.orientation.w);                    
      transform.setRotation(tf_quaternion);
      br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map_frame", "world")); 
      //the value given in tf::vector and quaternion are values of base_link from map_frame


      og_pub.publish(occupancy_grid_msg);
    }

    ros::NodeHandle n;
    ros::Timer og_timer;
    ros::Publisher og_pub;
};

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "Occupancy_grid");
    OccupancyGrid_Publisher grid;
    ros::spin();
    return 0;
}