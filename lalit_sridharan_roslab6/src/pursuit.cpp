#include <ackermann_msgs/AckermannDriveStamped.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/TransformStamped.h>

#include "csv_reader.h"
#include <string>
#include <vector>

#include "csv.h"
#include "math.h"

#define LOOKAHEAD_DISTANCE 1.2
#define KP 1.00

#define PI 3.1415927

class PurePursuit {
public:
    PurePursuit()
    {
        
        //Topic you want to publish
        pub_1 = n_.advertise<ackermann_msgs::AckermannDriveStamped>("/nav", 1);
        pub_2 = n_.advertise<visualization_msgs::Marker>("/env_viz", 100);
        //pub_3 = n_.advertise<visualization_msgs::Marker>("/dynamic_viz", 1000);

        //Topic you want to subscribe
        // sub_ = n_.subscribe("/pf/pose/odom", 1000, &SubscribeAndPublish::callback, this);
        sub_ = n_.subscribe("/odom", 1, &PurePursuit::callback, this);
        //sub2 = n_.subscribe("/pure_pursuit", 1, &PurePursuit::drive_callback, this);
        std::string CSV_path;
        n_.getParam("CSV_path", CSV_path);
        f110::CSVReader reader(CSV_path);
        global_path = reader.getData();
        // read in all the data
        
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time();
        marker.id = 0;
        marker.type = visualization_msgs::Marker::POINTS;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = 0.0;
        marker.pose.position.y = 0.0;
        marker.pose.position.z = 0.0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.color.a = 1.0;  // Don't forget to set the alpha!
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        geometry_msgs::Point points;
        for (int i=0; i<global_path.size(); ++i)
        {
            xes.push_back(global_path[i][0]);
            yes.push_back(global_path[i][1]);
            points.x = global_path[i][0];
            points.y = global_path[i][1];
            points.z = 0.0;
            marker.points.push_back(points);
        }
        
    }

    void callback(const nav_msgs::Odometry& odometry_info) 
    {
        x_current = odometry_info.pose.pose.position.x;
        y_current = odometry_info.pose.pose.position.y;
        double siny_cosp = 2.0 * (odometry_info.pose.pose.orientation.w * odometry_info.pose.pose.orientation.z + odometry_info.pose.pose.orientation.x * odometry_info.pose.pose.orientation.y);
        double cosy_cosp = 1.0 - 2.0 * (odometry_info.pose.pose.orientation.y * odometry_info.pose.pose.orientation.y + odometry_info.pose.pose.orientation.z * odometry_info.pose.pose.orientation.z);
        heading_current = std::atan2(siny_cosp, cosy_cosp);
        if (!flag) {
            double shortest_distance = 100.0;
            for (unsigned int i = 0; i < xes.size(); i++) {
                if ((xes[i] - x_current) * (xes[i] - x_current) + (yes[i] - y_current) * (yes[i] - y_current) < shortest_distance) {
                    shortest_distance = (xes[i] - x_current) * (xes[i] - x_current) + (yes[i] - y_current) * (yes[i] - y_current);
                    current_indx = i;
                }
            }
            flag = true;
        }
        while (std::sqrt((xes[current_indx] - x_current) * (xes[current_indx] - x_current) + (yes[current_indx] - y_current) * (yes[current_indx] - y_current)) < LOOKAHEAD_DISTANCE) {
            current_indx++;
            if (current_indx > xes.size() - 1) {
                current_indx = 0;
            }
        }
        double real_distance = std::sqrt((xes[current_indx] - x_current) * (xes[current_indx] - x_current) + (yes[current_indx] - y_current) * (yes[current_indx] - y_current));
        double lookahead_angle = std::atan2(yes[current_indx] - y_current, xes[current_indx] - x_current);
        double del_y = real_distance * std::sin(lookahead_angle - heading_current);
        angle = KP * 2.00 * del_y / (real_distance * real_distance);
        PurePursuit::reactive_control();
        pub_2.publish(marker);
    }

    void reactive_control() 
    {
        ackermann_msgs::AckermannDriveStamped ackermann_drive_result;
        ackermann_drive_result.drive.steering_angle = angle;
        /*
        if (std::abs(angle) > 20.0 / 180.0 * PI) {
            ackermann_drive_result.drive.speed = 0.5;  // 5.0
        } else if (std::abs(angle) > 10.0 / 180.0 * PI) {
            ackermann_drive_result.drive.speed = 1.0;
        } else {
            ackermann_drive_result.drive.speed = 1.5;
        }
        */
        std::srand(static_cast<unsigned>(std::time(0)));
        // Generate a random integer between 0 and 20
        int randomInt = std::rand() % 21;  // % 21 ensures the result is in the range [0, 20]

        // Convert the random integer to a float and scale it by 0.1
        float randomNumber = static_cast<float>(randomInt) * 0.1;
        ackermann_drive_result.drive.speed = randomNumber;
        pub_1.publish(ackermann_drive_result);
        // ROS_INFO_STREAM(angle);
    }

private:
    ros::NodeHandle n_;
    ros::Publisher pub_1;
    ros::Publisher pub_2;
    ros::Subscriber sub_;
    visualization_msgs::Marker marker;
    double angle;
    double x_current;
    double y_current;
    double heading_current;
    std::vector<double> xes;
    std::vector<double> yes;
    std::vector<double> headings;
    unsigned int current_indx;
    bool flag = false;
    std::vector<std::array<double, 2>> global_path;

    

};  // End of class SubscribeAndPublish

int main(int argc, char** argv) 
{
    //Initiate ROS
    ros::init(argc, argv, "pursuit");

    //Create an object of class SubscribeAndPublish that will take care of everything
    PurePursuit node;

    ros::spin();

    return 0;
}