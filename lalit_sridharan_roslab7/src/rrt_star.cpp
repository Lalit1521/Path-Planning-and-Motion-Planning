#include "lalit_sridharan_roslab7/rrt.h"
#include "lalit_sridharan_roslab7/csv_reader.h"
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <thread>
#include <chrono>
#define PI 3.1415927
static const bool debug = true;

RRT::RRT(ros::NodeHandle &nh): nh_(nh), gen((std::random_device())()), tf_listener(tf_buffer)
{
    //ROS Topic names
    std::string pose_topic, scan_topic, drive_topic, CSV_path;
    nh_.getParam("pose_topic", pose_topic);
    nh_.getParam("scan_topic", scan_topic);
    nh_.getParam("drive_topic", drive_topic);
    nh_.getParam("CSV_path", CSV_path);

    //Load Input Map from map_server
    input_map = *(ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("map", ros::Duration(2)));
    
    if (input_map.data.empty())
    {
        std::__throw_invalid_argument("Input Map Load Unsuccesful");
    }  
    ROS_INFO("%d %d", input_map.info.width, input_map.info.height);
    ROS_INFO("%f %f", input_map.info.origin.position.x, input_map.info.origin.position.y);
    ROS_INFO("%zu", input_map.data.size());


    ROS_INFO("Map Load Successfull");
    
    //Read Global Path
    f110::CSVReader reader(CSV_path);
    global_path = reader.getData();

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

    //get transform from laser to map or global
    try
    {
        tf_laser_to_map = tf_buffer.lookupTransform("map", "laser", ros::Time(0));
    }
    catch (tf::TransformException& ex)
    {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
    }

    //ROS Publishers and Subscribers
    dynamic_map_pub = nh_.advertise<nav_msgs::OccupancyGrid>("dynamic_map", 1);
    waypoint_viz_pub = nh_.advertise<visualization_msgs::Marker>("env_viz", 100);
    scan_sub = nh_.subscribe(scan_topic, 1, &RRT::scan_callback, this);
    sleep(1);
    pose_sub = nh_.subscribe(pose_topic, 1, &RRT::pose_callback, this);
    drive_pub = nh_.advertise<ackermann_msgs::AckermannDriveStamped>(drive_topic, 1);
    tree_edge_viz_pub = nh_.advertise<visualization_msgs::Marker>("tree_edge_viz_marker", 1);
    tree_node_viz_pub = nh_.advertise<visualization_msgs::Marker>("tree_node_viz_marker", 1);
    rrt_solution_pub = nh_.advertise<visualization_msgs::Marker>("rrt_solution_marker", 1);
    local_trackpoint_pub = nh_.advertise<visualization_msgs::Marker>("local_trackpoint_marker", 1);

    // Waypoint Visualization
    unique_id = 0;

    //Map data
    map_cols = input_map.info.width;
    new_obstacles = {};
    //new_obstacles.reserve(10000);
    clear_obstacles_count = 0;

    //RRT Parameter
    nh_.getParam("max_rrt_iters", max_rrt_iters);
    nh_.getParam("max_expansion_distance", max_expansion_distance);
    nh_.getParam("collision_checking_points", collision_checking_points);
    nh_.getParam("goal_tolerance", goal_tolerance);
    nh_.getParam("lookahead_distance", lookahead_distance);
    nh_.getParam("local_lookahead_distance", local_lookahead_distance);

    // Local Map Parameters
    nh_.getParam("inflation_radius", inflation_radius);

    // RRT* Parameters
    nh_.getParam("enable_rrtstar", enable_rrtstar);
    nh_.getParam("search_radius", search_radius);

    // Car Parameters
    nh_.getParam("high_speed", high_speed);
    nh_.getParam("medium_speed", medium_speed);
    nh_.getParam("low_speed", low_speed);
    waypoint_viz_pub.publish(marker);
    ROS_INFO("Created new RRT Object");

}

/// The scan callback updates the occupancy grid
/// @param scan_msg - pointer to the incoming scan message

void RRT::scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msgs)
{
    try
    {
        tf_laser_to_map = tf_buffer.lookupTransform("map", "laser", ros::Time(0));
    }
    catch (tf::TransformException& ex)
    {
        ROS_ERROR("%s", ex.what());
        ros::Duration(0.1).sleep();
    }        
    const auto translation = tf_laser_to_map.transform.translation;
    const double yaw = tf::getYaw(tf_laser_to_map.transform.rotation);

    const auto start = static_cast<int>(scan_msgs->ranges.size()/6);
    const auto end = static_cast<int>(5*scan_msgs->ranges.size()/6);
    const double angle_increment = scan_msgs->angle_increment;
    double theta = scan_msgs->angle_min + angle_increment*(start-1);
    /*ROS_INFO("%zu", static_cast<size_t>(start));
    ROS_INFO("start");
    ROS_INFO("%zu", static_cast<size_t>(end));
    ROS_INFO("end");*/
    for (int i=start; i<end; ++i)
    {
        
        theta +=angle_increment;
        const double hit_range = scan_msgs->ranges[i];
        if (std::isinf(hit_range) || std::isnan(hit_range))
        {
            continue;
        }
        // laser hit x, y in base_link frame
        const double x_base_link = hit_range * cos(theta);
        const double y_base_link = hit_range * sin(theta);

        if (x_base_link > lookahead_distance || y_base_link >lookahead_distance)
        {
            continue;
        }

        //laser hit x,y in map frame
        const double x_map = x_base_link*cos(yaw) - y_base_link*sin(yaw) + translation.x;
        const double y_map = x_base_link*sin(yaw) + y_base_link*cos(yaw) + translation.y;

        std::vector<int> index_of_expanded_obstacles = get_expanded_row_major_indices(x_map, y_map);
        
        for (const auto& index: index_of_expanded_obstacles)
        {
        
            if(input_map.data[index] != 100)
            {
                input_map.data[index] = 100;
                new_obstacles.emplace_back(index);
            }
        }
    }
    /*clear_obstacles_count++;
    if (clear_obstacles_count > 50)
    {
        for (const auto index: new_obstacles)
        {
            input_map.data[index] = 0;
        }
        new_obstacles.clear();
        clear_obstacles_count = 0;
        //ROS_INFO("Obstacles Cleared");
    }*/

    dynamic_map_pub.publish(input_map);
    //ROS_INFO("Map Updated");

}

/// The pose callback when subscribed to particle filter's inferred pose (RRT* Main Loop)
/// @param pose_msg - pointer to the incoming pose message
void RRT::pose_callback(const geometry_msgs::PoseStamped::ConstPtr &pose_msg)
{
   
    current_x = pose_msg->pose.position.x;
    current_y = pose_msg->pose.position.y;

    /*if (unique_id == 0)
    {
        visualize_waypoint_data();
    } */
    

    const auto trackpoint = get_best_global_trackpoint({pose_msg->pose.position.x,pose_msg->pose.position.y});
    std::vector<Node> tree;
    tree.emplace_back(Node(pose_msg->pose.position.x, pose_msg->pose.position.y, 0.0, -1));

    int count = 0;
    while(count < max_rrt_iters)
    {
        count++;
        // Sample a Node
        const auto sample_node = sample(); //it has point(x, y) not node

        //Check if it is occupied or not
        if (is_collided(sample_node[0], sample_node[1]))
        {
            ROS_DEBUG("Sample Node Colliding");
            continue;
        }
        //Find the nearest node in the tree to the sample node
        const int nearest_node_id = nearest(tree, sample_node); //return index id of tree of nearest node

        //Steer the tree from the nearest node towards the sample node
        Node new_node = steer(tree[nearest_node_id], nearest_node_id, sample_node);//return the new_node
        const auto current_node_index = tree.size();

        // Check if the segment joining the two nodes is in collision
        if(is_collided(tree[nearest_node_id], new_node))
        {
            ROS_DEBUG("Sample node edge is colliding");
            continue;
        }
        else if (enable_rrtstar)
        {
            new_node.cost = cost(tree, new_node);
            const auto near_indices = near(tree, new_node);

            std::vector<bool> is_near_neighbour_collided;
            int best_neighbour = new_node.parent_index;

            for (const int near_node_index: near_indices)
            {
                if(is_collided(tree[near_node_index], new_node))
                {
                    is_near_neighbour_collided.push_back(true);
                    continue;
                }
                is_near_neighbour_collided.push_back(false);

                double cost = tree[near_node_index].cost + line_cost(tree[near_node_index], new_node);
                if (cost < new_node.cost)
                {
                    new_node.cost = cost;
                    new_node.parent_index = near_node_index;
                    best_neighbour = near_node_index;
                }
            }

            for(int i=0; i<near_indices.size(); i++)
            {
                if(is_near_neighbour_collided[i] || i == best_neighbour)
                {
                    continue;
                }
                if(tree[near_indices[i]].cost > new_node.cost + line_cost(
                        new_node, tree[near_indices[i]]))
                {
                    ROS_DEBUG("Rewiring Parents");
                    tree[near_indices[i]].parent_index = current_node_index;
                }
            
            }
        }
        tree.emplace_back(new_node);
        ROS_DEBUG("Sample Node Edge Found");

        if (is_goal(new_node, trackpoint[0], trackpoint[1]))
        {
            ROS_DEBUG("Goal Reached. Backtracking ...");
            local_path = find_path(tree, new_node);
            ROS_INFO("%d", count);

            const auto trackpoint_and_distance =
                    get_best_local_trackpoint({pose_msg->pose.position.x, pose_msg->pose.position.y});
            const auto local_trackpoint = trackpoint_and_distance.first;
            const double distance = trackpoint_and_distance.second;
            geometry_msgs::Pose goal_way_point;
            goal_way_point.position.x = local_trackpoint[0];
            goal_way_point.position.y = local_trackpoint[1];
            goal_way_point.position.z = 0;
            goal_way_point.orientation.x = 0;
            goal_way_point.orientation.y = 0;
            goal_way_point.orientation.z = 0;
            goal_way_point.orientation.w = 1;

            geometry_msgs::Pose goal_way_point_car_frame;
            tf2::doTransform(goal_way_point, goal_way_point_car_frame, tf_map_to_laser);
            
            //Vizualization nodes,edges,rrt_solution path, local_track_point
            //gen_tree_marker(tree, 1, 0, 0);
            //gen_node_marker(tree, 0, 0, 1);
            gen_path_marker(local_path);
            viz_point(trackpoint);
            

            const double steering_angle = 2*(goal_way_point_car_frame.position.y)/pow(distance, 2);
            publish_corrected_speed_and_steering(steering_angle);
            break;
        }
    }

}

/// This method returns the nearest node id on the tree to the sampled point
/// @param tree - the current RRT tree
/// @param sampled_point - the sampled point in free space
/// @return - id of nearest node on the tree
std::array<double, 2> RRT::sample()
{
    std::uniform_real_distribution<>::param_type x_param(0, lookahead_distance);
    std::uniform_real_distribution<>::param_type y_param(-lookahead_distance, lookahead_distance);
    x_dist.param(x_param);
    y_dist.param(y_param);

    geometry_msgs::Pose sample_point;
    sample_point.position.x = x_dist(gen);
    sample_point.position.y = y_dist(gen);
    sample_point.position.z = 0;
    sample_point.orientation.x = 0;
    sample_point.orientation.y = 0;
    sample_point.orientation.z = 0;
    sample_point.orientation.w = 1;
    tf2::doTransform(sample_point, sample_point, tf_laser_to_map);

    return {sample_point.position.x, sample_point.position.y};
}

/// This method returns the nearest node id on the tree to the sampled point
/// @param tree - the current RRT tree
/// @param sampled_point - the sampled point in free space
/// @return - id of nearest node on the tree
int RRT::nearest(const std::vector<Node> &tree, const std::array<double,2> &sampled_point)
{
    int nearest_node = 0;
    double nearest_node_distance = std::numeric_limits<double>::max();
    for (size_t i=0; i<tree.size(); i++)
    {
        const auto distance_sqr = pow(tree[i].x - sampled_point[0], 2) + pow(tree[i].y - sampled_point[1], 2);
        if (distance_sqr < nearest_node_distance)
        {
            nearest_node = i;
            nearest_node_distance = distance_sqr;
        }
    }
    return nearest_node;
}

/// The function steer:(x,y)->z returns a point such that z is “closer”
/// to y than x is. The point z returned by the function steer will be
/// such that z minimizes ||z−y|| while at the same time maintaining
/// ||z−x|| <= max_expansion_dist, for a prespecified max_expansion_dist > 0
/// basically, expand the tree towards the sample point (within a max dist)
/// @param nearest_node - nearest node on the tree to the sampled point
/// @param sampled_point - the sampled point in free space
/// @return - new node created from steering
Node RRT::steer(const Node &nearest_node, const int nearest_node_index, const std::array<double, 2> &sample_point)
{
    const double x = sample_point[0] - nearest_node.x;
    const double y = sample_point[1] - nearest_node.y;
    const double distance = pow(x,2) + pow(y,2);
    Node new_node{};
    if (distance < max_expansion_distance)
    {
        new_node.x = sample_point[0];
        new_node.y = sample_point[1];
    }
    else
    {
        const double theta = atan2(y, x);
        new_node.x = nearest_node.x + cos(theta)*max_expansion_distance;
        new_node.y = nearest_node.y + sin(theta)*max_expansion_distance;
    }
    new_node.parent_index = nearest_node_index;
    return new_node;
}

/// This method returns a boolean indicating if the path between the
/// nearest node and the new node created from steering is collision free
/// @param nearest_node - nearest node on the tree to the sampled point
/// @param new_node - new node created from steering
/// @return - true if in collision, false otherwise
bool RRT::is_collided(const Node &nearest_node, const Node &new_node)
{
    double x_increment = (new_node.x - nearest_node.x)/collision_checking_points;
    double y_increment = (new_node.y - nearest_node.y)/collision_checking_points;
    double current_x = nearest_node.x;
    double current_y = nearest_node.y;

    for (int i=0; i<collision_checking_points; i++)
    {
        current_x += x_increment;
        current_y += y_increment;
        if (is_collided(current_x, current_y))
        {
            return true;
        }
    }
    return false;
}

/// This method returns the cost associated with a node
/// @param tree - the current tree
/// @param node - the node the cost is calculated for
/// @return - the cost value associated with the node
double RRT::cost(const std::vector<Node> &tree, const Node &new_node)
{
    return tree[new_node.parent_index].cost + line_cost(tree[new_node.parent_index], new_node);
}

/// This method returns the cost of the straight line path between two nodes
/// @param n1 - the Node at one end of the path
/// @param n2 - the Node at the other end of the path
/// @return - the cost value associated with the path (Euclidean Distance)
double RRT::line_cost(const Node &n1, const Node &n2)
{
    return sqrt(pow(n1.x - n2.x, 2) + pow(n1.y - n2.y, 2));
}

/// This method returns the set of Nodes in the neighborhood of a node. (Not Implemented)
/// @param tree - the current tree
/// @param node - the node to find the neighborhood for
/// @return - the index of the nodes in the neighborhood
std::vector<int> RRT::near(const std::vector<Node> &tree, const Node &node)
{
    std::vector<int> near_neighbour_indices;
    for (int i = 0; i<tree.size(); i++)
    {
        const double distance = sqrt(pow(node.x - tree[i].x, 2) + pow(node.y - tree[i].y, 2));
        if (distance < search_radius)
        {
            near_neighbour_indices.push_back(i);
        }
    }
    return near_neighbour_indices;
}

/// This method checks if the latest node added to the tree is close
/// enough (defined by goal_threshold) to the goal so we can terminate
/// the search and find a path
/// @param latest_added_node - latest addition to the tree
/// @param goal_x - x coordinate of the current goal in map frame
/// @param goal_y - y coordinate of the current goal in map frame
/// @return - true if node close enough to the goal
bool RRT::is_goal(const Node &latest_added_node, double goal_x, double goal_y)
{
    const double distance = sqrt(pow(latest_added_node.x - goal_x,2) + pow(latest_added_node.y - goal_y,2));
    return distance < goal_tolerance;
}

/// This method traverses the tree from the node that has been determined
/// as goal
/// @param tree - the RRT tree
/// @param latest_added_node - latest addition to the tree that has been
/// determined to be close enough to the goal
/// @return - the vector that represents the order of the nodes traversed as the found path
std::vector<std::array<double, 2>> RRT::find_path(const std::vector<Node> &tree, const Node&latest_added_node)
{
    std::vector<std::array<double, 2>> found_path;
    Node current_node = latest_added_node;
    while (current_node.parent_index != -1)
    {
        std::array<double, 2> local_trackpoint{current_node.x, current_node.y};
        found_path.emplace_back(local_trackpoint);
        current_node = tree[current_node.parent_index];
    }
    return found_path;
}


/// Checks if a sample in the workspace is colliding with an obstacle
/// @param x_map - x coordinates in map frame
/// @param y_map - y coordinates in map frame
/// @return true if the point is colliding with an obstacle
bool RRT::is_collided(const double x_map, const double y_map)
{
    const auto index = get_row_major_index(x_map, y_map);
    return input_map.data[index] == 100;
}

/// Returns the row major index for the map
/// @param x_map - x coordinates in map frame
/// @param y_map - y coordinates in map frame
/// @return row major index of the map
int RRT::get_row_major_index(const double x_map, const double y_map)
{
    const auto x_index = static_cast<int>((x_map - input_map.info.origin.position.x)/input_map.info.resolution);
    const auto y_index = static_cast<int>((y_map - input_map.info.origin.position.y)/input_map.info.resolution);
    return y_index * map_cols + x_index;    
}

/// Returns the row major indeices for the map of an inflated area around a point based on inflation radius
/// @param x_map - x coordinates in map frame
/// @param y_map - y coordinates in map frame
/// @return row major index of the map
std::vector<int> RRT::get_expanded_row_major_indices(const double x_map, const double y_map)
{
    std::vector<int> expanded_row_major_indices;
    
    const auto x_index = static_cast<int>((x_map - input_map.info.origin.position.x)/input_map.info.resolution);
    const auto y_index = static_cast<int>((y_map - input_map.info.origin.position.y)/input_map.info.resolution);
    for (int i= -inflation_radius+x_index; i<inflation_radius+1+x_index; i++)
    {
        
        for (int j=-inflation_radius+y_index; j<inflation_radius+1+y_index; j++)
        {
            
            expanded_row_major_indices.emplace_back(j*map_cols + i);
        }
    }
    return expanded_row_major_indices;
}

/// Returns the best way point from the local plan to track
/// @param current_pose - current pose (x, y) of the car in map frame
/// @param lookahead_distance - ideal distance to find a trackpoint ahead of the current pose
/// @return trackpoint (x, y) in map frame

std::array<double, 2> RRT::get_best_global_trackpoint(const std::array<double, 2>& current_pose)
{
    try
    {
        tf_map_to_laser = tf_buffer.lookupTransform("laser", "map", ros::Time(0));
    }
    catch(tf::TransformException& ex)
    {
        ROS_ERROR("%s", ex.what());
        ros::Duration(0.1).sleep();
    }
    
    int best_trackpoint_index = -1;
    double best_trackpoint_distance = std::numeric_limits<double>::max();

    for (int i=0; i<global_path.size(); ++i)
    {
        geometry_msgs::Pose goal_way_point;
        goal_way_point.position.x = global_path[i][0];
        goal_way_point.position.y = global_path[i][1];
        goal_way_point.position.z = 0;
        goal_way_point.orientation.x = 0;
        goal_way_point.orientation.y = 0;
        goal_way_point.orientation.z = 0;
        goal_way_point.orientation.w = 1;
        tf2::doTransform(goal_way_point, goal_way_point, tf_map_to_laser);
        if (goal_way_point.position.x < 0)
        {
            continue;
        }  

        double distance = std::abs(lookahead_distance - 
            sqrt(pow(goal_way_point.position.x, 2)+ pow(goal_way_point.position.y, 2)));      
        if (distance < best_trackpoint_distance)
        {
            const auto row_major_index = get_row_major_index(global_path[i][0], global_path[i][1]);
            if (input_map.data[row_major_index] == 100)
            {
                continue;
            }
            best_trackpoint_distance = distance;
            best_trackpoint_index = i;
        }
    }
    return global_path[best_trackpoint_index];
}

/// Returns the best way point from the local plan to track
/// @param current_pose - current pose (x, y) of the car in map frame
/// @param lookahead_distance - ideal distance to find a trackpoint ahead of the current pose
/// @return trackpoint (x, y) in map frame
std::pair<std::array<double, 2>, double> RRT::get_best_local_trackpoint(const std::array<double,2>& current_pose)
{
    std::array<double, 2> closest_point{};
    double closest_distance_to_current_pose = std::numeric_limits<double>::max();
    double closest_distance = std::numeric_limits<double>::max();
    for (const auto& trackpoint : local_path)
    {
        double distance = sqrt(pow(trackpoint[0] - current_pose[0],2)
                                + pow(trackpoint[1] - current_pose[1],2));
        double diff_distance = std::abs(local_lookahead_distance - distance);
        if (diff_distance < closest_distance)
        {
            closest_distance_to_current_pose = distance;
            closest_distance = diff_distance;
            closest_point = trackpoint;
        }
    }
    return {closest_point, closest_distance_to_current_pose};
}

/// Publish way point with respect to the given input properties
/// @param way_point - (x, y) wrt to the frame id
/// @param frame_id - frame represented in waypoint
/// @param r - red intensity
/// @param g - green intensity
/// @param b - blue intenisty
/// @param transparency (Do not put 0)
/// @param scale_x
/// @param scale_y
/// @param scale_z
void RRT::add_way_point_visualization(const std::array<double, 2>& way_point, const std::string& frame_id, double r,
        double g, double b, double transparency = 0.5, double scale_x=0.2, double scale_y=0.2, double scale_z=0.2)
{
    visualization_msgs::Marker way_point_marker;
    way_point_marker.header.frame_id = frame_id;
    way_point_marker.header.stamp = ros::Time();
    way_point_marker.ns = "pure_pursuit";
    way_point_marker.id = unique_id;
    way_point_marker.type = visualization_msgs::Marker::SPHERE;
    way_point_marker.action = visualization_msgs::Marker::ADD;
    way_point_marker.pose.position.x = way_point[0];
    way_point_marker.pose.position.y = way_point[1];
    way_point_marker.pose.position.z = 0;
    way_point_marker.pose.orientation.x = 0.0;
    way_point_marker.pose.orientation.y = 0.0;
    way_point_marker.pose.orientation.z = 0.0;
    way_point_marker.pose.orientation.w = 1.0;
    way_point_marker.scale.x = 0.2;
    way_point_marker.scale.y = 0.2;
    way_point_marker.scale.z = 0.2;
    way_point_marker.color.a = transparency;
    way_point_marker.color.r = r;
    way_point_marker.color.g = g;
    way_point_marker.color.b = b;
    waypoint_viz_pub.publish(way_point_marker);
    unique_id++;
}

/// visualize all way points in the global path
void RRT::visualize_waypoint_data()
{
    const size_t increment = global_path.size()/100;
    for(size_t i=0; i<global_path.size(); i++)
    {
        add_way_point_visualization(global_path[i], "map", 1.0, 0.0, 1.0, 0.5);
    }
    ROS_INFO("Published All Global WayPoints.");
}

/// visualize trackpoints handles the creation and deletion of trackpoints
void RRT::visualize_trackpoints(double x_local, double y_local, double x_global, double y_global)
{
    visualization_msgs::Marker line;

    line.header.frame_id    = "/map";
    line.header.stamp       = ros::Time::now();
    line.lifetime           = ros::Duration(0.1);
    line.id                 = 1;
    line.ns                 = "rrt";
    line.type               = visualization_msgs::Marker::LINE_STRIP;
    line.action             = visualization_msgs::Marker::ADD;
    line.pose.orientation.w = 1.0;
    line.scale.x            = 0.05;
    line.color.r            = 0.0f;
    line.color.g            = 0.0f;
    line.color.b            = 1.0f;
    line.color.a            = 1.0f;

    geometry_msgs::Point current;
    geometry_msgs::Point local;
    geometry_msgs::Point global;

    current.x = current_x;
    current.y = current_y;
    current.z =  0;

    local.x = x_local;
    local.y = y_local;
    local.z = 0;

    global.x = x_global;
    global.y = y_global;
    global.z = 0;

    line.points.push_back(current);
    line.points.push_back(local);
    line.points.push_back(global);

    rrt_solution_pub.publish(line);
    unique_id++;
}

//stores entire rrt tree in vector
/// @param tree
/// @return std::vector<geometry_msgs::Point> 
std::vector<geometry_msgs::Point> RRT::full_tree(const std::vector<Node> &tree)
{
    std::vector<geometry_msgs::Point> tree_points;
    for (int i = 1; i < tree.size(); ++i)
    {
        geometry_msgs::Point one;
        one.x = tree[i].x;
        one.y = tree[i].y;
        one.z = 0.1;
        tree_points.push_back(one);
        geometry_msgs::Point two;
        two.x = tree[tree[i].parent_index].x;
        two.y = tree[tree[i].parent_index].y;
        two.z = 0.0;
        tree_points.push_back(two);
    }
    return tree_points;
}

//Creates a visualisation of all rrt tree edges
///@param tree - tree vector
///@param r - red color value
///@param g - green color value
///@param b - blue color value
///@return visualization_msgs::Marker - line list markers
void RRT::gen_tree_marker(const std::vector<Node> &tree, float r, float g, float b)
{
    std::vector<geometry_msgs::Point> tree_points = full_tree(tree);
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.ns = "current";
    marker.id = 2;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.points = tree_points;
    marker.scale.x = 0.02;
    marker.scale.y = 0;
    marker.scale.z = 0;
    marker.color.a = 0.35; // Don't forget to set the alpha!
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    tree_edge_viz_pub.publish(marker);
}

//Creates a visualisation of all rrt tree nodes
///@param tree - tree vector
///@param r - red color value
///@param g - green color value
///@param b - blue color value
//publish visualization_msgs::Marker - list sphere markers
void RRT::gen_node_marker(const std::vector<Node> &tree, float r, float g, float b)
{
    std::vector<geometry_msgs::Point> tree_points = full_tree(tree);
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.ns = "current";
    marker.id = 3;
    marker.type = visualization_msgs::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.pose.position.z = 0.1;
    marker.points = tree_points;
    marker.scale.x = 0.045;
    marker.scale.y = 0.045;
    marker.scale.z = 0.045;
    marker.color.a = 0.35; // Don't forget to set the alpha!
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    tree_node_viz_pub.publish(marker);
}

//stores the given path in vector of type geometry_msgs::Point
///@param path
///@return std::vector<geometry_msgs::Point>
std::vector<geometry_msgs::Point> RRT::in_order_path(std::vector<std::array<double,2>> path)
{
    std::vector<geometry_msgs::Point> path_points;
    for (int i = 0; i < path.size(); ++i)
    {
        geometry_msgs::Point curr;
        curr.x = path[i][0];
        curr.y = path[i][1];
        curr.z = 0.15;
        path_points.push_back(curr);
    }
    return path_points;
}

//Generates line_strip visualization markers for rrt solution  
///@param path 
///publish visualization_msgs::Marker
void RRT::gen_path_marker(const std::vector<std::array<double,2>> &path)
{
    std::vector<geometry_msgs::Point> tree_points = in_order_path(path);
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.ns = "current";
    marker.id = 8;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.points = tree_points;
    marker.scale.x = 0.06;
    marker.scale.y = 0.06;
    marker.scale.z = 0;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.5;
    marker.pose.position.z = -0.01;
    rrt_solution_pub.publish(marker);
}

//Used to vizualize the current local_track_point
///@param local_track_point
///@return
void RRT::viz_point(std::array<double, 2> point)
{

    visualization_msgs::Marker point_msg;
    point_msg.header.frame_id = "map";
    point_msg.type = visualization_msgs::Marker::SPHERE;
    point_msg.pose.orientation.w = 1.0;
    point_msg.header.stamp = ros::Time::now();

    point_msg.pose.position.x = point[0];
    point_msg.pose.position.y = point[1];
    point_msg.pose.position.z = 0.0;
    point_msg.pose.orientation.w = 1.0;

    point_msg.scale.x = point_msg.scale.y = point_msg.scale.z = 0.1;
    
    point_msg.color.a = 1.0;
    point_msg.color.r = 0.0;
    point_msg.color.b = 1.0;
    point_msg.color.g = 0.0;
        
    local_trackpoint_pub.publish(point_msg);
        
}

/// Returns the appropriate speed based on the steering angle
/// @param steering_angle
/// @return
void RRT::publish_corrected_speed_and_steering(double angle)
{
    ackermann_msgs::AckermannDriveStamped ackermann_drive_result;
    ackermann_drive_result.header.stamp = ros::Time::now();
    //ackermann_drive_result.frame_id = "base_link";

    ackermann_drive_result.drive.steering_angle = angle;
    /*
    if(steering_angle > 0.1)
    {
        if (steering_angle > 0.2)
        {
            drive_msg.drive.speed = low_speed;
            if (steering_angle > 0.4)
            {
                drive_msg.drive.steering_angle = 0.4;
            }
        }
        else
        {
            drive_msg.drive.speed = medium_speed;
        }
    }
    else if(steering_angle < -0.1)
    {
        if (steering_angle < -0.2)
        {
            drive_msg.drive.speed = low_speed;
            if (steering_angle < -0.4)
            {
                drive_msg.drive.steering_angle = -0.4;
            }
        }
        else
        {
            drive_msg.drive.speed = medium_speed;
        }
    }
    else
    {
        drive_msg.drive.speed = high_speed;
    }*/
     if (std::abs(angle) > 20.0 / 180.0 * PI) {
            ackermann_drive_result.drive.speed = 0.5;  // 5.0
        } else if (std::abs(angle) > 10.0 / 180.0 * PI) {
            ackermann_drive_result.drive.speed = 1.0;
        } else {
            ackermann_drive_result.drive.speed = 1.5;
        }

    drive_pub.publish(ackermann_drive_result);
}

