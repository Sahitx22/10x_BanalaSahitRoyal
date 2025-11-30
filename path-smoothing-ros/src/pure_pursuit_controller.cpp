#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <path_smoothing_ros/Trajectory.h>

double lookahead_dist = 0.35;       // tuned for TB3 Burger
double max_lin_vel = 0.22;          // TB3 max speed
double max_ang_vel = 1.8;           // TB3 safe turn speed
double goal_tolerance = 0.15;

ros::Publisher cmd_pub;

path_smoothing_ros::Trajectory traj;
bool traj_received = false;

double robot_x = 0, robot_y = 0, robot_yaw = 0;
bool odom_received = false;

// ---------- ODOM CALLBACK ----------
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    robot_x = msg->pose.pose.position.x;
    robot_y = msg->pose.pose.position.y;

    robot_yaw = tf::getYaw(msg->pose.pose.orientation);

    odom_received = true;
}

// ----------- TRAJECTORY CALLBACK --------
void trajCallback(const path_smoothing_ros::Trajectory::ConstPtr& msg)
{
    traj = *msg;
    traj_received = true;
    ROS_INFO("[PP] Trajectory received with %lu points", traj.points.size());
}

// ---------- Pure Pursuit Controller ----------
void controlLoop()
{
    if (!traj_received || !odom_received) return;
    if (traj.points.empty()) return;

    // 1. Find the closest trajectory point
    double min_dist = 999;
    int closest_idx = 0;

    for (size_t i = 0; i < traj.points.size(); i++)
    {
        double dx = traj.points[i].x - robot_x;
        double dy = traj.points[i].y - robot_y;
        double d = hypot(dx, dy);

        if (d < min_dist)
        {
            min_dist = d;
            closest_idx = i;
        }
    }

    // 2. Goal reached?
    auto& goal = traj.points.back();
    if (hypot(goal.x - robot_x, goal.y - robot_y) < goal_tolerance)
    {
        geometry_msgs::Twist stop;
        cmd_pub.publish(stop);
        ROS_INFO_THROTTLE(1.0, "[PP] Goal reached!");
        return;
    }

    // 3. Lookahead point
    int target_idx = closest_idx;
    double dist_acc = 0.0;

    for (size_t i = closest_idx; i < traj.points.size() - 1; i++)
    {
        double dx = traj.points[i+1].x - traj.points[i].x;
        double dy = traj.points[i+1].y - traj.points[i].y;
        dist_acc += hypot(dx, dy);

        if (dist_acc >= lookahead_dist)
        {
            target_idx = i+1;
            break;
        }
    }

    auto& target = traj.points[target_idx];

    // 4. Transform target into robot frame
    double dx = target.x - robot_x;
    double dy = target.y - robot_y;

    double target_x_r =  cos(robot_yaw)*dx + sin(robot_yaw)*dy;
    double target_y_r = -sin(robot_yaw)*dx + cos(robot_yaw)*dy;

    // 5. Compute curvature
    double curvature = 2.0 * target_y_r / (lookahead_dist * lookahead_dist);

    // 6. Command velocities
    geometry_msgs::Twist cmd;

    // Slow down on sharp curves
    double speed_scale = std::max(0.3, 1.0 - fabs(curvature));

    cmd.linear.x  = max_lin_vel * speed_scale;
    cmd.angular.z = std::max(-max_ang_vel,
                        std::min(max_ang_vel, curvature * cmd.linear.x));

    cmd_pub.publish(cmd);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pure_pursuit_controller");
    ros::NodeHandle nh;

    // Publisher
    cmd_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    // Subscribers
    ros::Subscriber odom_sub = nh.subscribe("/odom", 1, odomCallback);
    ros::Subscriber traj_sub = nh.subscribe("/trajectory", 1, trajCallback);

    ROS_INFO("Pure Pursuit controller started.");

    ros::Rate rate(30);  // 30 Hz
    while (ros::ok())
    {
        controlLoop();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
