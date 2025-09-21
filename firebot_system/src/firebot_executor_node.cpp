#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/DeleteModel.h>
#include <tf/tf.h>

#include <fstream>
#include <queue>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>

ros::Publisher       cmd_pub;
ros::ServiceClient   delete_client;
ros::ServiceClient   model_client;
std::queue<std::string> actions;

std::string model_name = "fire_bot";
std::vector<std::string> fire_models = {
    "fire_marker1", "fire_marker2", "fire_marker3",
};

/* --- Tunable parameters (loaded in main) -------------------- */
double delete_radius_cm;      // default 75 cm
double k_lin;                 // linear speed (m/s)
double k_ang;                 // angular P gain (rad/s per rad error)

double current_x = 0.0, current_y = 0.0, current_yaw = 0.0;
bool   pose_received = false;

/* ------------------------------------------------------------ */

bool updateCurrentPose()
{
    gazebo_msgs::GetModelState srv;
    srv.request.model_name = model_name;

    if (model_client.call(srv) && srv.response.success)
    {
        current_x   = srv.response.pose.position.x;
        current_y   = srv.response.pose.position.y;
        current_yaw = tf::getYaw(srv.response.pose.orientation);
        pose_received = true;
        return true;
    }
    ROS_ERROR_THROTTLE(2.0, "firebot_executor: cannot get model state");
    return false;
}

void loadActionsFromFile(const std::string& file)
{
    std::ifstream in(file);
    std::string line;
    while (getline(in, line))
        if (!line.empty()) actions.push(line);
}

/* ---------- basic motion helpers --------------------------- */

void rotateTo(double target_yaw)
{
    geometry_msgs::Twist twist;
    ros::Rate rate(10);
    const double tol = 0.05;  // rad

    while (ros::ok())
    {
        if (!updateCurrentPose()) continue;

        double err = target_yaw - current_yaw;
        while (err >  M_PI) err -= 2*M_PI;
        while (err < -M_PI) err += 2*M_PI;

        if (fabs(err) < tol) break;

        twist.angular.z = k_ang * err;
        cmd_pub.publish(twist);
        rate.sleep();
    }
    twist.angular.z = 0;
    cmd_pub.publish(twist);
}

void moveToTarget(double tx, double ty)
{
    geometry_msgs::Twist twist;
    ros::Rate rate(10);
    const double tol = 0.025;   // m

    while (ros::ok())
    {
        if (!updateCurrentPose()) continue;

        double dx = tx - current_x;
        double dy = ty - current_y;
        double dist = hypot(dx, dy);
        if (dist < tol) break;

        double t_yaw = atan2(dy, dx);
        double yaw_err = t_yaw - current_yaw;
        while (yaw_err >  M_PI) yaw_err -= 2*M_PI;
        while (yaw_err < -M_PI) yaw_err += 2*M_PI;

        twist.linear.x  = k_lin;
        twist.angular.z = k_ang * yaw_err;
        cmd_pub.publish(twist);
        rate.sleep();
    }
    twist.linear.x = twist.angular.z = 0;
    cmd_pub.publish(twist);
}

/* ---------- extinguish helper --------------------------------*/

bool attemptExtinguish()
{
    /* Build a list of {distance, model_name} and sort by distance */
    struct Candidate { double dist; std::string name; };
    std::vector<Candidate> candidates;

    for (const auto& m : fire_models)
    {
        gazebo_msgs::GetModelState srv;
        srv.request.model_name = m;
        if (!model_client.call(srv) || !srv.response.success) continue;

        double fx = srv.response.pose.position.x;
        double fy = srv.response.pose.position.y;
        double d  = hypot(current_x - fx, current_y - fy) * 100.0;   // cm
        candidates.push_back({d, m});
    }
    if (candidates.empty())  return false;

    std::sort(candidates.begin(), candidates.end(),
              [](const Candidate& a, const Candidate& b){ return a.dist < b.dist; });

    const Candidate& best = candidates.front();
    if (best.dist > delete_radius_cm)
    {
        ROS_WARN("Closest fire (%.1f cm) is outside delete radius (%.1f cm)",
                 best.dist, delete_radius_cm);
        return false;
    }

    gazebo_msgs::DeleteModel del;
    del.request.model_name = best.name;
    if (delete_client.call(del) && del.response.success)
    {
        ROS_INFO("Extinguished %s at %.1f cm", best.name.c_str(), best.dist);
        fire_models.erase(std::remove(fire_models.begin(), fire_models.end(), best.name),
                          fire_models.end());
        return true;
    }
    ROS_WARN("Delete service call failed for %s", best.name.c_str());
    return false;
}

/* ---------- action dispatcher --------------------------------*/

void executeAction(const std::string& act)
{
    ROS_INFO_STREAM("Executing: " << act);
    if (!updateCurrentPose()) return;

    /* Snap to cell centre */
    int gx = static_cast<int>(std::round(current_x - 0.5));
    int gy = static_cast<int>(std::round(current_y - 0.5));
    double tx = gx + 0.5;
    double ty = gy + 0.5;

    if (act == "extinguish")
    {
        attemptExtinguish();
        return;
    }
    if (act == "move_up")        { rotateTo( M_PI/2); ty += 1.0; }
    else if (act == "move_down") { rotateTo(-M_PI/2); ty -= 1.0; }
    else if (act == "move_left") { rotateTo( M_PI   ); tx -= 1.0; }
    else if (act == "move_right"){ rotateTo( 0      ); tx += 1.0; }
    else if (act == "refill")    { /* no robot motion, but keep it for completeness */ return; }
    else {
        ROS_WARN_STREAM("Unknown action: " << act);
        return;
    }
    moveToTarget(tx, ty);
}

/* =======================  main  ============================== */

int main(int argc, char** argv)
{
    ros::init(argc, argv, "firebot_executor_node");
    ros::NodeHandle nh, pnh("~");      // pnh = private namespace

    /* load params or defaults */
    pnh.param("delete_radius_cm", delete_radius_cm, 75.0);
    pnh.param("k_lin",            k_lin,            0.25);
    pnh.param("k_ang",            k_ang,            1.0);

    cmd_pub = nh.advertise<geometry_msgs::Twist>("/fire_bot/cmd_vel", 10);
    model_client  = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
    delete_client = nh.serviceClient<gazebo_msgs::DeleteModel>("/gazebo/delete_model");

    loadActionsFromFile("/home/admin/catkin_ws/src/Assigment_2/firebot_system/robot_movements.txt");

    ros::Rate loop(1);
    while (!pose_received && ros::ok()) { updateCurrentPose(); loop.sleep(); }

    while (!actions.empty() && ros::ok())
    {
        std::string act = actions.front(); actions.pop();
        executeAction(act);
        loop.sleep();
    }
    ROS_INFO("Fire-bot plan complete.");
    return 0;
}
