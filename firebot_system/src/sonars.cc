#include <ros/ros.h>
#include <gazebo_msgs/GetModelState.h>
#include <firebot_system/Sonars.h>
#include <tf/tf.h>
#include <cmath>
#include <cstdint>
#include <vector>
#include <string>
#include <algorithm>
#include <unistd.h>

using firebot_system::Sonars;

/* ---------- helper ------------------------------------------------------- */
Sonars makeMsg(uint16_t d0, uint16_t d1, uint16_t d2)
{
    Sonars m;
    m.distance0 = d0;
    m.distance1 = d1;
    m.distance2 = d2;
    m.distance3 = m.distance4 = m.distance5 = UINT16_MAX;   // unused beams
    return m;
}

bool angleInCone(double beamDeg, double objDeg, double halfWidthDeg = 15.0)
{
    double diff = objDeg - beamDeg;
    while (diff > 180.0) diff -= 360.0;
    while (diff < -180.0) diff += 360.0;
    return std::fabs(diff) <= halfWidthDeg;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sonars");

    /* ----- CLI option: robot name --------------------------------------- */
    std::string robotName = "scout_bot";
    char c;
    while ((c = getopt(argc, argv, "r:h")) != -1)
    {
        if (c == 'r') robotName = std::string(optarg);
        if (c == 'h')
        {
            std::cout << "Usage: rosrun firebot_system sonars  [-r <robot_name>]\n";
            return 0;
        }
    }

    /* ----- ROS setup ---------------------------------------------------- */
    ros::NodeHandle nh;
    ros::Publisher  pub = nh.advertise<Sonars>("sonars", 10);
    ros::Rate       loop(10);                        // 10 Hz publish

    ros::ServiceClient get =
        nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");

    /* Models the beams can hit: only those actually in the world ---------- */
    const std::vector<std::string> models = {
        "fire_marker1", "fire_marker2", "fire_marker3", "fire_marker5"
        // Removed: "fire_marker4", "house_outer", "inner_room"
    };

    gazebo_msgs::GetModelState srv;
    srv.request.relative_entity_name = "";     // world frame

    while (ros::ok())
    {
        ros::spinOnce();

        /* ----- robot pose ---------------------------------------------- */
        srv.request.model_name = robotName;
        if (!get.call(srv) || !srv.response.success)
        {
            ROS_ERROR_THROTTLE(2.0, "sonars: cannot find robot model %s",
                               robotName.c_str());
            loop.sleep(); continue;
        }
        double rx = srv.response.pose.position.x;
        double ry = srv.response.pose.position.y;
        double ryaw = tf::getYaw(srv.response.pose.orientation);       // rad
        double ryawDeg = ryaw * 180.0 / M_PI;

        /* ----- initialise beam ranges (cm) ------------------------------ */
        double beamLeftDeg   =  30.0 + ryawDeg;
        double beamFrontDeg  =   0.0 + ryawDeg;
        double beamRightDeg  = -30.0 + ryawDeg;
        double d0 = 5e4, d1 = 5e4, d2 = 5e4;           // default 50 m

        /* ----- query every relevant model ------------------------------- */
        for (const auto& name : models)
        {
            srv.request.model_name = name;
            if (!get.call(srv) || !srv.response.success) continue;

            double cx = srv.response.pose.position.x;
            double cy = srv.response.pose.position.y;

            double dx = cx - rx;
            double dy = cy - ry;
            double dist_cm  = std::hypot(dx, dy) * 100.0;       // m → cm
            double angleDeg = std::atan2(dy, dx) * 180.0 / M_PI;

            if      (angleInCone(beamLeftDeg,  angleDeg)) d0 = std::min(d0, dist_cm);
            else if (angleInCone(beamFrontDeg, angleDeg)) d1 = std::min(d1, dist_cm);
            else if (angleInCone(beamRightDeg, angleDeg)) d2 = std::min(d2, dist_cm);
        }

        auto toU16 = [](double cm){
            if (cm > 5000.0) return static_cast<uint16_t>(UINT16_MAX);
            return static_cast<uint16_t>(std::max(0.0, std::min(cm, 65535.0)));
        };

        pub.publish(makeMsg(toU16(d0), toU16(d1), toU16(d2)));
        loop.sleep();
    }
    return 0;
}
