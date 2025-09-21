#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <gazebo_msgs/GetModelState.h>
#include <tf/tf.h>
#include <firebot_system/Sonars.h>
#include <fstream>
#include <vector>
#include <string>
#include <queue>
#include <cmath>

constexpr int ROWS = 12;
constexpr int COLS = 12;

enum Cell { EMPTY = 0, WALL = 1, FIRE = 2 };

std::vector<std::vector<int>> grid = {
    {1,1,1,1,1,1,1,1,1,1,1,1},
    {1,0,0,0,0,0,0,0,0,0,0,1},
    {1,0,0,0,0,0,0,0,0,0,0,1},
    {1,0,0,1,1,1,1,1,1,0,0,1},
    {1,0,0,1,0,0,0,0,1,0,0,1},
    {1,0,0,1,0,0,0,0,1,0,0,1},
    {1,0,0,1,0,0,0,0,1,0,0,1},
    {1,0,0,1,0,0,0,0,1,0,0,1},
    {1,0,0,1,1,0,0,0,1,0,0,1},
    {1,0,0,0,0,0,0,0,0,0,0,1},
    {1,0,0,0,0,0,0,0,0,0,0,1},
    {1,1,1,1,1,0,0,1,1,1,1,1}
};

/* ───────────────────────────── ROS handles ───────────────────────────── */
ros::Publisher      cmd_pub;
ros::ServiceClient  model_client;
ros::Subscriber     sonar_sub;

double current_x = 0.0, current_y = 0.0, current_yaw = 0.0;
bool   fire_detected = false;
constexpr uint16_t FIRE_THRESHOLD_CM = 100;

void sonarCallback(const firebot_system::Sonars::ConstPtr &msg)
{
    if (msg->distance0 < FIRE_THRESHOLD_CM ||
        msg->distance1 < FIRE_THRESHOLD_CM ||
        msg->distance2 < FIRE_THRESHOLD_CM)
        fire_detected = true;
}

/* ─────────────────────── movement command list ─────────────────────── */
std::vector<std::string> moves;
inline void pushMove(const char *cmd) { moves.emplace_back(cmd); }

/* ──────────────────────── direction utilities ──────────────────────── */
struct Dir { int dx, dy; const char *cmd; const char *back; };
const Dir DIRS[4] = {
    {-1, 0, "move_up",    "move_down" },
    { 0, 1, "move_right", "move_left" },
    { 1, 0, "move_down",  "move_up"   },
    { 0,-1, "move_left",  "move_right"}
};

/* ───────────────────────── DFS coverage routine ─────────────────────── */
std::vector<std::vector<bool>> visited(ROWS, std::vector<bool>(COLS,false));

void dfs(int x,int y)
{
    visited[x][y]=true;
    for(const auto &d:DIRS)
    {
        int nx=x+d.dx, ny=y+d.dy;
        if(nx<0||nx>=ROWS||ny<0||ny>=COLS)          continue;
        if(grid[nx][ny]!=EMPTY||visited[nx][ny])    continue;
        pushMove(d.cmd);
        dfs(nx,ny);
        pushMove(d.back);
    }
}

/* ───────────────────────────── plan route ───────────────────────────── */
void planRoute()
{
    /* temporarily mark other robot as wall (row 11, col 6) */
    grid[11][6]=WALL;

    /* three initial moves UP -> into maze */
    pushMove("move_up");
    pushMove("move_up");
    pushMove("move_up");

    /* DFS from (9,5) (after 3 ups) */
    dfs(9,5);

    /* restore */
    grid[11][6]=EMPTY;
}

/* ───────────────────── file save helpers ───────────────────── */
const std::string BASE_PATH = "/home/admin/catkin_ws/src/Assigment_2/firebot_system/";

void saveMoves()
{
    std::ofstream f(BASE_PATH+"scout_path.txt");
    for(const auto &m:moves) f<<m<<'\n';
    ROS_INFO("[scout_node] %lu moves written to %sscout_path.txt",
             moves.size(), BASE_PATH.c_str());
}

void saveGrid()
{
    std::ofstream g(BASE_PATH+"full_explored_map.txt");
    for(const auto &row:grid){
        for(size_t i=0;i<row.size();++i){
            g<<row[i];
            if(i+1<row.size()) g<<',';
        }
        g<<'\n';
    }
    ROS_INFO("[scout_node] grid saved to %sfull_explored_map.txt",
             BASE_PATH.c_str());
}

/* ───────────────────── pose update helper ───────────────────── */
bool updatePose()
{
    gazebo_msgs::GetModelState srv;
    srv.request.model_name="scout_bot";
    if(!model_client.call(srv)||!srv.response.success) return false;
    current_x=srv.response.pose.position.x;
    current_y=srv.response.pose.position.y;
    current_yaw=tf::getYaw(srv.response.pose.orientation);
    return true;
}

/* ───────────────────── motion primitives ───────────────────── */
void rotateTo(double yaw)
{
    geometry_msgs::Twist t; ros::Rate r(15);
    const double tol=0.04;
    while(ros::ok()){
        if(!updatePose()) continue;
        double err=yaw-current_yaw;
        while(err> M_PI) err-=2*M_PI;
        while(err<-M_PI) err+=2*M_PI;
        if(fabs(err)<tol) break;
        t.angular.z=1.8*err;
        cmd_pub.publish(t);
        r.sleep();
    }
    t.angular.z=0; cmd_pub.publish(t);
}

void driveTo(double tx,double ty)
{
    geometry_msgs::Twist t; ros::Rate r(15);
    const double tol=0.03;
    while(ros::ok()){
        if(!updatePose()) continue;
        double dx=tx-current_x, dy=ty-current_y;
        if(hypot(dx,dy)<tol) break;
        double yaw_t=atan2(dy,dx);
        double err=yaw_t-current_yaw;
        while(err> M_PI) err-=2*M_PI;
        while(err<-M_PI) err+=2*M_PI;
        t.linear.x=0.22;
        t.angular.z=1.8*err;
        cmd_pub.publish(t);
        r.sleep();
    }
    t.linear.x=t.angular.z=0; cmd_pub.publish(t);
}

/* ────────────────────── execute recorded moves ────────────────────── */
void executeMoves()
{
    ros::Rate loop(4);
    for(const auto &m:moves)
    {
        if(m=="move_up")        { rotateTo(M_PI/2);  driveTo(current_x,   current_y+1);}
        else if(m=="move_down") { rotateTo(-M_PI/2); driveTo(current_x,   current_y-1);}
        else if(m=="move_left") { rotateTo(M_PI);    driveTo(current_x-1, current_y  );}
        else if(m=="move_right"){ rotateTo(0);       driveTo(current_x+1, current_y  );}

        ros::spinOnce();
        int gx=static_cast<int>(std::round(current_x-0.5));
        int gy=static_cast<int>(std::round(current_y-0.5));
        if(fire_detected && gx>=0 && gx<ROWS && gy>=0 && gy<COLS){
            grid[gx][gy]=FIRE;
            fire_detected=false;
            ROS_WARN("🔥  Fire marked at (%d,%d)",gx,gy);
        }
        loop.sleep();
    }
}

/* ───────────────────────────── main ───────────────────────────── */
int main(int argc,char **argv)
{
    ros::init(argc,argv,"scout_node");
    ros::NodeHandle nh;

    cmd_pub      = nh.advertise<geometry_msgs::Twist>("/scout_bot/cmd_vel",10);
    model_client = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
    sonar_sub    = nh.subscribe("/sonars",10,sonarCallback);

    /* 1) plan, 2) save path file */
    planRoute();
    saveMoves();

    /* wait for first valid pose */
    ros::Rate wait(2);
    while(ros::ok() && !updatePose()) wait.sleep();

    /* execute */
    executeMoves();
    saveGrid();

    ROS_INFO("[scout_node] Exploration complete.");
    return 0;
}
