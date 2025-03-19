#include <ros/ros.h>
#include <stdio.h>
#include <string>
#include <iostream>
#include <geometry_msgs/InertiaStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int16.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include "quadrotor_msgs/PositionCommand.h"

static char buf[16] = {0};
static bool enable_track=false;
static uint8_t enable_pos=0;
float pos_odom_001_x=0.0f; float pos_odom_001_y=0.0f; float pos_odom_001_z=0.0f;
float pos_odom_002_x=0.0f; float pos_odom_002_y=0.0f; float pos_odom_002_z=0.0f;
float pos_odom_003_x=0.0f; float pos_odom_003_y=0.0f; float pos_odom_003_z=0.0f;
float pos_odom_004_x=0.0f; float pos_odom_004_y=0.0f; float pos_odom_004_z=0.0f;
float pos_odom_005_x=0.0f; float pos_odom_005_y=0.0f; float pos_odom_005_z=0.0f;
float pos_odom_006_x=0.0f; float pos_odom_006_y=0.0f; float pos_odom_006_z=0.0f;

float pos_takeoff_001_x=0.0f; float pos_takeoff_001_y=0.0f; float pos_takeoff_001_z=0.0f;
float pos_takeoff_002_x=0.0f; float pos_takeoff_002_y=0.0f; float pos_takeoff_002_z=0.0f;
float pos_takeoff_003_x=0.0f; float pos_takeoff_003_y=0.0f; float pos_takeoff_003_z=0.0f;
float pos_takeoff_004_x=0.0f; float pos_takeoff_004_y=0.0f; float pos_takeoff_004_z=0.0f;
float pos_takeoff_005_x=0.0f; float pos_takeoff_005_y=0.0f; float pos_takeoff_005_z=0.0f;
float pos_takeoff_006_x=0.0f; float pos_takeoff_006_y=0.0f; float pos_takeoff_006_z=0.0f;

void cmdHandler(const std_msgs::Int16::ConstPtr& cmd){
  switch(cmd->data){
    case 3://起飞时刻记录坐标,原地悬停
        pos_takeoff_001_x=pos_odom_001_x; pos_takeoff_001_y=pos_odom_001_y; pos_takeoff_001_z=pos_odom_001_z;
        pos_takeoff_002_x=pos_odom_002_x; pos_takeoff_002_y=pos_odom_002_y; pos_takeoff_002_z=pos_odom_002_z;
        pos_takeoff_003_x=pos_odom_003_x; pos_takeoff_003_y=pos_odom_003_y; pos_takeoff_003_z=pos_odom_003_z;
        pos_takeoff_004_x=pos_odom_004_x; pos_takeoff_004_y=pos_odom_004_y; pos_takeoff_004_z=pos_odom_004_z;
        pos_takeoff_005_x=pos_odom_005_x; pos_takeoff_005_y=pos_odom_005_y; pos_takeoff_005_z=pos_odom_005_z;
        pos_takeoff_006_x=pos_odom_006_x; pos_takeoff_006_y=pos_odom_006_y; pos_takeoff_006_z=pos_odom_006_z;
        break;
    case 5:
        enable_track=true;
        break;
    case 6:
        enable_track=false;
        break;
    case 7:
        enable_pos=1;
        break;
    case 8:
        enable_pos=2;
        break;
    case 9:
        enable_pos=3;
        break;
    case 10:
        enable_pos=4;
        break;
    default:
        break;
  }
}

static geometry_msgs::InertiaStamped mission_001;
static ros::Publisher mission_pub_001;
static float yaw=0.0f, yaw_rate=0.0f;
static float px=0.0f, py=0.0f, pz=0.0f;
static float vx=0.0f, vy=0.0f, vz=0.0f;
static float ax=0.0f, ay=0.0f, az=0.0f;
static float theta=0.0f;

static float  px1=0.0f, py1=0.0f, pz1=0.0f,
              px2=0.0f, py2=0.0f, pz2=0.0f,
              px3=0.0f, py3=0.0f, pz3=0.0f,
              px4=0.0f, py4=0.0f, pz4=0.0f,
              px5=0.0f, py5=0.0f, pz5=0.0f,
              px6=0.0f, py6=0.0f, pz6=0.0f;
void execute_mission_001(void){
  //demo圆形轨迹
  if(enable_track){
    theta+=M_PI/20/10;
    px=1.0*cosf(theta)+2;
    py=1.0*sinf(theta)+2;

    px1=px;
    py1=py;
    pz1=0.6;

    px2=1.0*cosf(theta+M_PI*2/6)+2;
    py2=1.0*sinf(theta+M_PI*2/6)+2;

    px3=1.0*cosf(theta+M_PI*4/6)+2;
    py3=1.0*sinf(theta+M_PI*4/6)+2;

    px4=1.0*cosf(theta+M_PI*6/6)+2;
    py4=1.0*sinf(theta+M_PI*6/6)+2;

    px5=1.0*cosf(theta+M_PI*8/6)+2;
    py5=1.0*sinf(theta+M_PI*8/6)+2;

    px6=1.0*cosf(theta+M_PI*10/6)+2;
    py6=1.0*sinf(theta+M_PI*10/6)+2;
  }else{
      switch(enable_pos){
        case 1:
            px=0.0;
            py=0.0;
            pz=1.0;

            px1=pos_takeoff_001_x+px; py1=pos_takeoff_001_y+py; pz1=pz;
            px2=pos_takeoff_002_x+px; py2=pos_takeoff_002_y+py; pz2=pz;
            px3=pos_takeoff_003_x+px; py3=pos_takeoff_003_y+py; pz3=pz;
            px4=pos_takeoff_004_x+px; py4=pos_takeoff_004_y+py; pz4=pz;
            px5=pos_takeoff_005_x+px; py5=pos_takeoff_005_y+py; pz5=pz;
            px6=pos_takeoff_006_x+px; py6=pos_takeoff_006_y+py; pz6=pz;

            break;
        case 2:
            px=0.5;
            py=0.0;
            pz=1.0;

            px1=pos_takeoff_001_x+px; py1=pos_takeoff_001_y+py; pz1=pz;
            px2=pos_takeoff_002_x+px; py2=pos_takeoff_002_y+py; pz2=pz;
            px3=pos_takeoff_003_x+px; py3=pos_takeoff_003_y+py; pz3=pz;
            px4=pos_takeoff_004_x+px; py4=pos_takeoff_004_y+py; pz4=pz;
            px5=pos_takeoff_005_x+px; py5=pos_takeoff_005_y+py; pz5=pz;
            px6=pos_takeoff_006_x+px; py6=pos_takeoff_006_y+py; pz6=pz;

            break;
        case 3:
            px=2.0;
            py=2.0;
            pz=1.0;

            px1=pos_takeoff_001_x+px; py1=pos_takeoff_001_y+py; pz1=pz;
            px2=pos_takeoff_002_x+px; py2=pos_takeoff_002_y+py; pz2=pz;
            px3=pos_takeoff_003_x+px; py3=pos_takeoff_003_y+py; pz3=pz;
            px4=pos_takeoff_004_x+px; py4=pos_takeoff_004_y+py; pz4=pz;
            px5=pos_takeoff_005_x+px; py5=pos_takeoff_005_y+py; pz5=pz;
            px6=pos_takeoff_006_x+px; py6=pos_takeoff_006_y+py; pz6=pz;

            break;
        case 4:
            px=0.0;
            py=0.5;
            pz=1.0;

            px1=pos_takeoff_001_x+px; py1=pos_takeoff_001_y+py; pz1=pz;
            px2=pos_takeoff_002_x+px; py2=pos_takeoff_002_y+py; pz2=pz;
            px3=pos_takeoff_003_x+px; py3=pos_takeoff_003_y+py; pz3=pz;
            px4=pos_takeoff_004_x+px; py4=pos_takeoff_004_y+py; pz4=pz;
            px5=pos_takeoff_005_x+px; py5=pos_takeoff_005_y+py; pz5=pz;
            px6=pos_takeoff_006_x+px; py6=pos_takeoff_006_y+py; pz6=pz;

            break;
        default:
            px1=pos_takeoff_001_x; py1=pos_takeoff_001_y; pz1=0.0f;
            px2=pos_takeoff_002_x; py2=pos_takeoff_002_y; pz2=0.0f;
            px3=pos_takeoff_003_x; py3=pos_takeoff_003_y; pz3=0.0f;
            px4=pos_takeoff_004_x; py4=pos_takeoff_004_y; pz4=0.0f;
            px5=pos_takeoff_005_x; py5=pos_takeoff_005_y; pz5=0.0f;
            px6=pos_takeoff_006_x; py6=pos_takeoff_006_y; pz6=0.0f;
            break;
      }
  }

  //发布mission
  mission_001.header.frame_id = "mission_001";
  mission_001.header.stamp = ros::Time::now();
  mission_001.inertia.m=yaw;//rad
  mission_001.inertia.com.x=px1;
  mission_001.inertia.com.y=py1;
  mission_001.inertia.com.z=pz1;
  mission_001.inertia.ixx=vx;
  mission_001.inertia.ixy=vy;
  mission_001.inertia.ixz=vz;
  mission_001.inertia.iyy=ax;
  mission_001.inertia.iyz=ay;
  mission_001.inertia.izz=az;
  mission_pub_001.publish(mission_001);
}

static geometry_msgs::InertiaStamped mission_002;
static ros::Publisher mission_pub_002;
void execute_mission_002(void){
  //demo圆形轨迹
  //发布mission_002
  mission_002.header.frame_id = "mission_002";
  mission_002.header.stamp = ros::Time::now();
  mission_002.inertia.m=0.0f;//rad
  mission_002.inertia.com.x=px2;
  mission_002.inertia.com.y=py2;
  mission_002.inertia.com.z=pz2;
  mission_002.inertia.ixx=0.0f;
  mission_002.inertia.ixy=0.0f;
  mission_002.inertia.ixz=0.0f;
  mission_002.inertia.iyy=0.0f;
  mission_002.inertia.iyz=0.0f;
  mission_002.inertia.izz=0.0f;
  mission_pub_002.publish(mission_002);
}

static geometry_msgs::InertiaStamped mission_003;
static ros::Publisher mission_pub_003;
void execute_mission_003(void){
  //demo圆形轨迹
  //发布mission_003
  mission_003.header.frame_id = "mission_003";
  mission_003.header.stamp = ros::Time::now();
  mission_003.inertia.m=0.0f;//rad
  mission_003.inertia.com.x=px3;
  mission_003.inertia.com.y=py3;
  mission_003.inertia.com.z=pz3;
  mission_003.inertia.ixx=0.0f;
  mission_003.inertia.ixy=0.0f;
  mission_003.inertia.ixz=0.0f;
  mission_003.inertia.iyy=0.0f;
  mission_003.inertia.iyz=0.0f;
  mission_003.inertia.izz=0.0f;
  mission_pub_003.publish(mission_003);
}

static geometry_msgs::InertiaStamped mission_004;
static ros::Publisher mission_pub_004;
void execute_mission_004(void){
  //demo圆形轨迹
  //发布mission_004
  mission_004.header.frame_id = "mission_004";
  mission_004.header.stamp = ros::Time::now();
  mission_004.inertia.m=0.0f;//rad
  mission_004.inertia.com.x=px4;
  mission_004.inertia.com.y=py4;
  mission_004.inertia.com.z=pz4;
  mission_004.inertia.ixx=0.0f;
  mission_004.inertia.ixy=0.0f;
  mission_004.inertia.ixz=0.0f;
  mission_004.inertia.iyy=0.0f;
  mission_004.inertia.iyz=0.0f;
  mission_004.inertia.izz=0.0f;
  mission_pub_004.publish(mission_004);
}

static geometry_msgs::InertiaStamped mission_005;
static ros::Publisher mission_pub_005;
void execute_mission_005(void){
  //demo圆形轨迹
  //发布mission_005
  mission_005.header.frame_id = "mission_005";
  mission_005.header.stamp = ros::Time::now();
  mission_005.inertia.m=0.0f;//rad
  mission_005.inertia.com.x=px5;
  mission_005.inertia.com.y=py5;
  mission_005.inertia.com.z=pz5;
  mission_005.inertia.ixx=0.0f;
  mission_005.inertia.ixy=0.0f;
  mission_005.inertia.ixz=0.0f;
  mission_005.inertia.iyy=0.0f;
  mission_005.inertia.iyz=0.0f;
  mission_005.inertia.izz=0.0f;
  mission_pub_005.publish(mission_005);
}

static geometry_msgs::InertiaStamped mission_006;
static ros::Publisher mission_pub_006;
void execute_mission_006(void){
  //demo圆形轨迹
  //发布mission_006
  mission_006.header.frame_id = "mission_006";
  mission_006.header.stamp = ros::Time::now();
  mission_006.inertia.m=0.0f;//rad
  mission_006.inertia.com.x=px6;
  mission_006.inertia.com.y=py6;
  mission_006.inertia.com.z=pz6;
  mission_006.inertia.ixx=0.0f;
  mission_006.inertia.ixy=0.0f;
  mission_006.inertia.ixz=0.0f;
  mission_006.inertia.iyy=0.0f;
  mission_006.inertia.iyz=0.0f;
  mission_006.inertia.izz=0.0f;
  mission_pub_006.publish(mission_006);
}

void odom_global001_handler(const nav_msgs::Odometry::ConstPtr& odom)
{
  pos_odom_001_x=(float)odom->pose.pose.position.x;
  pos_odom_001_y=(float)odom->pose.pose.position.y;
  pos_odom_001_z=-(float)odom->pose.pose.position.z;
}

void odom_global002_handler(const nav_msgs::Odometry::ConstPtr& odom)
{
  pos_odom_002_x=(float)odom->pose.pose.position.x;
  pos_odom_002_y=(float)odom->pose.pose.position.y;
  pos_odom_002_z=-(float)odom->pose.pose.position.z;
}

void odom_global003_handler(const nav_msgs::Odometry::ConstPtr& odom)
{
  pos_odom_003_x=(float)odom->pose.pose.position.x;
  pos_odom_003_y=(float)odom->pose.pose.position.y;
  pos_odom_003_z=-(float)odom->pose.pose.position.z;
}

void odom_global004_handler(const nav_msgs::Odometry::ConstPtr& odom)
{
  pos_odom_004_x=(float)odom->pose.pose.position.x;
  pos_odom_004_y=(float)odom->pose.pose.position.y;
  pos_odom_004_z=-(float)odom->pose.pose.position.z;
}

void odom_global005_handler(const nav_msgs::Odometry::ConstPtr& odom)
{
  pos_odom_005_x=(float)odom->pose.pose.position.x;
  pos_odom_005_y=(float)odom->pose.pose.position.y;
  pos_odom_005_z=-(float)odom->pose.pose.position.z;
}

void odom_global006_handler(const nav_msgs::Odometry::ConstPtr& odom)
{
  pos_odom_006_x=(float)odom->pose.pose.position.x;
  pos_odom_006_y=(float)odom->pose.pose.position.y;
  pos_odom_006_z=-(float)odom->pose.pose.position.z;
}

static bool get_pos_cmd=false;
void pos_cmd_handler(const quadrotor_msgs::PositionCommand::ConstPtr& pose_plan)//仅机载电脑运行此函数
{
  //发布mission
  get_pos_cmd=true;
  mission_001.header.frame_id = "mission_001";
  mission_001.header.stamp = ros::Time::now();
  mission_001.inertia.m=-pose_plan->yaw;//rad
  mission_001.inertia.com.x=pose_plan->position.x;
  mission_001.inertia.com.y=-pose_plan->position.y;
  mission_001.inertia.com.z=pose_plan->position.z;
  mission_001.inertia.ixx=pose_plan->velocity.x;
  mission_001.inertia.ixy=-pose_plan->velocity.y;
  mission_001.inertia.ixz=pose_plan->velocity.z;
  mission_001.inertia.iyy=pose_plan->acceleration.x;
  mission_001.inertia.iyz=-pose_plan->acceleration.y;
  mission_001.inertia.izz=pose_plan->acceleration.z;
  mission_pub_001.publish(mission_001);
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "fcu_mission");
  ros::NodeHandle nh;
  ros::Subscriber comm=nh.subscribe<std_msgs::Int16>("/fcu_bridge/command", 100, cmdHandler);
  ros::Subscriber odom001=nh.subscribe<nav_msgs::Odometry>("odom_global_001", 100, odom_global001_handler);
  ros::Subscriber odom002=nh.subscribe<nav_msgs::Odometry>("odom_global_002", 100, odom_global002_handler);
  ros::Subscriber odom003=nh.subscribe<nav_msgs::Odometry>("odom_global_003", 100, odom_global003_handler);
  ros::Subscriber odom004=nh.subscribe<nav_msgs::Odometry>("odom_global_004", 100, odom_global004_handler);
  ros::Subscriber odom005=nh.subscribe<nav_msgs::Odometry>("odom_global_005", 100, odom_global005_handler);
  ros::Subscriber odom006=nh.subscribe<nav_msgs::Odometry>("odom_global_006", 100, odom_global006_handler);
  ros::Subscriber pos_cmd=nh.subscribe<quadrotor_msgs::PositionCommand>("planning/pos_cmd", 100, pos_cmd_handler);

  mission_pub_001 = nh.advertise<geometry_msgs::InertiaStamped>("/fcu_bridge/mission_001",100);
  mission_pub_002 = nh.advertise<geometry_msgs::InertiaStamped>("/fcu_bridge/mission_002",100);
  mission_pub_003 = nh.advertise<geometry_msgs::InertiaStamped>("/fcu_bridge/mission_003",100);
  mission_pub_004 = nh.advertise<geometry_msgs::InertiaStamped>("/fcu_bridge/mission_004",100);
  mission_pub_005 = nh.advertise<geometry_msgs::InertiaStamped>("/fcu_bridge/mission_005",100);
  mission_pub_006 = nh.advertise<geometry_msgs::InertiaStamped>("/fcu_bridge/mission_006",100);

  ros::Rate loop_rate(10);
  while (ros::ok()) {
    ros::spinOnce();
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    transform.setOrigin(tf::Vector3(0, 0,0));
    q.setRPY(M_PI, 0, 0);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform,  ros::Time::now(), "map", "uwb"));
    q.setRPY(0, 0, 0);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform,  ros::Time::now(), "map", "world"));
    if(!get_pos_cmd){
      execute_mission_001();
      execute_mission_002();
      execute_mission_003();
      execute_mission_004();
      execute_mission_005();
      execute_mission_006();
    }

    loop_rate.sleep();
  }

  return 0;
}