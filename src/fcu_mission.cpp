#include <ros/ros.h>
#include <stdio.h>
#include <string>
#include <iostream>
#include <geometry_msgs/InertiaStamped.h>
#include <std_msgs/Int16.h>

static char buf[16] = {0};
static bool enable_track=false;
static uint8_t enable_pos=0;

void cmdHandler(const std_msgs::Int16::ConstPtr& cmd){
  switch(cmd->data){
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
void execute_mission_001(void){
  //demo圆形轨迹
  if(enable_track){
    theta+=M_PI/20/10;
    px=0.8*cosf(theta)+1.2;
    py=0.8*sinf(theta)-1.6;
  }else{
      switch(enable_pos){
        case 1:
            px=1.0;
            py=-1.0;
            break;
        case 2:
            px=2.0;
            py=-1.0;
            break;
        case 3:
            px=2.0;
            py=-2.0;
            break;
        case 4:
            px=1.0;
            py=-2.0;
            break;
        default:
            px=2.0f;
            py=-1.6f;
            theta=0.0f;
            break;
      }
  }

  //发布mission
  mission_001.header.frame_id = "mission_001";
  mission_001.header.stamp = ros::Time::now();
  mission_001.inertia.m=yaw;
  mission_001.inertia.com.x=px;
  mission_001.inertia.com.y=py;
  mission_001.inertia.com.z=pz;
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
  mission_002.inertia.m=0.0f;
  mission_002.inertia.com.x=0.8*cosf(theta+M_PI*2/3)+1.2;
  mission_002.inertia.com.y=0.8*sinf(theta+M_PI*2/3)-1.6;
  mission_002.inertia.com.z=0.0f;
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
  mission_003.inertia.m=0.0f;
  mission_003.inertia.com.x=0.8*cosf(theta+M_PI*4/3)+1.2;
  mission_003.inertia.com.y=0.8*sinf(theta+M_PI*4/3)-1.6;
  mission_003.inertia.com.z=0.0f;
  mission_003.inertia.ixx=0.0f;
  mission_003.inertia.ixy=0.0f;
  mission_003.inertia.ixz=0.0f;
  mission_003.inertia.iyy=0.0f;
  mission_003.inertia.iyz=0.0f;
  mission_003.inertia.izz=0.0f;
  mission_pub_003.publish(mission_003);
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "fcu_mission");
  ros::NodeHandle nh;
  ros::Subscriber comm=nh.subscribe<std_msgs::Int16>("/fcu_bridge/command", 100, cmdHandler);

  mission_pub_001 = nh.advertise<geometry_msgs::InertiaStamped>("/fcu_bridge/mission_001",100);
  mission_pub_002 = nh.advertise<geometry_msgs::InertiaStamped>("/fcu_bridge/mission_002",100);
  mission_pub_003 = nh.advertise<geometry_msgs::InertiaStamped>("/fcu_bridge/mission_003",100);

  ros::Rate loop_rate(10);
  while (ros::ok()) {
    ros::spinOnce();

    execute_mission_001();
    execute_mission_002();
    execute_mission_003();

    loop_rate.sleep();
  }

  return 0;
}
