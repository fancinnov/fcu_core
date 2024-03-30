#include <ros/ros.h>
#include <stdio.h>
#include <string>
#include <iostream>
#include <geometry_msgs/InertiaStamped.h>
#include <std_msgs/Int16.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

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

static float px1=0.0f, py1=0.0f, px2=0.0f, py2=0.0f, px3=0.0f, py3=0.0f;
void execute_mission_001(void){
  //demo圆形轨迹
  if(enable_track){
    theta+=M_PI/20/10;
    px=1.0*cosf(theta)+2;
    py=1.0*sinf(theta)+2;
  }else{
      switch(enable_pos){
        case 1:
            px=1.0;
            py=1.0;

            px1=3.0f;
            px2=2.0f;
            px3=1.0f;
            py1=2.0f;
            py2=2.0f;
            py3=2.0f;

            break;
        case 2:
            px=2.0;
            py=1.0;

            px1=3.0f;
            px2=2.0f;
            px3=1.0f;
            py1=1.0f;
            py2=2.0f;
            py3=3.0f;

            break;
        case 3:
            px=2.0;
            py=2.0;

            px1=3.0f;
            px2=2.0f;
            px3=1.0f;
            py1=3.0f;
            py2=1.0f;
            py3=3.0f;

            break;
        case 4:
            px=1.0;
            py=2.0;

            px1=3.0f;
            px2=2.0f;
            px3=1.0f;
            py1=1.0f;
            py2=3.0f;
            py3=1.0f;

            break;
        default:
            px=3.0f;
            py=2.0f;
            theta=0.0f;

            px1=3.0f;
            px2=2.0f;
            px3=1.0f;
            py1=2.0f;
            py2=2.0f;
            py3=2.0f;

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
  mission_002.inertia.com.x=1.0*cosf(theta+M_PI*1/3)+2;
  mission_002.inertia.com.y=1.0*sinf(theta+M_PI*1/3)+2;
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
  mission_003.inertia.com.x=1.0*cosf(theta+M_PI*2/3)+2;
  mission_003.inertia.com.y=1.0*sinf(theta+M_PI*2/3)+2;
  mission_003.inertia.com.z=0.0f;
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
  mission_004.inertia.m=0.0f;
  mission_004.inertia.com.x=1.0*cosf(theta+M_PI)+2;
  mission_004.inertia.com.y=1.0*sinf(theta+M_PI)+2;
  mission_004.inertia.com.z=0.0f;
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
  //发布mission_002
  mission_005.header.frame_id = "mission_005";
  mission_005.header.stamp = ros::Time::now();
  mission_005.inertia.m=0.0f;
  mission_005.inertia.com.x=1.0*cosf(theta+M_PI*4/3)+2;
  mission_005.inertia.com.y=1.0*sinf(theta+M_PI*4/3)+2;
  mission_005.inertia.com.z=0.0f;
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
  mission_006.inertia.m=0.0f;
  mission_006.inertia.com.x=1.0*cosf(theta+M_PI*5/3)+2;
  mission_006.inertia.com.y=1.0*sinf(theta+M_PI*5/3)+2;
  mission_006.inertia.com.z=0.0f;
  mission_006.inertia.ixx=0.0f;
  mission_006.inertia.ixy=0.0f;
  mission_006.inertia.ixz=0.0f;
  mission_006.inertia.iyy=0.0f;
  mission_006.inertia.iyz=0.0f;
  mission_006.inertia.izz=0.0f;
  mission_pub_006.publish(mission_006);
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "fcu_mission");
  ros::NodeHandle nh;
  ros::Subscriber comm=nh.subscribe<std_msgs::Int16>("/fcu_bridge/command", 100, cmdHandler);

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
    execute_mission_001();
    execute_mission_002();
    execute_mission_003();
    execute_mission_004();
    execute_mission_005();
    execute_mission_006();

    loop_rate.sleep();
  }

  return 0;
}
