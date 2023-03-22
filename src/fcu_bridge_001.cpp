#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <serial/serial.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/InertiaStamped.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int16.h>
#include <cmath>
#include <string>
#include <iostream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include "fcu_bridge.h"
#include "../mavlink/common/mavlink.h"

#define DRONE_PORT 333 //port
#define BUF_SIZE 1024

static char* DRONE_IP = "192.168.1.201"; //ip
static int socket_cli;
static int get_drone;
struct sockaddr_in drone_addr;
static serial::Serial ser; //声明串口对象
static mavlink_channel_t mav_chan=MAVLINK_COMM_1;//MAVLINK_COMM_0串口发送，MAVLINK_COMM_1网口发送
static mavlink_system_t mavlink_system;
static mavlink_message_t msg_received;
static mavlink_status_t status;
static mavlink_scaled_imu_t imu;
static mavlink_global_vision_position_estimate_t pose;
static mavlink_global_position_int_t position;
static mavlink_battery_status_t batt;
static uint8_t buffer[BUF_SIZE];
static uint8_t TxBuffer[BUF_SIZE];
static uint8_t RxBuffer[BUF_SIZE];
static uint8_t TxBuffer_buf[BUF_SIZE];
static double time_start;

ros::Publisher imu_global;
ros::Publisher odom_global;
ros::Subscriber odom;
ros::Subscriber cmd;
ros::Subscriber mission;
ros::Publisher path_global;
sensor_msgs::Imu imu_pub;
nav_msgs::Odometry odom_pub;
nav_msgs::Path path_pub;
geometry_msgs::PoseStamped odomPose;

typedef struct {
	uint8_t* pBuff;
	uint8_t* pEnd;  // pBuff + legnth
	uint8_t* wp;    // Write Point
	uint8_t* rp;    // Read Point
	uint16_t length;
	uint8_t  flagOverflow; // set when buffer overflowed
} RingBuffer;
RingBuffer mav_buf_send;
RingBuffer mav_buf_receive;

//初始化RingBuffer
void rbInit(RingBuffer* pRingBuff, uint8_t* buff, uint16_t length)
{
	pRingBuff->pBuff = buff;
	pRingBuff->pEnd  = buff + length;
	pRingBuff->wp = buff;
	pRingBuff->rp = buff;
	pRingBuff->length = length;
	pRingBuff->flagOverflow = 0;
}

//清空RingBuffer
inline void rbClear(RingBuffer* pRingBuff)
{
 	pRingBuff->wp = pRingBuff->pBuff;
	pRingBuff->rp = pRingBuff->pBuff;
	pRingBuff->flagOverflow = 0;
}

//把一字节数据存进RingBuffer
inline void rbPush(RingBuffer* pRingBuff, uint8_t value)
{
	uint8_t* wp_next = pRingBuff->wp + 1;
	if( wp_next == pRingBuff->pEnd ) {
		wp_next -= pRingBuff->length; // Rewind pointer when exceeds bound
	}
	if( wp_next != pRingBuff->rp ) {
		*pRingBuff->wp = value;
		pRingBuff->wp = wp_next;
	} else {
		pRingBuff->flagOverflow = 1;
	}
}

//读取一个字节数据出来
inline uint8_t rbPop(RingBuffer* pRingBuff)
{
	if( pRingBuff->rp == pRingBuff->wp )
		return 0; // empty

	uint8_t ret = *pRingBuff->rp;
	pRingBuff->rp++;
	if( pRingBuff->rp == pRingBuff->pEnd ) {
		pRingBuff->rp -= pRingBuff->length; // Rewind pointer when exceeds bound
	}
	return ret;
}

//当前还有多少数据没被读取
inline uint16_t rbGetCount(const RingBuffer* pRingBuff)
{
	return (pRingBuff->wp - pRingBuff->rp + pRingBuff->length) % pRingBuff->length;
}

//当前RingBuffer是不是空的
inline int8_t rbIsEmpty(const RingBuffer* pRingBuff)
{
	return pRingBuff->wp == pRingBuff->rp;
}

//当前RingBuffer是不是满的
inline int8_t rbIsFull(const RingBuffer* pRingBuff)
{
 	return (pRingBuff->rp - pRingBuff->wp + pRingBuff->length - 1) % pRingBuff->length == 0;
}

void flush_data(void){
	uint16_t length=rbGetCount(&mav_buf_send);
	if(length>0){
  	  for(uint16_t i=0; i<length; i++){
  		  TxBuffer_buf[i]=rbPop(&mav_buf_send);
  	  }
      if (mav_chan == MAVLINK_COMM_0){
        ser.write(TxBuffer_buf,length);
      }else{
        send(socket_cli, TxBuffer_buf, length, 0);
      }
	}
}

void mav_send_buffer(mavlink_channel_t chan, char *buf, uint16_t len){
  for(uint16_t i=0; i<len; i++){
    rbPush(&mav_buf_send, buf[i]);
  }
}

void mavlink_send_msg(mavlink_channel_t chan, mavlink_message_t *msg)
{
	uint8_t ck[2];

	ck[0] = (uint8_t)(msg->checksum & 0xFF);
	ck[1] = (uint8_t)(msg->checksum >> 8);
	// XXX use the right sequence here

	mav_send_buffer(chan, (char *)&msg->magic, MAVLINK_NUM_HEADER_BYTES);
	mav_send_buffer(chan, (char *)&msg->payload64, msg->len);
	mav_send_buffer(chan, (char *)ck, 2);
}

//心跳
void mav_send_heartbeat(void){
  mavlink_message_t msg_heartbeat;
  mavlink_heartbeat_t heartbeat;
  heartbeat.type=MAV_TYPE_GCS;
  heartbeat.autopilot=MAV_AUTOPILOT_INVALID;
  heartbeat.base_mode=MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
  mavlink_msg_heartbeat_encode(mavlink_system.sysid, mavlink_system.compid, &msg_heartbeat, &heartbeat);
  mavlink_send_msg(mav_chan, &msg_heartbeat);
}

//解锁
void mav_send_arm(void){
  mavlink_set_mode_t setmode;
  mavlink_message_t msg_setmode;
  setmode.base_mode=MAV_MODE_AUTO_ARMED;
  mavlink_msg_set_mode_encode(mavlink_system.sysid, mavlink_system.compid, &msg_setmode, &setmode);
  mavlink_send_msg(mav_chan, &msg_setmode);
}

//锁定
void mav_send_disarm(void){
  mavlink_set_mode_t setmode;
  mavlink_message_t msg_setmode;
  setmode.base_mode=MAV_MODE_AUTO_DISARMED;
  mavlink_msg_set_mode_encode(mavlink_system.sysid, mavlink_system.compid, &msg_setmode, &setmode);
  mavlink_send_msg(mav_chan, &msg_setmode);
}

//起飞
void mav_send_takeoff(void){
  mavlink_command_long_t command_long;
  mavlink_message_t msg_command_long;
  command_long.command=MAV_CMD_NAV_TAKEOFF;
  mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
  mavlink_send_msg(mav_chan, &msg_command_long);
}

//降落
void mav_send_land(void){
  mavlink_command_long_t command_long;
  mavlink_message_t msg_command_long;
  command_long.command=MAV_CMD_NAV_LAND;
  mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
  mavlink_send_msg(mav_chan, &msg_command_long);
}

//设置目标。注意：这里输入的目标都是全局坐标系下的目标值
void mav_send_target(float target_pos_x, float target_pos_y, float target_pos_z, //单位：m
                    float target_vel_x, float target_vel_y, float target_vel_z,  //单位：m/s
                    float target_acc_x, float target_acc_y, float target_acc_z,  //单位：m/ss
                    float target_yaw, float target_yaw_rate){                    //单位：rad, rad/s
  mavlink_set_position_target_local_ned_t set_position_target_local_ned;
  mavlink_message_t msg_position_target_local_ned;
  set_position_target_local_ned.x=target_pos_x;
  set_position_target_local_ned.y=target_pos_y;
  set_position_target_local_ned.z=target_pos_z;
  set_position_target_local_ned.vx=target_vel_x;
  set_position_target_local_ned.vy=target_vel_y;
  set_position_target_local_ned.vz=target_vel_z;
  set_position_target_local_ned.afx=target_acc_x;
  set_position_target_local_ned.afy=target_acc_y;
  set_position_target_local_ned.afz=target_acc_z;
  set_position_target_local_ned.yaw=target_yaw;
  set_position_target_local_ned.yaw_rate=target_yaw_rate;
  mavlink_msg_set_position_target_local_ned_encode(mavlink_system.sysid, mavlink_system.compid, &msg_position_target_local_ned, &set_position_target_local_ned);
  mavlink_send_msg(mav_chan, &msg_position_target_local_ned);
}

void parse_data(void){
	 int chan = 0;
  uint16_t n=rbGetCount(&mav_buf_receive);
	if(n){
		// printf("Reading from serial port\n");
		for(int i=0; i<n; i++){
			if (mavlink_parse_char(chan,rbPop(&mav_buf_receive), &msg_received, &status)){
					//printf("Received \n");
					switch (msg_received.msgid) {
						case MAVLINK_MSG_ID_HEARTBEAT:
						printf("001 Received heartbeat time: %fs, voltage:%fV, current:%fA\n", ros::Time::now().toSec()-time_start, (double)batt.voltages[1]/1000, (double)batt.current_battery/100);
							// mavlink_msg_heartbeat_decode(&msg_received, &heartbeat);
							mav_send_heartbeat();
							break;

						case MAVLINK_MSG_ID_BATTERY_STATUS:
							mavlink_msg_battery_status_decode(&msg_received, &batt);
							break;

						case MAVLINK_MSG_ID_SCALED_IMU:
							mavlink_msg_scaled_imu_decode(&msg_received, &imu);
							imu_pub.header.frame_id = "scaled_imu";
							imu_pub.header.stamp = ros::Time::now();
							imu_pub.linear_acceleration.x =(double)imu.xacc/1000;
							imu_pub.linear_acceleration.y = -(double)imu.yacc/1000;
							imu_pub.linear_acceleration.z = -(double)imu.zacc/1000;
							imu_pub.angular_velocity.x = (double)imu.xgyro/1000;
							imu_pub.angular_velocity.y = -(double)imu.ygyro/1000;
							imu_pub.angular_velocity.z = -(double)imu.zgyro/1000;
							imu_global.publish(imu_pub);
							// printf("acc0,acc1,acc2,gyr0,gyr1,gyr2: %f %f %f %f %f %f\n",
							//     (double)imu.xacc/1000,-(double)imu.yacc/1000,-(double)imu.zacc/1000,
							//     (double)imu.xgyro/1000,-(double)imu.ygyro/1000,-(double)imu.zgyro/1000);
							break;

						case	MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
						 mavlink_msg_global_position_int_decode(&msg_received, &position);
						 break;

						case  MAVLINK_MSG_ID_GLOBAL_VISION_POSITION_ESTIMATE:
							mavlink_msg_global_vision_position_estimate_decode(&msg_received, &pose);
							odom_pub.header.frame_id = "map";
							odom_pub.header.stamp = ros::Time::now();
							float quaternion_odom[4];
							mavlink_euler_to_quaternion(pose.roll, -pose.pitch, -pose.yaw, quaternion_odom);
							odom_pub.pose.pose.orientation.w=quaternion_odom[0];
							odom_pub.pose.pose.orientation.x=quaternion_odom[1];
							odom_pub.pose.pose.orientation.y=quaternion_odom[2];
							odom_pub.pose.pose.orientation.z=quaternion_odom[3];
							odom_pub.pose.pose.position.x=pose.x*0.01;
							odom_pub.pose.pose.position.y=-pose.y*0.01;
							odom_pub.pose.pose.position.z=(float)position.relative_alt*0.001;

							odomPose.header = odom_pub.header;
							odomPose.pose = odom_pub.pose.pose;
							path_pub.header.stamp = odom_pub.header.stamp;
							path_pub.poses.push_back(odomPose);
							path_pub.header.frame_id = "map";

							odom_global.publish(odom_pub);
							path_global.publish(path_pub);
							break;

						default:
							// printf("msgid: %d\n", msg_received.msgid);
							break;
					}
			}
		}
	}
}

void odomHandler(const nav_msgs::Odometry::ConstPtr& odom)
{
  Eigen::Vector3f position_map ((float)odom->pose.pose.position.x, (float)odom->pose.pose.position.y, (float)odom->pose.pose.position.z) ;
  float quaternion_odom[4]={(float)odom->pose.pose.orientation.w,
                            (float)odom->pose.pose.orientation.x,
                            (float)odom->pose.pose.orientation.y,
                            (float)odom->pose.pose.orientation.z};

  float roll, pitch, yaw;
  mavlink_quaternion_to_euler(quaternion_odom, &roll, &pitch, &yaw);
  printf("x:%f,y:%f,z:%f,yaw:%f\n",position_map.x(),position_map.y(),position_map.z(),yaw);

  mavlink_message_t msg_local_position_ned_cov, msg_attitude;
  mavlink_attitude_t attitude;
  mavlink_local_position_ned_cov_t local_position_ned_cov;

  attitude.yaw = yaw;
  mavlink_msg_attitude_encode(mavlink_system.sysid, mavlink_system.compid, &msg_attitude, &attitude);
  mavlink_send_msg(mav_chan, &msg_attitude);

  local_position_ned_cov.x=position_map.x();
  local_position_ned_cov.y=position_map.y();
  local_position_ned_cov.z=position_map.z();
  mavlink_msg_local_position_ned_cov_encode(mavlink_system.sysid, mavlink_system.compid, &msg_local_position_ned_cov, &local_position_ned_cov);
  mavlink_send_msg(mav_chan, &msg_local_position_ned_cov);
}

void cmdHandler(const std_msgs::Int16::ConstPtr& cmd){
  switch(cmd->data){
    case 1:
        mav_send_arm();
        break;
    case 2:
        mav_send_disarm();
        break;
    case 3:
        mav_send_takeoff();
        break;
    case 4:
        mav_send_land();
        break;
    default:
        break;
  }
}

void missionHandler(const geometry_msgs::InertiaStamped::ConstPtr& mission){
    float yaw=0.0f, yaw_rate=0.0f;
    float px=0.0f, py=0.0f, pz=0.0f;
    float vx=0.0f, vy=0.0f, vz=0.0f;
    float ax=0.0f, ay=0.0f, az=0.0f;
    yaw=mission->inertia.m;
    px=mission->inertia.com.x;
    py=mission->inertia.com.y;
    pz=mission->inertia.com.z;
    vx=mission->inertia.ixx;
    vy=mission->inertia.ixy;
    vz=mission->inertia.ixz;
    ax=mission->inertia.iyy;
    ay=mission->inertia.iyz;
    az=mission->inertia.izz;

		mav_send_target(
			px, py, pz,
			vx, vy, vz,
			ax, ay, az,
			yaw, yaw_rate
		);

}

int main(int argc, char **argv) {

  ros::init(argc, argv, "fcu_bridge_001");
  ros::NodeHandle nh;
  mavlink_system.sysid=254;//强制飞控进入自主模式
  mavlink_system.compid=MAV_COMP_ID_MISSIONPLANNER;

  ros::Rate loop_rate(200);
  imu_global = nh.advertise<sensor_msgs::Imu>("imu_global",100);
  odom_global = nh.advertise<nav_msgs::Odometry>("odom_global_001",100);
  odom=nh.subscribe<nav_msgs::Odometry>("/vins_estimator/odometry", 100, odomHandler);
  cmd=nh.subscribe<std_msgs::Int16>("/fcu_bridge/command", 100, cmdHandler);
  mission=nh.subscribe<geometry_msgs::InertiaStamped>("/fcu_bridge/mission_001", 100, missionHandler);
  path_global = nh.advertise<nav_msgs::Path>("/path_global_001", 100);

  rbInit(&mav_buf_send, TxBuffer, BUF_SIZE);
	rbInit(&mav_buf_receive, RxBuffer, BUF_SIZE);

  if(mav_chan==MAVLINK_COMM_0){
    try{
    //设置串口属性，并打开串口
        ser.setPort("/dev/ttyACM0");
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(5000);
        ser.setTimeout(to);
        ser.open();
    }catch (serial::IOException& e)
    {
        printf("Unable to open port \n");
        return -1;
    }

    //检测串口是否已经打开，并给出提示信息
    if(ser.isOpen())
    {
        printf("Serial Port initialized\n");
    }
    else
    {
        return -1;
    }
  }else{
    socket_cli=socket(AF_INET, SOCK_STREAM, 0);
    if(socket_cli < 0){
      printf("socket error!\n");
      return -1;
    }

    memset(&drone_addr, 0, sizeof(drone_addr));
    drone_addr.sin_family      = AF_INET;
    drone_addr.sin_port        = htons(DRONE_PORT);
    drone_addr.sin_addr.s_addr = inet_addr(DRONE_IP);
    printf("fcu_bridge 001 connecting...\n");

    get_drone=connect(socket_cli, (struct sockaddr*)&drone_addr, sizeof(drone_addr));

    if(get_drone<0){
      printf("fcu_bridge 001 connect error!\n");
      return -1;
    }else{
      printf("fcu_bridge 001 connect succeed!\n");
    }
  }

	int flag = fcntl(socket_cli,F_GETFL,0);//获取socket_cli当前的状态
	if(flag<0){
		printf("fcntl F_GETFL fail");
		close(socket_cli);
		return -1;
	}
	fcntl(socket_cli, F_SETFL, flag | O_NONBLOCK);//设置为非阻塞态

	time_start=ros::Time::now().toSec();

  int n=0;
  while (ros::ok()) {
    ros::spinOnce();
		if(!get_receive_lock()){
			set_receive_lock(true);
			if(mav_chan==MAVLINK_COMM_0){
	      n=ser.available();
	      if(n){
	        //读出数据
	        ser.read(buffer, n);
	      }
	    }else{
	      n=recv(socket_cli, buffer, sizeof(buffer), 0);
	    }
			if(n>0){
				for(uint16_t i=0; i<n; i++){
					rbPush(&mav_buf_receive, buffer[i]);
				}
			}
			set_receive_lock(false);
		}
 	  parse_data();
		if(!get_send_lock()){
			set_send_lock(true);
			flush_data();
			set_send_lock(false);
		}
    loop_rate.sleep();
  }
  ser.close();
	close(socket_cli);
  return 0;
}
