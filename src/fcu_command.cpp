#include <ros/ros.h>
#include <stdio.h>
#include <string>
#include <iostream>
#include <std_msgs/Int16.h>

static char buf[16] = {0};
static std_msgs::Int16 cmd;

int main(int argc, char **argv) {

  ros::init(argc, argv, "fcu_command");
  ros::NodeHandle nh;
  ros::Publisher command = nh.advertise<std_msgs::Int16>("/fcu_bridge/command",100);

  while (ros::ok()) {
    // 获取从键盘输入的数据
    printf("请输入指令：\n");
    ssize_t size = read(STDIN_FILENO, buf, sizeof(buf));
    if(size!=2){
      printf("非法指令！\n");
      continue;
    }
    switch(buf[0]){
      case 'a':
        printf("解锁\n");
        cmd.data=1;
        command.publish(cmd);
        break;
      case 'd':
        printf("锁定\n");
        cmd.data=2;
        command.publish(cmd);
        break;
      case 't':
        printf("起飞\n");
        cmd.data=3;
        command.publish(cmd);
        break;
      case 'l':
        printf("降落\n");
        cmd.data=4;
        command.publish(cmd);
        break;
      case 'r':
        printf("运行\n");
        cmd.data=5;
        command.publish(cmd);
        break;
      case 's':
        printf("停止\n");
        cmd.data=6;
        command.publish(cmd);
        break;
      default:
        break;
    }
  }

  return 0;
}
