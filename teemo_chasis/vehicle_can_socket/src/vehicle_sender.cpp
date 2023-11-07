/*
 * @Author: DaiKai 
 * @Date: 2022-04-06 11:56:54 
 * @Last Modified by: Chihow Yao
 * @Last Modified time: 2022-04-06 14:38:18
 * @Comment: Intel add counter
 */

#include <iostream>
#include <ros/ros.h>
#include <algorithm>
#include <bitset>
#include "can_msgs/Frame.h"
#include "byte.h"
#include "can_id.h"
#include "autoware_msgs/VehicleCmd.h"

using namespace std;
static ros::Publisher can_frame_pub;

double mps2kmph(double velocity_mps)
{
    return (velocity_mps * 60 * 60) / 1000;
}

typedef  struct{
    unsigned int IECU_Flag;      
	
    unsigned int steer_valid; 	
    int steer_angle;    //0~600
    unsigned int steer_angle_rate; 
    unsigned int up_or_down;

    unsigned int speed_valid; //1有效，0无效   
    unsigned int speed;   //0~800 0.1km/h
    int acc;
    unsigned int brake;   //0~100%  0-8mpaMPa factor=1

    bool light_turn_left;   //0 false, 1 true
    bool light_turn_right;  //0 false, 1 true
    bool light_brake;       //0 false, 1 true
} wireControl_t;

wireControl_t wc;//<线控底盘结构体

can_msgs::Frame canEncoding_IECU_Flag(wireControl_t *wcPtr)
{
    can_msgs::Frame frame;
    frame.id = IECU_Flag;
    frame.is_rtr = false;
    frame.is_extended = false;
  
    frame.dlc = 8;
    for (size_t i=0;i<frame.dlc;i++) {frame.data[i]=0;}

    frame.data[0]=wcPtr->IECU_Flag;
    frame.data[1]=0;
    frame.data[2]=0;
    frame.data[3]=0;
    frame.data[4]=0;
    frame.data[5]=0;
    frame.data[6]=0;
    return frame;
}

can_msgs::Frame canEncoding_Steer(wireControl_t *wcPtr)
{
    can_msgs::Frame frame;
    frame.id = IECU_Steer;
    frame.is_rtr = false;
    frame.is_extended = false;
 
    frame.dlc = 8;
    for (size_t i=0;i<frame.dlc;i++) {frame.data[i]=0;}

    frame.data[0]=wcPtr->steer_valid;
    frame.data[1]=wcPtr->steer_angle_rate; 
    frame.data[2]=0;
    frame.data[3]=0;
    frame.data[4]|=(wcPtr->steer_angle & 0x00ff);
    frame.data[5]|=(wcPtr->steer_angle & 0xff00)>>8;
    frame.data[6]|=(300 & 0x00ff);
    frame.data[7]|=(300 & 0xff00)>>8;

    return  frame;
}
/**
 * @brief send message for 0x504
  */
can_msgs::Frame canEncoding_Speed(wireControl_t *wcPtr)
{
    can_msgs::Frame frame;
    frame.id = IECU_Speed;
    frame.is_rtr = false;
    frame.is_extended = false;
    frame.dlc = 8;
    for (size_t i=0;i<frame.dlc;i++) {frame.data[i]=0;}

    frame.data[0]=1;
    frame.data[1]=0;
    frame.data[2]=1; //0表示扭矩控制，1表示速度控制，2表示加速度控制模式
    frame.data[3]=wcPtr->up_or_down; //1表示前进档，3表示倒车档
    frame.data[4]=wcPtr->acc; //对应过来的加减速度
    frame.data[5]=0;
    frame.data[6]|=(wcPtr->speed & 0x00ff);
    frame.data[7]|=(wcPtr->speed & 0xff00);
   
    return  frame;
}

can_msgs::Frame canEncoding_Brake(wireControl_t *wcPtr)
{
    can_msgs::Frame frame;
    frame.id = IECU_Brake;
    frame.is_rtr = false;
    frame.is_extended = false;
    frame.dlc = 8;
    for (size_t i=0;i<frame.dlc;i++) {frame.data[i]=0;}

    frame.data[0]=1; //valid brake
    frame.data[1]=wc.brake;
    frame.data[2]=0; 
    frame.data[3]=0;
    frame.data[4]=0; 
    frame.data[5]=0;
    frame.data[6]=0;
    frame.data[7]=0;
   
    return  frame;
}

can_msgs::Frame canEncoding_Light(wireControl_t *wcPtr)
{
    can_msgs::Frame frame;
    frame.id = Light_Flag;
    frame.is_rtr = false;
    frame.is_extended = false;
    frame.dlc = 8;
    for (size_t i=0;i<frame.dlc;i++) {frame.data[i]=0;}

    frame.data[0]=(wcPtr->light_turn_left << 1) | wcPtr->light_turn_right;
    frame.data[1]=wcPtr->light_brake;
    frame.data[2]=0; 
    frame.data[3]=0;
    frame.data[4]=0; 
    frame.data[5]=0;
    frame.data[6]=0;
    frame.data[7]=0;
   
    return  frame;
}

void ControlCmdCallback(const autoware_msgs::VehicleCmd& msg)
{
    can_msgs::Frame frame;
    can_msgs::Frame frame_1;
    can_msgs::Frame frame_2;
    can_msgs::Frame frame_3;
    can_msgs::Frame frame_4;

    //使能自动驾驶
    wc.IECU_Flag = 1;  

    //转角控制
    wc.steer_valid = 1;
    wc.steer_angle_rate = 200;
  
    wc.light_turn_left = false;
    wc.light_turn_right = false;
    wc.light_brake = false;
  
    //制动控制
    wc.acc =  msg.accel_cmd.accel;
    if (wc.acc == -5.0) //当有这个减速度请求的时候说明到终点了
    {
        wc.brake = 10;
        wc.light_brake = true;
        wc.steer_angle = 300;  
    }
    else if (wc.acc == -8.0) //当检测到障碍物时，发出-8的减速度
    {
        wc.brake = 20;
        wc.light_brake = true;
        wc.speed = 0;
        wc.steer_angle = msg.ctrl_cmd.steering_angle*57.3*10+300;  
        wc.steer_angle = wc.steer_angle < 0 ? 0 :
                                      wc.steer_angle > 600 ? 600 : wc.steer_angle; 
    }  
    else    //正常模式下无制动压力，转角、速度由自动驾驶系统传过来
    {
        wc.brake = 0;
    
        wc.steer_angle = msg.ctrl_cmd.steering_angle*57.3*10+300;  //rad -> angle -> dbc
        wc.steer_angle = wc.steer_angle < 0 ? 0 :
                         wc.steer_angle > 600 ? 600 : wc.steer_angle;
    
        if(msg.ctrl_cmd.linear_velocity >= 0){
            wc.up_or_down = 1;//on
        }
        else{
            wc.up_or_down = 3;//back
        }

        wc.speed =10 * mps2kmph(abs(msg.ctrl_cmd.linear_velocity));
        //做15的限速
        wc.speed = wc.speed< 0 ? 0 :
                   wc.speed > 150 ? 150 : wc.speed;
    }
  
    cout<<"当前速度是: "<< msg.ctrl_cmd.linear_velocity <<"  m/s "<<endl;
    cout<<"当前制动压力是: "<< wc.brake<<"  % "<<endl;
    cout<<"当前转角是 "<< msg.ctrl_cmd.steering_angle*57.3 <<"  度 "<<endl;

    //转向灯控制
    if (msg.ctrl_cmd.steering_angle*57.3 > 8){
        wc.light_turn_right = true;
    }
    else if (msg.ctrl_cmd.steering_angle*57.3 < -8){
        wc.light_turn_left = true;
    }

    //发送CAN信号
    frame=canEncoding_IECU_Flag(&wc);
    frame_1=canEncoding_Speed(&wc);
    frame_2=canEncoding_Steer(&wc);
    frame_3=canEncoding_Brake(&wc);
    frame_4=canEncoding_Light(&wc);

    //add counter i => (0-F) 0000->1111 与心跳有关
    for(int i=0; i <= 15; i++){
        frame.data[0] |= (i << 4); frame.header = msg.header;
        frame.header.stamp =  ros::Time::now();
        frame_1.data[0] |= (i << 4);frame_1.header = msg.header;
        frame_1.header.stamp = ros::Time::now();
        frame_2.data[0] |= (i << 4);frame_2.header = msg.header;
        frame_2.header.stamp = ros::Time::now();
        frame_3.data[0] |= (i << 4);frame_3.header = msg.header;
        frame_3.header.stamp = ros::Time::now();
        frame_4.data[0] |= (i << 4);frame_4.header = msg.header;
        frame_4.header.stamp = ros::Time::now();


        can_frame_pub.publish(frame);frame.data[0] = 1;
        can_frame_pub.publish(frame_1);frame_1.data[0] = 1;
        can_frame_pub.publish(frame_2);frame_2.data[0] = 1;
        can_frame_pub.publish(frame_3);frame_3.data[0] = 1;
        can_frame_pub.publish(frame_4);frame_4.data[0] &= 0x0f;
        usleep(20000);
    }     
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "vehicle_sender");
    ros::NodeHandle nh;
    can_frame_pub = nh.advertise<can_msgs::Frame>("sent_messages", 10);
    ros::Subscriber control_cmd_sub = nh.subscribe("/vehicle_cmd", 10, ControlCmdCallback);

    ros::spin();
    return 0;
}
