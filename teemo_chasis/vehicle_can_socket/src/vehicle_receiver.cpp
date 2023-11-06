/*
 * @Author: DaiKai 
 * @Date: 2022-04-06 11:47:31 
 * @Last Modified by: Chihow Yao
 * @Last Modified time: 2022-08-29 14:39:22
 */

#include <iostream>
#include <ros/ros.h>
#include <cstdlib>
#include <algorithm>
#include <bitset>
#include "can_msgs/Frame.h"
#include "byte.h"
#include "can_id.h"

#include "geometry_msgs/TwistStamped.h"     //vehicle feedback needed
#include "diagnostic_msgs/DiagnosticStatus.h" //vehicle diagnosis needed
#include "diagnostic_msgs/KeyValue.h"      

using namespace std;

static ros::Publisher vehicle_diagnosis_pub;
static ros::Publisher vehicle_state_pub;
static ros::Publisher vehicle_fdb_pub;

double ParseCAN(unsigned char*data_b,unsigned char start,unsigned char len,double factor,double offset)
{
    double result = 0;

    int k =1;
    for (int i =0;i< len;i++) {
        result = result + data_b[i+start]*k;
        k = k*2;
    }
    result = result * factor + offset;
    return  result;
}

geometry_msgs::TwistStamped ParseVCU(const can_msgs::Frame::ConstPtr& can_msgs)
{
    geometry_msgs::TwistStamped vcu;
    vcu.header.frame_id="base_link";
    vcu.header.stamp = ros::Time::now();
    vcu.header.seq ++;

    unsigned char data_b[64];
    for (size_t i=0;i<sizeof (can_msgs->data);i++) {
        for (size_t j=0;j<8;j++) {
            data_b[i*8+j]=(can_msgs->data[i]&(0x01<<j))>>j;
        }
    }

    vcu.twist.linear.x = static_cast<float>(ParseCAN(data_b,0,16,0.1,-80.0));       //speed(kmh)
    vcu.twist.linear.y = 0;                                                         //not used yet
    vcu.twist.linear.z = static_cast<float>(ParseCAN(data_b,16,16,0.01,0.0))-0.2f;  //brake_pressure
    vcu.twist.angular.x = static_cast<float>(ParseCAN(data_b,32,16,0.1,-30.0));     //front wheel theta
    vcu.twist.angular.y = 0;                                                        //not used yet
    vcu.twist.angular.z = 0;                                                        //not used yet

    return  vcu;
}

diagnostic_msgs::DiagnosticStatus ParseDiagnosis(const can_msgs::Frame::ConstPtr& can_msgs)
{
    diagnostic_msgs::DiagnosticStatus diags;
    diags.level = 0;
    diags.name = "VCU vehicle diagnosis";
    diags.message = "Detecting";
    
    unsigned char data_b[64];
    for (size_t i=0;i<sizeof (can_msgs->data);i++) {
        for (size_t j=0;j<8;j++) {
            data_b[i*8+j]=(can_msgs->data[i]&(0x01<<j))>>j;
        }
    }

    //Common Diagnosis
    for(int i = 0; i < 16; i++){
        diags.values.push_back(diagnostic_msgs::KeyValue());
        diags.values[i].key = DIAG_NAME_LIST[i];

        int err = ParseCAN(data_b, i,1,1,0);
        diags.values[i].value = err ? "Fault" : "Ready";
    }

    //Fault level
    diags.values.push_back(diagnostic_msgs::KeyValue());
    diags.values[16].key = DIAG_NAME_LIST[16];
    int level = ParseCAN(data_b,16,2,1,0);
    switch (level)
    {
    case 0:
        diags.values[16].value = "READY";
        diags.level = 0;break;
    case 1:
        diags.values[16].value = "LEVEL 1 FAULT";
        diags.level = 1;break;
    case 2:
        diags.values[16].value = "LEVEL 2 FAULT";
        diags.level = 2;break;
    case 3:
        diags.values[16].value = "LEVEL 3 FAULT";
        diags.level = 3;break;
    default:
        ROS_ERROR("Undefined Fault Level parse.");break;
    }
    
    //Left Light && Right light && Brake Light
    for(int i=0; i < 3; i++){
        diags.values.push_back(diagnostic_msgs::KeyValue());
        diags.values[i+17].key = DIAG_NAME_LIST[i+17];
        int light_on = ParseCAN(data_b, 32+i*8,1,1,0);
        diags.values[i+17].value = light_on ? "ON" : "OFF";
    }

    //Diagnosis summary
    diags.values.push_back(diagnostic_msgs::KeyValue());
    diags.values[20].key = DIAG_NAME_LIST[20];
    int count = ParseCAN(data_b,60,4,1,0);
    diags.values[20].value = to_string(count);

    return diags;
}

diagnostic_msgs::DiagnosticStatus ParseStatus(const can_msgs::Frame::ConstPtr& can_msgs){
    diagnostic_msgs::DiagnosticStatus vcu_status;
    vcu_status.level = 0;
    vcu_status.name = "VCU vehicle status";
    vcu_status.message = "Detecting";

    unsigned char data_b[64];
    for (size_t i=0;i<sizeof (can_msgs->data);i++) {
        for (size_t j=0;j<8;j++) {
            data_b[i*8+j]=(can_msgs->data[i]&(0x01<<j))>>j;
        }
    }

    //Set the vcu status name
    for(int i = 0; i < 13; i++){
        vcu_status.values.push_back(diagnostic_msgs::KeyValue());
        vcu_status.values[i].key = STATE_NAME_LIST[i];
    }

    //0 -> SOC
    vcu_status.values[0].value = to_string(static_cast<int>(ParseCAN(data_b,48,8,1,0)));
    
    //1 -> Gear state
    int gear = static_cast<int>(ParseCAN(data_b,0,2,1,0));
    switch (gear)
    {
    case 0:
        vcu_status.values[1].value = "BRAKE";break;
    case 1:
        vcu_status.values[1].value = "DRIVE";break;
    case 2:
        vcu_status.values[1].value = "NONE";break;
    case 3:
        vcu_status.values[1].value = "REVERSE";break;
    default:
        ROS_ERROR("Undefined gear status.");break;
    }
    
    //2 -> EPB_State
    vcu_status.values[2].value = static_cast<int>(ParseCAN(data_b,3,1,1,0)) ?
                                 "RELEASE" : "FASTEN";
    
    //3 -> EPB_Valid
    vcu_status.values[3].value = static_cast<int>(ParseCAN(data_b,4,1,1,0)) ?
                                 "INVALID" : "VALID";
    
    //4 -> Autodriving_Switch_State
    vcu_status.values[4].value = static_cast<int>(ParseCAN(data_b,5,1,1,0)) ?
                                 "OPEN" : "CLOSED";

    //5 -> Ignition_State
    vcu_status.values[5].value = static_cast<int>(ParseCAN(data_b,6,1,1,0)) ?
                                 "ON" : "OFF";

    //6 -> VCU_Ready_Flag
    vcu_status.values[6].value = static_cast<int>(ParseCAN(data_b,7,1,1,0)) ?
                                 "FAULT" : "READY";

    //7 -> Drive_Mode
    int mode = (static_cast<int>(ParseCAN(data_b,8,4,1,0)));
    switch (mode)
    {
    case 0:     // 0：遥控模式
        vcu_status.values[7].value = "REMOTE CONTROL";break;
    case 1:     // 1：IECU模式
        vcu_status.values[7].value = "IECU MODE";break;
    case 2:     // 2：扭矩环模式
        vcu_status.values[7].value = "TORQUE MODE";break;
    case 3:     // 3：其他原因 转入人工驾驶模式
        vcu_status.values[7].value = "MANUAL MODE";break;
    case 4:     // 4：平行驾驶模式
        vcu_status.values[7].value = "PARALLEL MODE";break;
    default:
        ROS_ERROR("Undefined drive mode.");break;
    }
        
    //8 -> Steering_Mode
    int steer = (static_cast<int>(ParseCAN(data_b,12,4,1,0)));
    switch (steer)
    {
    case 0:     // 0：前轮转向模式
        vcu_status.values[8].value = "FRONT WHEEL STEERING";break;
    case 1:     // 1：后轮转向模式
        vcu_status.values[8].value = "REAR WHEEL STEERING";break;
    case 2:     // 2：四轮同向模式
        vcu_status.values[8].value = "4 WHEEL SAME DIRECTION";break;
    case 3:     // 3：四轮异向模式
        vcu_status.values[8].value = "4 WHEEL DIFF DIRECTION";break;
    default:
        ROS_ERROR("Undefined steering mode.");break;
    }

    //9 -> Speed_Require
    vcu_status.values[9].value = to_string(static_cast<int>(ParseCAN(data_b,16,16,0.1,0)));
    
    //10 -> Brake_Pedal_State
    vcu_status.values[10].value = static_cast<int>(ParseCAN(data_b,32,8,1,0)) ?
                                  "PRESSED" : "LIFT";

    //11 -> Accelerator_Pedal_State
    vcu_status.values[11].value = to_string(static_cast<int>(ParseCAN(data_b,40,8,1,0)));

    //12 -> Vehicle_Status_Counter
    vcu_status.values[12].value = to_string(static_cast<int>(ParseCAN(data_b,60,4,1,0)));

    return vcu_status;
}

void CanFrameCallback(const can_msgs::Frame::ConstPtr& can_msg)
{
    switch (can_msg->id) {
        case VCU_Diagnosis:
            vehicle_diagnosis_pub.publish(ParseDiagnosis(can_msg));break;
        case VCU_State:
            vehicle_state_pub.publish(ParseStatus(can_msg));break;
        case Vehicle_Feedback:
            vehicle_fdb_pub.publish(ParseVCU(can_msg));break;
        default:
            ROS_INFO("case test, %d", can_msg->id);break;    
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "vehicle_receiver");
    ros::NodeHandle nh;
    ros::Subscriber can_frame_pub = nh.subscribe("vehicle_can_messages", 10, CanFrameCallback);
    ROS_INFO("CAN receive start");

    vehicle_diagnosis_pub = nh.advertise<diagnostic_msgs::DiagnosticStatus>("VCU_status/vehicle_diagnosis", 1);
    vehicle_state_pub = nh.advertise<diagnostic_msgs::DiagnosticStatus>("VCU_status/vehicle_status", 1);
    vehicle_fdb_pub = nh.advertise<geometry_msgs::TwistStamped>("VCU_status/vehicle_feedback", 1);

    ros::spin();
    return 0;
}
