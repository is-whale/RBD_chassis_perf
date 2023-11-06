
/**
 * @file
 * @brief Defines can id && VCU protocol.
 */

#pragma once

#include <string>


/******request******/
const unsigned int IECU_Flag = 0x501;
const unsigned int IECU_Steer = 0x502;
const unsigned int IECU_Brake = 0x503;
const unsigned int IECU_Speed = 0x504;
const unsigned int Light_Flag = 0x506;


/******feedback******/
const unsigned int VCU_Diagnosis = 0x301;
const unsigned int VCU_State = 0x303;
const unsigned int Vehicle_Feedback = 0x304;


/*****diagnosis******/
const std::string DIAG_NAME_LIST[21] = 
    {"Emergency_Button_State",
    "FRMotor_State",
    "FLMotor_State",
    "RRMotor_State",
    "RRMotor_State",
    "Remote_State",
    "IECU_State",
    "DBS_State",
    "Parallel_State",
    "BMS_State",
    "Fsteering_State",
    "Rsteering_State",
    "F_Attach_Switch_State",
    "R_Attach_Switch_State",
    "EPB_Diagnosis",
    "Headlight_State",
    "Vehicle_Fault_Grade",
    "Light_State_Left",
    "Light_State_Brake",
    "Light_State_Right",
    "Vehicle_Diagnosis_Counter"
    };


/*******state********/
const std::string STATE_NAME_LIST[13] = 
    {"Battery_Percentage(%)",
    "Gear_State",
    "EPB_State",
    "EPB_Valid",
    "Autodriving_Switch_State",
    "Ignition_State",
    "VCU_Ready_Flag",
    "Drive_Mode",
    "Steering_Mode",
    "Speed_Require(km/h)",
    "Brake_Pedal_State",
    "Accelerator_Pedal_State(%)",
    "Vehicle_Status_Counter"
    };
