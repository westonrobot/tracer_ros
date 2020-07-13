/* 
 * tracer_can_parser.c
 * 
 * Created on: Aug 31, 2019 04:25
 * Description: 
 * 
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#include "tracer_sdk/tracer_protocol/tracer_can_parser.h"

#include "string.h"

bool DecodeTracerStatusMsgFromCAN(const struct can_frame *rx_frame, TracerStatusMessage *msg)
{
    msg->msg_type = TracerStatusNone;

    switch (rx_frame->can_id)
    {
    // in the current implementation, both MsgType and can_frame include 8 * uint8_t
    case CAN_MSG_MOTION_CONTROL_STATUS_ID:
    {
        msg->msg_type = TracerMotionStatusMsg;
        // msg->motion_status_msg.id = CAN_MSG_MOTION_CONTROL_STATUS_ID;
        memcpy(msg->motion_status_msg.data.raw, rx_frame->data, rx_frame->can_dlc * sizeof(uint8_t));
        break;
    }
    case CAN_MSG_LIGHT_CONTROL_STATUS_ID:
    {
        msg->msg_type = TracerLightStatusMsg;
        // msg->light_status_msg.id = CAN_MSG_LIGHT_CONTROL_STATUS_ID;
        memcpy(msg->light_status_msg.data.raw, rx_frame->data, rx_frame->can_dlc * sizeof(uint8_t));
        break;
    }
    case CAN_MSG_SYSTEM_STATUS_STATUS_ID:
    {
        msg->msg_type = TracerSystemStatusMsg;
        // msg->system_status_msg.id = CAN_MSG_SYSTEM_STATUS_STATUS_ID;
        memcpy(msg->system_status_msg.data.raw, rx_frame->data, rx_frame->can_dlc * sizeof(uint8_t));
        break;
    }
    case CAN_MSG_MOTOR1_DRIVER_STATUS_ID:
    {
        msg->msg_type = TracerMotorDriverStatusMsg;
        // msg->motor_driver_status_msg.id = CAN_MSG_MOTOR1_DRIVER_STATUS_ID;
        msg->motor_driver_status_msg.motor_id = TRACER_MOTOR1_ID;
        memcpy(msg->motor_driver_status_msg.data.raw, rx_frame->data, rx_frame->can_dlc * sizeof(uint8_t));
        break;
    }
    case CAN_MSG_MOTOR2_DRIVER_STATUS_ID:
    {
        msg->msg_type = TracerMotorDriverStatusMsg;
        // msg->motor_driver_status_msg.id = CAN_MSG_MOTOR2_DRIVER_STATUS_ID;
        msg->motor_driver_status_msg.motor_id = TRACER_MOTOR2_ID;
        memcpy(msg->motor_driver_status_msg.data.raw, rx_frame->data, rx_frame->can_dlc * sizeof(uint8_t));
        break;
    }
    default:
        break;
    }

    return true;
}

bool DecodeTracerControlMsgFromCAN(const struct can_frame *rx_frame, TracerControlMessage *msg)
{
    msg->msg_type = TracerControlNone;

    switch (rx_frame->can_id)
    {
    // in the current implementation, both MsgType and can_frame include 8 * uint8_t
    case CAN_MSG_MOTION_CONTROL_CMD_ID:
    {
        msg->msg_type = TracerMotionControlMsg;
        // msg->motion_control_msg.id = CAN_MSG_MOTION_CONTROL_CMD_ID;
        memcpy(msg->motion_control_msg.data.raw, rx_frame->data, rx_frame->can_dlc * sizeof(uint8_t));
        break;
    }
    case CAN_MSG_LIGHT_CONTROL_CMD_ID:
    {
        msg->msg_type = TracerLightControlMsg;
        // msg->light_control_msg.id = CAN_MSG_LIGHT_CONTROL_STATUS_ID;
        memcpy(msg->light_control_msg.data.raw, rx_frame->data, rx_frame->can_dlc * sizeof(uint8_t));
        break;
    }
    default:
        break;
    }

    return true;
}

void EncodeTracerStatusMsgToCAN(const TracerStatusMessage *msg, struct can_frame *tx_frame)
{
    switch (msg->msg_type)
    {
    // in the current implementation, both MsgType and can_frame include 8 * uint8_t
    case TracerMotionStatusMsg:
    {
        tx_frame->can_id = CAN_MSG_MOTION_CONTROL_STATUS_ID;
        tx_frame->can_dlc = 8;
        memcpy(tx_frame->data, msg->motion_status_msg.data.raw, tx_frame->can_dlc);
        break;
    }
    case TracerLightStatusMsg:
    {
        tx_frame->can_id = CAN_MSG_LIGHT_CONTROL_STATUS_ID;
        tx_frame->can_dlc = 8;
        memcpy(tx_frame->data, msg->light_status_msg.data.raw, tx_frame->can_dlc);
        break;
    }
    case TracerSystemStatusMsg:
    {
        tx_frame->can_id = CAN_MSG_SYSTEM_STATUS_STATUS_ID;
        tx_frame->can_dlc = 8;
        memcpy(tx_frame->data, msg->system_status_msg.data.raw, tx_frame->can_dlc);
        break;
    }
    case TracerMotorDriverStatusMsg:
    {
        if (msg->motor_driver_status_msg.motor_id == TRACER_MOTOR1_ID)
            tx_frame->can_id = CAN_MSG_MOTOR1_DRIVER_STATUS_ID;
        else if (msg->motor_driver_status_msg.motor_id == TRACER_MOTOR2_ID)
            tx_frame->can_id = CAN_MSG_MOTOR2_DRIVER_STATUS_ID;
        tx_frame->can_dlc = 8;
        memcpy(tx_frame->data, msg->motor_driver_status_msg.data.raw, tx_frame->can_dlc);
        break;
    }
    default:
        break;
    }
    tx_frame->data[7] = CalcTracerCANChecksum(tx_frame->can_id, tx_frame->data, tx_frame->can_dlc);
}

void EncodeTracerControlMsgToCAN(const TracerControlMessage *msg, struct can_frame *tx_frame)
{
    switch (msg->msg_type)
    {
    case TracerMotionControlMsg:
    {
        EncodeTracerMotionControlMsgToCAN(&(msg->motion_control_msg), tx_frame);
        break;
    }
    case TracerLightControlMsg:
    {
        EncodeTracerLightControlMsgToCAN(&(msg->light_control_msg), tx_frame);
        break;
    }
    default:
        break;
    }
}

void EncodeTracerMotionControlMsgToCAN(const MotionControlMessage *msg, struct can_frame *tx_frame)
{
    tx_frame->can_id = CAN_MSG_MOTION_CONTROL_CMD_ID;
    tx_frame->can_dlc = 8;
    memcpy(tx_frame->data, msg->data.raw, tx_frame->can_dlc);
    tx_frame->data[7] = CalcTracerCANChecksum(tx_frame->can_id, tx_frame->data, tx_frame->can_dlc);
}

void EncodeTracerLightControlMsgToCAN(const LightControlMessage *msg, struct can_frame *tx_frame)
{
    tx_frame->can_id = CAN_MSG_LIGHT_CONTROL_CMD_ID;
    tx_frame->can_dlc = 8;
    memcpy(tx_frame->data, msg->data.raw, tx_frame->can_dlc);
    tx_frame->data[7] = CalcTracerCANChecksum(tx_frame->can_id, tx_frame->data, tx_frame->can_dlc);
}

uint8_t CalcTracerCANChecksum(uint16_t id, uint8_t *data, uint8_t dlc)
{
    uint8_t checksum = 0x00;
    checksum = (uint8_t)(id & 0x00ff) + (uint8_t)(id >> 8) + dlc;
    for (int i = 0; i < (dlc - 1); ++i)
        checksum += data[i];
    return checksum;
}
