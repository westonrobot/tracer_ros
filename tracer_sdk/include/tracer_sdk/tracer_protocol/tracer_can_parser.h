/* 
 * tracer_can_parser.h
 * 
 * Created on: Aug 31, 2019 04:23
 * Description: 
 * 
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */ 

#ifndef TRACER_CAN_PARSER_H
#define TRACER_CAN_PARSER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#include "tracer_sdk/tracer_protocol/tracer_protocol.h"

#ifdef __linux__
#include <linux/can.h>
#else
struct can_frame
{
    uint32_t can_id;
    uint8_t can_dlc;
    uint8_t data[8]__attribute__((aligned(8)));
};
#endif

bool DecodeTracerStatusMsgFromCAN(const struct can_frame *rx_frame, TracerStatusMessage *msg);
bool DecodeTracerControlMsgFromCAN(const struct can_frame *rx_frame, TracerControlMessage *msg);

void EncodeTracerStatusMsgToCAN(const TracerStatusMessage *msg, struct can_frame *tx_frame);
void EncodeTracerControlMsgToCAN(const TracerControlMessage *msg, struct can_frame *tx_frame);

void EncodeTracerMotionControlMsgToCAN(const MotionControlMessage *msg, struct can_frame *tx_frame);
void EncodeTracerLightControlMsgToCAN(const LightControlMessage *msg, struct can_frame *tx_frame);

uint8_t CalcTracerCANChecksum(uint16_t id, uint8_t *data, uint8_t dlc);

#ifdef __cplusplus
}
#endif

#endif /* TRACER_CAN_PARSER_H */
