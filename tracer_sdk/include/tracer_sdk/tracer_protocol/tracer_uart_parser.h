/* 
 * tracer_uart_parser.h
 * 
 * Created on: Aug 14, 2019 12:01
 * Description: 
 * 
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#ifndef TRACER_UART_PARSER_H
#define TRACER_UART_PARSER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#include "tracer_sdk/tracer_protocol/tracer_protocol.h"

bool DecodeTracerStatusMsgFromUART(uint8_t c, TracerStatusMessage *msg);
bool DecodeTracerControlMsgFromUART(uint8_t c, TracerControlMessage *msg);

void EncodeTracerStatusMsgToUART(const TracerStatusMessage *msg, uint8_t *buf, uint8_t *len);
void EncodeTracerControlMsgToUART(const TracerControlMessage *msg, uint8_t *buf, uint8_t *len);

void EncodeMotionControlMsgToUART(const MotionControlMessage *msg, uint8_t *buf, uint8_t *len);
void EncodeLightControlMsgToUART(const LightControlMessage *msg, uint8_t *buf, uint8_t *len);

uint8_t CalcTracerUARTChecksum(uint8_t *buf, uint8_t len);

#ifdef __cplusplus
}
#endif

#endif /* TRACER_UART_PARSER_H */
