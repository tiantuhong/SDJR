/**
******************************************************************************
* @file    handshake.h
* @author  Jalon
* @date    2017/07/14
* @attention Copyright (C) 2017 Inmotion
******************************************************************************
*/
#ifndef  __HANDSHAKE_H__
#define  __HANDSHAKE_H__

typedef enum{false,true} bool;

bool HandshakeInit(void);
bool HandshakeData1(u8* in_data, u8 out_data[32]);
bool HandshakeData2(u8 out_data[24]);

extern bool HandshakeInit(void);
extern bool HandshakeData1(u8* in_data, u8 out_data[32]);
extern bool HandshakeData2(u8 out_data[24]);

#endif
