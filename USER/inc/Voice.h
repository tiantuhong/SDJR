#ifndef _VOICE_H
#define _VOICE_H

#define PLAYSTATUS      0x01    
#define PLAY            0x02
#define PAUSE           0x03
#define STOP            0x04
#define UP              0x05
#define VOLUME          0x13


typedef struct{
    uint8_t Cmd;            //控制码
    uint8_t DataLength;     //数据长度
    uint8_t Data[5];        //数据
    uint8_t Req;            //语音播放请求 1=有请求 0=无请求
    uint8_t Volume;         // 音量
}T_VOICE_SendFarme;

extern T_VOICE_SendFarme VoiceCur;
extern uint8_t VoiceStatus, VoiceHandshake, VoiceReq, Vocie_DealEn;

extern void Voice_Init(void);
extern void Voice_Deal(void);
extern void VoiceincreasVolum(void);

#endif
