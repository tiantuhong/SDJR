
#include "stm32f10x.h"

void VoiceCmd(uint8_t cmd, uint8_t len, uint8_t * data);

T_VOICE_SendFarme VoiceCur;


uint8_t VoiceStatus; // 00- 停止 1--播放 2--暂停
uint8_t VoiceHandshake, VoiceInit;
uint8_t VoiceReq, Vocie_DealEn;

void Voice_Init(void)
{
    VoiceCur.Cmd = PLAYSTATUS;
    VoiceCur.DataLength = 0;
    VoiceCur.Req = 1;
    VoiceStatus = 0;
    VoiceHandshake = 0;
    VoiceCur.Volume = 30;
    VoiceReq = 0;
}


//1. 定时调用 100ms
//2. VoiceHandshake 握手是否成功
//3. 初始化是否成功  (音量设置等)
//4. 是否有控制请求 , (0 - 没有  1 - 播放1, 2-播放2, 3-播放3, ....5-播放5)
void Voice_Deal(void)
{
    static uint8_t voicesend = 0;
    uint8_t data[3];
    
    if(!Vocie_DealEn)
        return;
    Vocie_DealEn = 0;
    
    if(VoiceHandshake)
    {
        if(!VoiceInit)
        {
            VoiceInit = 1;
            data[0] = VoiceCur.Volume;
            VoiceCmd(VOLUME, 1, data);
        }
        else
        {
            if(VoiceReq)
            {
                if(VoiceStatus == 0)
                {
                    data[0] = 0;
                    data[1] = VoiceReq;
                    VoiceCmd(7, 2, data);
                    voicesend = 1;
                }
                else
                {
                    // 发送后，状态变为工作，表示已经执行
                    if(voicesend)
                    {
                        VoiceReq = 0;
                        voicesend = 0;
                    }
                }
            }
        }
    }
    else
    {
        //状态查询指令定时上传，此处不需要处理
          //发送状态查询指令
//        VoiceCur.Cmd = PLAYSTATUS;
//        VoiceCur.DataLength = 0;
//        VoiceCur.Req = 1;
    }
}

// cmd: 语音控制码 02 - play  03 - pause   04 - stop  07 - 指定曲目播放  0x13-音量设置  0x14-音量+  0x15-音量-
// data: 数据指针
// len:  数据长度
void VoiceCmd(uint8_t cmd, uint8_t len, uint8_t * data)
{
    uint8_t i;
    VoiceCur.Cmd = cmd;
    VoiceCur.DataLength = len;
    for(i = 0; i < len; i++)
    {
        VoiceCur.Data[i] = data[i];
    }   
    VoiceCur.Req = 1;
}

