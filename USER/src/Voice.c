
#include "stm32f10x.h"

void VoiceCmd(uint8_t cmd, uint8_t len, uint8_t * data);

T_VOICE_SendFarme VoiceCur;


uint8_t VoiceStatus; // 00- ֹͣ 1--���� 2--��ͣ
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


//1. ��ʱ���� 100ms
//2. VoiceHandshake �����Ƿ�ɹ�
//3. ��ʼ���Ƿ�ɹ�  (�������õ�)
//4. �Ƿ��п������� , (0 - û��  1 - ����1, 2-����2, 3-����3, ....5-����5)
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
                    // ���ͺ�״̬��Ϊ��������ʾ�Ѿ�ִ��
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
        //״̬��ѯָ�ʱ�ϴ����˴�����Ҫ����
          //����״̬��ѯָ��
//        VoiceCur.Cmd = PLAYSTATUS;
//        VoiceCur.DataLength = 0;
//        VoiceCur.Req = 1;
    }
}

// cmd: ���������� 02 - play  03 - pause   04 - stop  07 - ָ����Ŀ����  0x13-��������  0x14-����+  0x15-����-
// data: ����ָ��
// len:  ���ݳ���
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

