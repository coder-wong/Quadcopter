#ifndef __APP_FLIGHT
#define __APP_FLIGHT

#include "main.h"
#include "usart.h"
#include "Int_LED.h"
#include "Int_MPU6050.h"
#include "Com_Kalman.h"
#include "NRF24L01.h"
#include "Com_PID.h"
#include "tim.h"

/* ���弸���꣬����������Ĳ�ͬ״̬ */
#define ENMERGENCY_0 0 //����׶��ж������Ƿ���ͣ�����ǣ���ɵڶ��׶�
#define WAITING_1 1 //����׶��ж������Ƿ���ͣ�����ǣ���ɵڶ��׶�
#define WAITING_2 2 //����׶��ж������Ƿ���ߣ�����ǣ���ɵ����׶�
#define WAITING_3 3 //����׶��ж������Ƿ���ͣ�����ǣ���ɵ��Ľ׶�
#define WAITING_4 4 //����׶α�ʾ�������ɹ��׶�
#define PROCESS 5   //����׶α�ʾ����ʽ���ƽ׶�
#define EXIT 6   //����׶α�ʾ����ʽ���ƽ׶�

#define LIMIT(x,min,max) (x<min)?min:((x>max)?max:x)

typedef struct
{
    uint16_t FlashTime;
    enum
    {
        AlwaysOn,
        AlwaysOff,
        AllFlashLight,
        AlternateFlash,
        WARNING,
        DANGEROURS,
        GET_OFFSET
    } status;
}sLED;

typedef struct 
{
    int16_t accX;
    int16_t accY;
    int16_t accZ;
    int16_t gryoX;
    int16_t gryoY;
    int16_t gryoZ;
} _stMPU;

typedef struct
{
    float pitch; // ������
    float roll; // �����
    float yaw; // ƫ����
} _stAngle;

typedef struct 
{
    int16_t THR; // ���ţ���ҡ������
    int16_t YAW; // ƫ������ҡ������
    int16_t ROL; // �������ҡ������
    int16_t PIT; // ��������ҡ������
    /* Ԥ����6������ͨ��������ʲô��;���Լ����壬���������ǲ��� */
    int16_t AUX1;
    int16_t AUX2;
    int16_t AUX3;
    int16_t AUX4;
    int16_t AUX5;
    int16_t AUX6;
}_stRemote;

extern _stMPU MPU6050;
extern _stAngle Angle;
extern _stRemote remote_data;

extern PidObject pidPitch;
extern PidObject pidRoll;
extern PidObject pidYaw;
extern PidObject pidRateX;
extern PidObject pidRateY;
extern PidObject pidRateZ;

extern sLED LED; // ��������ļ��������ͷ�ļ�
void App_PolitLED(void);
void App_Flight_Volcheck(void);
void App_Flight_MPU_Data(void);
void App_Flight_MPU_Offsets(void);
void App_Flight_Remote_Check(uint8_t *buf, uint8_t len);
void App_Flight_RC_Unlock(void);
void App_Flight_RC_Analysis(void);
void App_Flight_Motor_Control(void);
void App_Flight_PID_Control(float dt);
void App_Flight_Mode_Control(void);
void App_PID_Param_Init(void);


#endif
