#include "Com_PID.h"

void ResetPID(PidObject ** pidObjects,uint8_t len)
{
    for (uint8_t i = 0; i < len; i++)
    {
        pidObjects[i] -> prevError = 0;
        pidObjects[i] -> integ = 0;
        pidObjects[i] -> out = 0;
    }
    
}


void PID_Update(PidObject *pid, float dt)
{
    float temp_err; // ����ƫ��
    float dri;
    /* 1������ƫ��ֵ */
    temp_err = pid->desired - pid->measured;
    /* 2��������֣� ƫ���ۻ��� */
    pid->integ += temp_err * dt;
    /* 3������΢�֣� ƫ��ı仯�� */
    dri = (temp_err - pid->prevError) / dt;
    /* 4��������浽out�����ƫ��ֵ���浽 ���ϴ�ƫ� �ֶ� */
    pid->out = pid->kp * temp_err + pid->ki * pid->integ + pid->kd * dri;
    pid ->prevError = temp_err;
}

void CasecadePID(PidObject *pidAngle,PidObject *pidRate,float dt)
{
    /* 1���Ƕ��⻷����PID���� */
    PID_Update(pidAngle,dt);
    /* 2���⻷���������ֵ���ڻ�������ֵ */
    pidRate -> desired = pidAngle ->out;
    /* 3�����ڻ�����PID���� */
    PID_Update(pidRate,dt);
}
