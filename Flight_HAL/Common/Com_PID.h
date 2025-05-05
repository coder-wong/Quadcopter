#ifndef __COM_PID_H
#define __COM_PID_H

#include "main.h"

typedef volatile struct
{
    float desired;   // ����ֵ
    float prevError; // �ϴ�ƫ��
    float integ;     // �������ۼ�ֵ
    float kp;        // p����
    float ki;        // i����
    float kd;        // d����
    float measured;  // ʵ�ʲ���ֵ
    float out;       // pid���
} PidObject;

void ResetPID(PidObject ** pidObjects, uint8_t len);

void PID_Update(PidObject *pid, float dt);

void CasecadePID(PidObject *pidAngle, PidObject *pidRate, float dt);

#endif
