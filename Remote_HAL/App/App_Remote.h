#ifndef __APP_REMOTE_H
#define __APP_REMOTE_H

#include "main.h"
#include "usart.h"

#define Filter_Num 10
#define READ_KEY_LEFT_X HAL_GPIO_ReadPin(KEY_LEFT_X_GPIO_Port, KEY_LEFT_X_Pin)
/* ��ȡ�ĸ�΢�������ĵ�ƽ */
#define READ_KEY_U HAL_GPIO_ReadPin(KEY_U_GPIO_Port, KEY_U_Pin)
#define READ_KEY_D HAL_GPIO_ReadPin(KEY_D_GPIO_Port, KEY_D_Pin)
#define READ_KEY_L HAL_GPIO_ReadPin(KEY_L_GPIO_Port, KEY_L_Pin)
#define READ_KEY_R HAL_GPIO_ReadPin(KEY_R_GPIO_Port, KEY_R_Pin)

struct _Rc
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
};

struct _Offset
{
    int16_t THR; // ���ţ���ҡ������
    int16_t YAW; // ƫ������ҡ������
    int16_t ROL; // �������ҡ������
    int16_t PIT; // ��������ҡ������
};

struct _Filter
{
    uint32_t sum;
    uint16_t old[Filter_Num];
};

extern struct _Rc rc;
extern struct _Offset offset;

void App_Remote_Stick_Scan(void);
void App_Remote_KeyPress(void);
void App_Remote_Data(uint8_t *remote_data);

#endif
