#include "App_Remote.h"

struct _Rc rc;
struct _Offset offset;
extern uint16_t ADC_Value[4];
struct _Filter Filter_Thr, Filter_Pitch, Filter_Roll, Filter_Yaw;


/**
 * @description: ȡ���N�ε�ƽ��ֵ
 * @param {_Rc} *rc
 * @return {*}
 */
void App_Remote_Window_Filter(struct _Rc *rc)
{
    static uint16_t count = 0;
    /* 1������������ */
    Filter_Thr.sum -= Filter_Thr.old[count];
    Filter_Pitch.sum -= Filter_Pitch.old[count];
    Filter_Roll.sum -= Filter_Roll.old[count];
    Filter_Yaw.sum -= Filter_Yaw.old[count];

    /* 2������������:������Ҫ�ӵ�sum�ͬʱҲҪ�浽old�� */
    Filter_Thr.old[count] = rc->THR;
    Filter_Pitch.old[count] = rc->PIT;
    Filter_Roll.old[count] = rc->ROL;
    Filter_Yaw.old[count] = rc->YAW;

    Filter_Thr.sum += Filter_Thr.old[count];
    Filter_Pitch.sum += Filter_Pitch.old[count];
    Filter_Roll.sum += Filter_Roll.old[count];
    Filter_Yaw.sum += Filter_Yaw.old[count];

    /* 3����ƽ��ֵ */
    rc->THR = Filter_Thr.sum / Filter_Num;
    rc->PIT = Filter_Pitch.sum / Filter_Num;
    rc->ROL = Filter_Roll.sum / Filter_Num;
    rc->YAW = Filter_Yaw.sum / Filter_Num;

    /* 4���ж�����ֵ�ķ�Χ */
    count++;
    if (count >= 10)
    {
        count = 0;
    }
}

void App_Remote_Mid_Offset(struct _Rc *rc)
{
	static uint16_t key_count = 0; // ��¼����������ʱ��
	static uint32_t sum_thr = 0, sum_yaw = 0, sum_rol = 0, sum_pit = 0, count = 0; // ��¼N�ε�ҡ��ֵ�ĺͣ��Լ�Ŀǰ��¼�Ĵ���count
	if(!READ_KEY_LEFT_X) // ������������Ϊ�͵�ƽ
	{
		key_count++; // ��ֹ�󴥣����ó�������У׼
		if(key_count>=20)
		{
			printf("start offset...\r\n");
			if(count == 0) // ��һ�ν��룬��ʼ��
			{
				sum_thr = 0;
				sum_yaw = 0;
				sum_rol = 0; 
				sum_pit = 0; 
				count = 1; // ��ֹ�ڽ����ʼ��
				offset.THR = 0;
				offset.ROL = 0;
				offset.PIT = 0;
				offset.YAW = 0;
				return; //��ʼ����ֱ�ӷ��أ�������һ��ѭ��
			}
			else
			{
				sum_yaw += rc->YAW;
				sum_rol += rc->ROL;
				sum_pit += rc->PIT;
				sum_thr += rc->THR;
				count++;
			}
			
			if(count == 51) //��Ϊ��һ���ǳ�ʼ�����ܹ��ۼ�50��
			{
				count--;//--��Ϊ������50��
				offset.PIT = sum_pit/count - 1500;
				offset.THR = sum_thr/count - 1000; // ��������͵�У׼����������е�У׼
				offset.ROL = sum_rol/count - 1500;
				offset.YAW = sum_yaw/count - 1500;
				
				 /* ��ȡ��ƫ��ֵ����ȥ����count��key_count */
				count = 0;
				key_count = 0;
			}
		}
		
	}
}

/**
 * @description: ���ֵ����Сֵ�޷�����ֹ���������������Խ��
 * @param {_Rc} *rc
 * @return {*}
 */
void App_Remote_Limit(struct _Rc *rc)
{
    rc->THR = (rc->THR < 1000) ? 1000 : rc->THR;
    rc->THR = (rc->THR > 2000) ? 2000 : rc->THR;

    rc->PIT = (rc->PIT < 1000) ? 1000 : rc->PIT;
    rc->PIT = (rc->PIT > 2000) ? 2000 : rc->PIT;

    rc->ROL = (rc->ROL < 1000) ? 1000 : rc->ROL;
    rc->ROL = (rc->ROL > 2000) ? 2000 : rc->ROL;

    rc->YAW = (rc->YAW < 1000) ? 1000 : rc->YAW;
    rc->YAW = (rc->YAW > 2000) ? 2000 : rc->YAW;
}

/**
 * @description: �е��޷�����Ϊ���м丽��������Ҫ��ֵ
 * @param {_Rc} *rc
 * @return {*}
 */
void App_Remote_Mid_Limit(struct _Rc *rc)
{
    /* ���ǵ����Ų���ص����ֶ�������ȫ���м���ѣ���Χ����һ�� */
    if (rc->THR > 1450 && rc->THR < 1550)
    {
        rc->THR = 1500;
    }

    /* ��������ҡ�˻��Զ����У���Χ���Ը�Сһ�� */
    if (rc->PIT > 1490 && rc->PIT < 1510)
    {
        rc->PIT = 1500;
    }
    if (rc->ROL > 1490 && rc->ROL < 1510)
    {
        rc->ROL = 1500;
    }
    if (rc->YAW > 1490 && rc->YAW < 1510)
    {
        rc->YAW = 1500;
    }
}

void App_Remote_Stick_Scan(void)
{
	//��0.25����ΪADC��������ԭʼֵ��0-4095������4��Χ����0-1000��2000-����������ң�еļ��Է�ת����Ȼң�����������Ƿ��ģ����÷�Χ��1000-2000
	rc.THR = 2000 - (0.25f*ADC_Value[1]) - offset.THR;
	rc.YAW = 2000 - (0.25f*ADC_Value[0]) - offset.YAW;
	rc.ROL = 2000 - (0.25f*ADC_Value[2]) - offset.ROL;
	rc.PIT = 2000 - (0.25f*ADC_Value[3]) - offset.PIT;
	
	
	// ���������˲��������ݱ仯ƽ��
	App_Remote_Window_Filter(&rc);
	// У׼
	App_Remote_Mid_Offset(&rc);
	// �����С�޷�
	App_Remote_Limit(&rc);	
	// �е��޷�
	App_Remote_Mid_Limit(&rc);
}

void App_Remote_KeyPress(void)
{
	/* 
    ������־λ����=
    if(��������&& ��־λ==0)
    {
        �ӳ�һ��
        if(�������� && ��־λ==0)
        {
            �����߼���
            ��־=1��
        }

    }
    if(�����ɿ� && ��־λ=1)
    {
        ��־λ=0��
    }
 */

    /* 1���ж��ĸ��������£���Ӧ��ȥ΢�� */
    if (!READ_KEY_U)
    {
        /* ǰ���������£�΢��pitch */
        offset.PIT = offset.PIT - 10; // ǰ���ü�����pitch���=������ƫ��ֵ��С��
    }
    /* Ϊʲô����else if:�û��п��ܶ������ͬʱ�������ж���Ч�Ļ�������if */
    if (!READ_KEY_D)
    {
        /* ǰ���������£�΢��pitch */
        offset.PIT = offset.PIT + 10; // �����üӣ���pitch��С=������ƫ��ֵ���
    }
    if (!READ_KEY_L)
    {
        /* ��߰������£�΢��roll */
        offset.ROL = offset.ROL + 10; // �����üӣ���roll��С=������ƫ��ֵ���
    }
    if (!READ_KEY_R)
    {
        /* ��߰������£�΢��roll */
        offset.ROL = offset.ROL - 10; // �����ü�����roll���=������ƫ��ֵ��С��
    }
}

void App_Remote_Data(uint8_t *remote_data)
{
	uint8_t index = 0;
	uint32_t SUM = 0;
	
	remote_data[index++] = 0xAA;
	remote_data[index++] = 0XAF; //֡ͷ�����ֽ�
	
	remote_data[index++] = 0X03; //����λ
	
	remote_data[index++] = 0X00; //�ȴ��������ݳ���
	
	remote_data[index++] = (uint8_t)(rc.THR >> 8); //ң������16λ���ȴ���λ
	remote_data[index++] = (uint8_t)rc.THR;
	remote_data[index++] = (uint8_t)(rc.YAW >> 8); //ң������16λ���ȴ���λ
	remote_data[index++] = (uint8_t)rc.YAW;
	remote_data[index++] = (uint8_t)(rc.ROL >> 8); //ң������16λ���ȴ���λ
	remote_data[index++] = (uint8_t)rc.ROL;
	remote_data[index++] = (uint8_t)(rc.PIT >> 8); //ң������16λ���ȴ���λ
	remote_data[index++] = (uint8_t)rc.PIT;
	
	remote_data[index++] = 0x0;
	remote_data[index++] = 0x0;
	remote_data[index++] = 0x0;
	remote_data[index++] = 0x0;
	remote_data[index++] = 0x0;
	remote_data[index++] = 0x0;
	remote_data[index++] = 0x0;
	remote_data[index++] = 0x0;
	remote_data[index++] = 0x0;
	remote_data[index++] = 0x0;
	remote_data[index++] = 0x0;
	remote_data[index++] = 0x0;
	
	
	//���¼������ݳ���
	remote_data[3] = index - 4; // 4����֡ͷ������λ�����ݳ���λ���ܹ�8���ֽ�
	
	//У��ͣ�����SUM�ֶ�
	for(uint8_t i = 0; i < index; i++)
	{
		SUM += remote_data[i];
	}
	remote_data[index++] = (uint8_t)(SUM >> 24); // ����ֱ�ӽ�SUM�remote_data[index++]��SUM��32λ��remote_data��8λ
	remote_data[index++] = (uint8_t)(SUM >> 16);
	remote_data[index++] = (uint8_t)(SUM >> 8);
	remote_data[index++] = (uint8_t)(SUM);
	
	
}




