#include "App_Flight.h"

float bat_val = 4000; // ����mv
extern uint16_t ADC_Value[5];
sLED LED = {1000, AlwaysOff};
uint16_t led_count = 0;
uint16_t MPU_Offsets[6] = {0};
uint8_t unlock_flag = 0;

PidObject pidPitch;
PidObject pidRoll;
PidObject pidYaw;
PidObject pidRateX;
PidObject pidRateY;
PidObject pidRateZ;

PidObject *pids[] = {&pidPitch, &pidRoll, &pidYaw, &pidRateX, &pidRateY, &pidRateZ};
uint16_t motor1, motor2, motor3, motor4 = 0; // ���մ������PWM��ֵ

extern const float Gyro_G;


_stMPU MPU6050;
_stAngle Angle;
// ����ң�����ݵĽṹ��
_stRemote remote_data;

void App_PolitLED(void)
{
    static uint16_t lasttime = 0;
    uint16_t nowTime = HAL_GetTick();
    
    if(nowTime - lasttime > LED.FlashTime)
    {
        LED.status = AlwaysOn;
        lasttime = nowTime;
    }
    // ��1000ms�ڣ�LEDһֱִ������ĳ����ѳ���1000ms���ͳ�����ˢ��lasttime������lasttimeΪ��׼����1000ms
    HAL_Delay(50);
    switch(LED.status)
    {
        case AlwaysOn:
            Int_LED_On(LED1_GPIO_Port, LED1_Pin);
            Int_LED_On(LED2_GPIO_Port, LED2_Pin);
            Int_LED_On(LED3_GPIO_Port, LED3_Pin);
            Int_LED_On(LED4_GPIO_Port, LED4_Pin);
            break;
        case AlwaysOff:
            Int_LED_Off(LED1_GPIO_Port, LED1_Pin);
            Int_LED_Off(LED2_GPIO_Port, LED2_Pin);
            Int_LED_Off(LED3_GPIO_Port, LED3_Pin);
            Int_LED_Off(LED4_GPIO_Port, LED4_Pin);
            break;
        case AllFlashLight:
            Int_LED_Toggle(LED1_GPIO_Port, LED1_Pin);
            Int_LED_Toggle(LED2_GPIO_Port, LED2_Pin);
            Int_LED_Toggle(LED3_GPIO_Port, LED3_Pin);
            Int_LED_Toggle(LED4_GPIO_Port, LED4_Pin);
            break;
        case AlternateFlash:
            /* ǰ������˸�������ó�ʼ״̬ */
            Int_LED_On(LED1_GPIO_Port, LED1_Pin);
            Int_LED_On(LED2_GPIO_Port, LED2_Pin);
            Int_LED_Off(LED3_GPIO_Port, LED3_Pin);
            Int_LED_Off(LED4_GPIO_Port, LED4_Pin);
            LED.status = AllFlashLight; // �Գ�ʼ״̬��ʼ��˸
            break;
        case WARNING:
            /* �Խ��� ������˸ */
            Int_LED_On(LED1_GPIO_Port, LED1_Pin);
            Int_LED_On(LED3_GPIO_Port, LED3_Pin);
            Int_LED_Off(LED2_GPIO_Port, LED2_Pin);
            Int_LED_Off(LED4_GPIO_Port, LED4_Pin);
            LED.status = AllFlashLight; // �Գ�ʼ״̬��ʼ��˸
            break;
        case DANGEROURS:
            /* ����һ�ᣬ��˸һ�� */
            led_count++;
            if (led_count < 10)
            {
                Int_LED_On(LED1_GPIO_Port, LED1_Pin);
                Int_LED_On(LED2_GPIO_Port, LED2_Pin);
                Int_LED_On(LED3_GPIO_Port, LED3_Pin);
                Int_LED_On(LED4_GPIO_Port, LED4_Pin);
            }
            else if (led_count < 12)
            {
                Int_LED_Toggle(LED1_GPIO_Port, LED1_Pin);
                Int_LED_Toggle(LED2_GPIO_Port, LED2_Pin);
                Int_LED_Toggle(LED3_GPIO_Port, LED3_Pin);
                Int_LED_Toggle(LED4_GPIO_Port, LED4_Pin);
            }
            else
            {
                led_count = 0;
            }
            break;

    }
}


/*��ص�ѹ���*/
void App_Flight_Volcheck(void)
{
    // ��ѹֵ=0.2 * ��ǰ��ѹֵ + 0.8 * ֮ǰ��ѹֵ
    /*���Զ�ȡ��ѹ*/
    //HAL_ADC_GetValue(); // ��ʹ��DMA��ʱ�����
    //bat_val = ADC_Value[0] * 3.3 / 4095 * 2;
    bat_val += 0.2 * ((ADC_Value[0] * 3300 / 4095 * 2) - bat_val);
    printf("��ص�ѹ=%.1f\r\n", bat_val);
}

/*��ȡMPU����ԭʼ���ݣ������ŷ����*/
void App_Flight_MPU_Data(void)
{
    /*1.��ȡԭʼ����*/
    Int_MPU6050_GetAccl(&MPU6050.accX, &MPU6050.accY,&MPU6050.accZ);
    Int_MPU6050_GetGyro(&MPU6050.gryoX,&MPU6050.gryoY,&MPU6050.gryoZ);
//    printf("============MPU��ʼֵ================\r\n");
//    printf("accX=%d\r\n",MPU6050.accX);
//    printf("accY=%d\r\n",MPU6050.accY);
//    printf("accZ=%d\r\n",MPU6050.accZ);
//    printf("gryoX=%d\r\n",MPU6050.gryoX);
//    printf("gryoY=%d\r\n",MPU6050.gryoY);
//    printf("gryoZ=%d\r\n",MPU6050.gryoZ);
    
    /*У׼*/
    MPU6050.accX = MPU6050.accX - MPU_Offsets[0];
    MPU6050.accY = MPU6050.accY - MPU_Offsets[1];
    MPU6050.accZ = MPU6050.accZ - MPU_Offsets[2];
    MPU6050.gryoX = MPU6050.gryoX - MPU_Offsets[3];
    MPU6050.gryoY = MPU6050.gryoY - MPU_Offsets[4];
    MPU6050.gryoZ = MPU6050.gryoZ - MPU_Offsets[5];
    
    /*2. �Լ��ٶȽ��м��׵�һά�������˲�*/
    Com_Kalman_1(&ekf[0], MPU6050.accX);
    MPU6050.accX = (int16_t)ekf[0].out;
    Com_Kalman_1(&ekf[1], MPU6050.accY);
    MPU6050.accY = (int16_t)ekf[1].out;
    Com_Kalman_1(&ekf[2], MPU6050.accZ);
    MPU6050.accZ = (int16_t)ekf[2].out; // ת��int16_t����ΪaccXYZ������int16_t
    /*3. �Խ��ٶȽ��м򵥵�һ�׵�ͨ�˲�*/
    static short lastGryo[3] = {0};
    MPU6050.gryoX = 0.85 * lastGryo[0] + 0.15 * MPU6050.gryoX;
    lastGryo[0] = MPU6050.gryoX;
    MPU6050.gryoY = 0.85 * lastGryo[1] + 0.15 * MPU6050.gryoY;
    lastGryo[0] = MPU6050.gryoY;
    MPU6050.gryoZ = 0.85 * lastGryo[2] + 0.15 * MPU6050.gryoZ;
    lastGryo[0] = MPU6050.gryoZ;
    
//    printf("============MPU�˲����ֵ================\r\n");
//    printf("accX=%d\r\n",MPU6050.accX);
//    printf("accY=%d\r\n",MPU6050.accY);
//    printf("accZ=%d\r\n",MPU6050.accZ);
//    printf("gryoX=%d\r\n",MPU6050.gryoX);
//    printf("gryoY=%d\r\n",MPU6050.gryoY);
//    printf("gryoZ=%d\r\n",MPU6050.gryoZ);
    /*4. ����Ƕȣ���Ԫ������*/
    
}

void App_Flight_MPU_Offsets(void)
{
    /*�жϴ������Ƿ��ھ���״̬��ȡ�����ǽ��ٶȣ����εĲ�ֵ��һ����Χ�ڣ�����Ϊ�Ǿ�ֹ��*/
    const int8_t MAX_GRYO_ERR = 5;
    const int8_t MIN_GRYO_ERR = -5;
    int16_t last_gryo[3] = {0}; // ��һ�������ǵ�ֵ
    int16_t err_gryo[3] = {0}; // ������ƫ��ֵ
    int32_t buff[6] = {0};
    /*�����ж�30���������Ƿ�ֹ�����һֱ����ֹ�ͻῨ������ѭ��*/
    uint8_t gryo_i = 30;
    while(gryo_i --)
    {
        do
        {
            HAL_Delay(10);
            // ȡ����ֵ
            App_Flight_MPU_Data();
            // ����ƫ��ֵ
            err_gryo[0] = MPU6050.gryoX - last_gryo[0];
            err_gryo[1] = MPU6050.gryoY - last_gryo[1];
            err_gryo[2] = MPU6050.gryoZ - last_gryo[2];
            // ����
            last_gryo[0] = MPU6050.gryoX;
            last_gryo[1] = MPU6050.gryoY;
            last_gryo[2] = MPU6050.gryoZ;
        }
        while // һֱ���������Сƫ��ֵ��һֱ���㣬֪����ֹ�Ƴ�
        (err_gryo[0] < MIN_GRYO_ERR || err_gryo[0] > MAX_GRYO_ERR ||
         err_gryo[1] < MIN_GRYO_ERR || err_gryo[1] > MAX_GRYO_ERR ||
         err_gryo[2] < MIN_GRYO_ERR || err_gryo[2] > MAX_GRYO_ERR 
        );
    }
    
    
    /*����ֹ��ȡ���ԭʼֵ������ƽ��ֵ*/
    for(uint16_t i = 0; i < 356; i++)
    {
        /* ����MPU��ֵ */
        App_Flight_MPU_Data();
        if(i >= 100)
        {
            buff[0] += MPU6050.accX;
            buff[1] += MPU6050.accY;
            buff[2] += MPU6050.accZ - 16384; // ��1�������ٶ�g��16384���Ļ�׼�ϼ���ƫ��
            buff[3] += MPU6050.gryoX;
            buff[4] += MPU6050.gryoY;
            buff[5] += MPU6050.gryoZ;
        }
    }
    /* ����256�Σ�ֱ������8λ */
    for(uint8_t i = 0; i < 6; i++)
    {
        MPU_Offsets[i] = buff[i] >> 8;
    }
}


/*����ң�ط��������ݰ�*/
void App_Flight_Remote_Check(uint8_t *buf, uint8_t len)
{
//	printf("connect_flag:%d\r\n", connect_flag);
	connect_flag++;
    /* �����պ���ִ����֮��ִ�иú��������������flag++��һ��=1������1�������� */
	if(connect_flag == 1)
	{
		uint32_t origin_sum = 0, flight_sum = 0;
		// У��֡ͷ
		if(!(buf[0] == 0xAA && buf[1] == 0xAF)) // ����ȷ���˳�if��ֻҪ��һ�����ԣ��ͽ��룬ֱ�ӷ��غ���
		{
			printf("֡ͷ����\r\n");
			return; // ���ݲ���
		}
		// У�鹦��λ
		if(!(buf[2] == 0x03))
		{
			printf("����λ����\r\n");
			return; //���ݲ���
		}
		//У���λУ��
		// ��ȡУ���
		origin_sum = (buf[len - 4] << 24) | (buf[len - 3] << 16) | (buf[len - 2] << 8) | (buf[len - 1]); // Ҫ��|���������ǡ�+����λ������ǰ��Ჹ0
	
		// �Լ����㴫�������ݵ�У���
		for(uint8_t i = 0; i < len - 4; i++) // -4����ΪУ���SUM��32λ4���ֽ�
		{
			flight_sum += buf[i];
		}
		// �ж��Ƿ����
		if(origin_sum != flight_sum)
		{
			printf("У��ʹ���\r\n");
			return; // ���ݲ���
		}
	
		// �жϹ����֣���������ȡ��
		if(buf[2] == 0x03)
		{
			remote_data.THR = (int16_t)((buf[4] << 8) | buf[5]);
			remote_data.YAW = (int16_t)((buf[6] << 8) | buf[7]);
			remote_data.ROL = (int16_t)((buf[8] << 8) | buf[9]);
			remote_data.PIT = (int16_t)((buf[10] << 8) | buf[11]);
			remote_data.AUX1 = (int16_t)((buf[12] << 8) | buf[13]);
			remote_data.AUX2 = (int16_t)((buf[14] << 8) | buf[15]);
			remote_data.AUX3 = (int16_t)((buf[16] << 8) | buf[17]);
			remote_data.AUX4 = (int16_t)((buf[18] << 8) | buf[19]);
			remote_data.AUX5 = (int16_t)((buf[20] << 8) | buf[21]);
			remote_data.AUX6 = (int16_t)((buf[22] << 8) | buf[23]);
		
		/* ====================���ԣ���ӡ�����������===================== */
			printf("��ȡ��ֵ��\r\n");
			printf("THR=%d\r\n", remote_data.THR);
			printf("YAW=%d\r\n", remote_data.YAW);
			printf("ROL=%d\r\n", remote_data.ROL);
			printf("PIT=%d\r\n", remote_data.PIT);
		}
	}
	if(connect_flag > 5000)
	{
		/* ����ʱ��ʧ����flag��ӵ�һ���ܴ������Ϊ�˱���Խ�磬��һ������������Ϊ1��������0����Ϊ����ʧ���� */
        /* ����Խ��������͸�λֵ��ע��ο����� ʧ�����������ж�ʱ�� */
		connect_flag = 1251;
	}
}


void App_Flight_RC_Unlock(void)
{
	static uint8_t status = WAITING_1;
	static uint16_t unlock_count = 0;
	
	if (status == ENMERGENCY_0)
    {
        /* Ϊ�˰�ȫ����һ���жϣ�����ǽ���״̬��ֱ���˳� */
        status = EXIT;
    }
	
	switch(status)
	{
		case WAITING_1:
			if(remote_data.THR < 1030) // ��д̫��
			{
				printf("=========W1=========\r\n");
				status = WAITING_2;
			}
			break;
		case WAITING_2:
			if(remote_data.THR > 1950)
			{
				printf("=========W2=========\r\n");
				status = WAITING_3;
			}
			break;
		case WAITING_3:
			if(remote_data.THR < 1030) // ��д̫��
			{
				printf("=========W3=========\r\n");
				status = WAITING_4;
			}
			break;
		case WAITING_4:
			printf("=========W4=========\r\n");
			unlock_flag = 1; // ����
			status = PROCESS;
			break;
		case PROCESS:
			printf("=========P=========\r\n");
			if(remote_data.THR < 1030 && unlock_count++ > 1000) // �ɹ�������ʱ��û�ж�ҡ�ˣ�������
			{
				unlock_flag = 0;
				status = WAITING_1;
				unlock_count = 0;
			}
			/* �����־λ�������ط������㣬ֱ���˳����ƣ������˳�״̬ */
			if (!unlock_flag)
			{
				status = EXIT;
			}
			break;
		case EXIT:
			printf("!!!!!!!!!!!!!!!!! exit\r\n");
			unlock_flag = 0;
			unlock_count = 0;
			status = WAITING_1;
			break;
		default:
			status = EXIT;
	
	}
}

void App_Flight_RC_Analysis(void)
{
	static uint16_t thr_count = 0;
	if(connect_flag == 1) // �յ����ݣ�connect_flag��ʼΪ0���������ݺ�connect_flagΪ1
	{
		App_Flight_RC_Unlock();//�ж��Ƿ����
	}
	else
	{
		/* ʧ��: ����3s���ж�ʧ�������д��� */
		if(connect_flag > 1250)
		{
			printf("ʧ��������������\r\n");
			//ҡ�˻ص��е�
			remote_data.PIT = 1500;
			remote_data.ROL = 1500;
			remote_data.YAW = 1500;
			
			if(remote_data.THR < 1200) // ʧ��ʱ���ű�����С����ֱ��ͣ
			{
				remote_data.THR = 1000;
				printf("����2.4G!!!!!!!\r\n");
				while(NRF24L01_Check()); // ��������
				NRF24L01_RX_Mode();
				printf("����2.4G�ɹ�!!!!!!!\r\n");
			}
			else
			{
				// ���ʧ��ʱ���źܴ���ֱ��ͣ��������С��ֱ����С����������if
				if (thr_count++ > 100) // ����10msִ��һ�Σ�100��=1s��ÿ1s��С50
                {
                    printf("���ţ�%d -50\r\n", remote_data.THR);
                    remote_data.THR -= 50;
                    thr_count = 0;
                } 
			}
			remote_data.THR = LIMIT(remote_data.THR, 1000, 2000); // ��ֹ������ֵԽ��
		}	
	}
}

void App_Flight_PID_Control(float dt)
{
    static uint8_t status = WAITING_1;

    switch (status)
    {
    case WAITING_1: /* 1���׶�һ�����ݱ�־λ �ж��Ƿ�����������׶�2 */
        /* code */
        if (unlock_flag == 1)
        {
            status = WAITING_2;
        }
        break;
    case WAITING_2: /* 2���׶ζ�����λPID��������ʽ�׶� */
        /* code */
        ResetPID(pids, 6);
        status = PROCESS;
        break;
    case PROCESS: /* 3����ʽ�׶Σ� PID���� */
        /* ��ֵ�ǶȵĲ���ֵ */
        pidPitch.measured = Angle.pitch;
        pidRoll.measured = Angle.roll;
        pidYaw.measured = Angle.yaw;
        /* ��ֵ���ٶȵĲ���ֵ */
        pidRateX.measured = MPU6050.gryoX * Gyro_G;
        pidRateY.measured = MPU6050.gryoY * Gyro_G;
        pidRateZ.measured = MPU6050.gryoZ * Gyro_G;
        /*
            ������ ---�� Y����ٶ�
            ����� ---�� X����ٶ�
            ƫ���� ---�� Z����ٶ�
         */
        CasecadePID(&pidPitch, &pidRateY, dt);
        CasecadePID(&pidRoll, &pidRateX, dt);
        CasecadePID(&pidYaw, &pidRateZ, dt);

        break;

    default:
        break;
    }
}

// �������
void App_Flight_Motor_Control(void)
{
	static uint8_t status = WAITING_1; /// һ��Ҫstatic��������Ȼÿ�ν��⺯��status�������±�����ΪWAITING_1
	switch(status)
	{
		case WAITING_1:
			motor1 = 0;
			motor2 = 0;
			motor3 = 0;
			motor4 = 0;
			if(unlock_flag == 1) // ����Ѿ��������ͽ���ڶ��׶�
			{
				status = WAITING_2 ;
			}
			break;
		case WAITING_2:
			if(remote_data.THR > 1100) //������Ŵ���1100˵���ڲٿأ���ʽ������ƽ׶�
			{
				status = PROCESS;
			}
			break;
		case PROCESS:
			int16_t thr_temp;
			thr_temp = remote_data.THR - 1000; // ��������ֵӳ�䵽0-1000
			if(remote_data.THR < 1020) // �ڲ��������У���������ǳ��;�ֱ����0����ֹ����
			{
				motor1 = motor2 = motor3 = motor4 = 0;
				break;
			}
			/* 3�� Ԥ��100��PID���� */
			motor1 = motor2 = motor3 = motor4 = LIMIT(thr_temp, 0, 900);

			/* 4������3��PID��ֵ */
			motor1 += +pidRateX.out + pidRateY.out + pidRateZ.out; // �Һ�
			motor2 += +pidRateX.out - pidRateY.out - pidRateZ.out; // ��ǰ
			motor3 += -pidRateX.out - pidRateY.out + pidRateZ.out; // ��ǰ
			motor4 += -pidRateX.out + pidRateY.out - pidRateZ.out; // ���
			break;

		default:
			break;	
	}
	// ���õ��
//	 printf("motor1=%d,motor2=%d,motor3=%d,motor4=%d\r\n",motor1,motor2,motor3,motor4);
    /* ����pwm */
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, LIMIT(motor3,0,1000)); // ��ǰ
    __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, LIMIT(motor2,0,1000)); // ��ǰ
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_4, LIMIT(motor4,0,1000)); // ���
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, LIMIT(motor1,0,1000)); // �Һ�
}


/**
 * @description: ҡ�˿��Ʒɻ�ǰ�����ˡ������ƶ�
 * @return {*}
 */
void App_Flight_Mode_Control(void)
{
    const float roll_pitch_ratio = 0.04f;
    /* PID������÷ɻ�����ƽ�⣬Ҫ�÷ɻ��ƶ���Ҫ�ýǶȻ�������ֵ���� */
    pidPitch.desired = -(remote_data.PIT - 1500) * roll_pitch_ratio; // ҡ�˿���
    pidRoll.desired = -(remote_data.ROL - 1500) * roll_pitch_ratio;   // ҡ�˿���
    Angle.yaw = pidYaw.desired = pidYaw.measured = 0; // ����ƫ����
}


/**
 * @description: ��ʼ��PIDϵ��
 * @return {*}
 */
void App_PID_Param_Init(void)
{
    /* �ڻ� */
    /*
        �����ǣ� �ڻ� Y����ٶ�
        ����ǣ� �ڻ� X����ٶ�
        ƫ���ǣ� �ڻ� Z����ٶ�
     */
    pidRateX.kp = -2.0f; // -3.0
    pidRateY.kp = 2.0f;  // 2.0
    pidRateZ.kp = -3.0f; // -2.0

    pidRateX.ki = 0.0f;
    pidRateY.ki = 0.0f;
    pidRateZ.ki = 0.0f;

    pidRateX.kd = -0.08f; //-0.08
    pidRateY.kd = 0.08f;  // 0.08
    pidRateZ.kd = 0.00f;

    /* �⻷ */
    pidPitch.kp = 7.0f; // 7.0
    pidRoll.kp = 7.0f;  //
    pidYaw.kp = -4.0f;

    pidPitch.ki = 0.0f;
    pidRoll.ki = 0.0f;
    pidYaw.ki = 0.0f;

    pidPitch.kd = 0.0f;
    pidRoll.kd = 0.0f;
    pidYaw.kd = 0.0f;
}

