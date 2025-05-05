#include "App_Flight.h"

float bat_val = 4000; // 毫伏mv
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
uint16_t motor1, motor2, motor3, motor4 = 0; // 最终传给电机PWM的值

extern const float Gyro_G;


_stMPU MPU6050;
_stAngle Angle;
// 接受遥控数据的结构体
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
    // 在1000ms内，LED一直执行下面的程序，已超过1000ms，就常亮并刷新lasttime，再以lasttime为基准持续1000ms
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
            /* 前后交替闪烁，先设置初始状态 */
            Int_LED_On(LED1_GPIO_Port, LED1_Pin);
            Int_LED_On(LED2_GPIO_Port, LED2_Pin);
            Int_LED_Off(LED3_GPIO_Port, LED3_Pin);
            Int_LED_Off(LED4_GPIO_Port, LED4_Pin);
            LED.status = AllFlashLight; // 以初始状态开始闪烁
            break;
        case WARNING:
            /* 对角线 交替闪烁 */
            Int_LED_On(LED1_GPIO_Port, LED1_Pin);
            Int_LED_On(LED3_GPIO_Port, LED3_Pin);
            Int_LED_Off(LED2_GPIO_Port, LED2_Pin);
            Int_LED_Off(LED4_GPIO_Port, LED4_Pin);
            LED.status = AllFlashLight; // 以初始状态开始闪烁
            break;
        case DANGEROURS:
            /* 长亮一会，闪烁一下 */
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


/*电池电压检测*/
void App_Flight_Volcheck(void)
{
    // 电压值=0.2 * 当前电压值 + 0.8 * 之前电压值
    /*测试读取电压*/
    //HAL_ADC_GetValue(); // 不使用DMA的时候调用
    //bat_val = ADC_Value[0] * 3.3 / 4095 * 2;
    bat_val += 0.2 * ((ADC_Value[0] * 3300 / 4095 * 2) - bat_val);
    printf("电池电压=%.1f\r\n", bat_val);
}

/*获取MPU六轴原始数据，计算出欧拉角*/
void App_Flight_MPU_Data(void)
{
    /*1.获取原始数据*/
    Int_MPU6050_GetAccl(&MPU6050.accX, &MPU6050.accY,&MPU6050.accZ);
    Int_MPU6050_GetGyro(&MPU6050.gryoX,&MPU6050.gryoY,&MPU6050.gryoZ);
//    printf("============MPU初始值================\r\n");
//    printf("accX=%d\r\n",MPU6050.accX);
//    printf("accY=%d\r\n",MPU6050.accY);
//    printf("accZ=%d\r\n",MPU6050.accZ);
//    printf("gryoX=%d\r\n",MPU6050.gryoX);
//    printf("gryoY=%d\r\n",MPU6050.gryoY);
//    printf("gryoZ=%d\r\n",MPU6050.gryoZ);
    
    /*校准*/
    MPU6050.accX = MPU6050.accX - MPU_Offsets[0];
    MPU6050.accY = MPU6050.accY - MPU_Offsets[1];
    MPU6050.accZ = MPU6050.accZ - MPU_Offsets[2];
    MPU6050.gryoX = MPU6050.gryoX - MPU_Offsets[3];
    MPU6050.gryoY = MPU6050.gryoY - MPU_Offsets[4];
    MPU6050.gryoZ = MPU6050.gryoZ - MPU_Offsets[5];
    
    /*2. 对加速度进行简易的一维卡尔曼滤波*/
    Com_Kalman_1(&ekf[0], MPU6050.accX);
    MPU6050.accX = (int16_t)ekf[0].out;
    Com_Kalman_1(&ekf[1], MPU6050.accY);
    MPU6050.accY = (int16_t)ekf[1].out;
    Com_Kalman_1(&ekf[2], MPU6050.accZ);
    MPU6050.accZ = (int16_t)ekf[2].out; // 转成int16_t是因为accXYZ定义是int16_t
    /*3. 对角速度进行简单的一阶低通滤波*/
    static short lastGryo[3] = {0};
    MPU6050.gryoX = 0.85 * lastGryo[0] + 0.15 * MPU6050.gryoX;
    lastGryo[0] = MPU6050.gryoX;
    MPU6050.gryoY = 0.85 * lastGryo[1] + 0.15 * MPU6050.gryoY;
    lastGryo[0] = MPU6050.gryoY;
    MPU6050.gryoZ = 0.85 * lastGryo[2] + 0.15 * MPU6050.gryoZ;
    lastGryo[0] = MPU6050.gryoZ;
    
//    printf("============MPU滤波后的值================\r\n");
//    printf("accX=%d\r\n",MPU6050.accX);
//    printf("accY=%d\r\n",MPU6050.accY);
//    printf("accZ=%d\r\n",MPU6050.accZ);
//    printf("gryoX=%d\r\n",MPU6050.gryoX);
//    printf("gryoY=%d\r\n",MPU6050.gryoY);
//    printf("gryoZ=%d\r\n",MPU6050.gryoZ);
    /*4. 计算角度：四元数解算*/
    
}

void App_Flight_MPU_Offsets(void)
{
    /*判断传感器是否处于精致状态，取陀螺仪角速度，两次的差值在一定范围内，就认为是静止的*/
    const int8_t MAX_GRYO_ERR = 5;
    const int8_t MIN_GRYO_ERR = -5;
    int16_t last_gryo[3] = {0}; // 上一次陀螺仪的值
    int16_t err_gryo[3] = {0}; // 陀螺仪偏差值
    int32_t buff[6] = {0};
    /*连续判断30次陀螺仪是否静止，如果一直不静止就会卡死在内循环*/
    uint8_t gryo_i = 30;
    while(gryo_i --)
    {
        do
        {
            HAL_Delay(10);
            // 取六轴值
            App_Flight_MPU_Data();
            // 计算偏差值
            err_gryo[0] = MPU6050.gryoX - last_gryo[0];
            err_gryo[1] = MPU6050.gryoY - last_gryo[1];
            err_gryo[2] = MPU6050.gryoZ - last_gryo[2];
            // 更新
            last_gryo[0] = MPU6050.gryoX;
            last_gryo[1] = MPU6050.gryoY;
            last_gryo[2] = MPU6050.gryoZ;
        }
        while // 一直大于最大最小偏差值就一直计算，知道静止推出
        (err_gryo[0] < MIN_GRYO_ERR || err_gryo[0] > MAX_GRYO_ERR ||
         err_gryo[1] < MIN_GRYO_ERR || err_gryo[1] > MAX_GRYO_ERR ||
         err_gryo[2] < MIN_GRYO_ERR || err_gryo[2] > MAX_GRYO_ERR 
        );
    }
    
    
    /*机身静止后，取多次原始值，计算平均值*/
    for(uint16_t i = 0; i < 356; i++)
    {
        /* 更新MPU的值 */
        App_Flight_MPU_Data();
        if(i >= 100)
        {
            buff[0] += MPU6050.accX;
            buff[1] += MPU6050.accY;
            buff[2] += MPU6050.accZ - 16384; // 在1重力加速度g（16384）的基准上计算偏差
            buff[3] += MPU6050.gryoX;
            buff[4] += MPU6050.gryoY;
            buff[5] += MPU6050.gryoZ;
        }
    }
    /* 除以256次，直接右移8位 */
    for(uint8_t i = 0; i < 6; i++)
    {
        MPU_Offsets[i] = buff[i] >> 8;
    }
}


/*解析遥控发来的数据包*/
void App_Flight_Remote_Check(uint8_t *buf, uint8_t len)
{
//	printf("connect_flag:%d\r\n", connect_flag);
	connect_flag++;
    /* ！接收函数执行完之后，执行该函数，如果正常，flag++后一定=1，除了1都不正常 */
	if(connect_flag == 1)
	{
		uint32_t origin_sum = 0, flight_sum = 0;
		// 校验帧头
		if(!(buf[0] == 0xAA && buf[1] == 0xAF)) // 都正确就退出if，只要有一个不对，就进入，直接返回函数
		{
			printf("帧头错误\r\n");
			return; // 数据不对
		}
		// 校验功能位
		if(!(buf[2] == 0x03))
		{
			printf("功能位错误\r\n");
			return; //数据不对
		}
		//校验和位校验
		// 获取校验和
		origin_sum = (buf[len - 4] << 24) | (buf[len - 3] << 16) | (buf[len - 2] << 8) | (buf[len - 1]); // 要“|”，不能是“+”，位数不够前面会补0
	
		// 自己计算传来的数据的校验和
		for(uint8_t i = 0; i < len - 4; i++) // -4是因为校验和SUM是32位4个字节
		{
			flight_sum += buf[i];
		}
		// 判断是否相等
		if(origin_sum != flight_sum)
		{
			printf("校验和错误\r\n");
			return; // 数据不对
		}
	
		// 判断功能字，并将数据取出
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
		
		/* ====================测试：打印解析完的数据===================== */
			printf("获取的值：\r\n");
			printf("THR=%d\r\n", remote_data.THR);
			printf("YAW=%d\r\n", remote_data.YAW);
			printf("ROL=%d\r\n", remote_data.ROL);
			printf("PIT=%d\r\n", remote_data.PIT);
		}
	}
	if(connect_flag > 5000)
	{
		/* ！长时间失联，flag会加到一个很大的数，为了避免越界，到一定数，重新置为1（不能是0，因为还是失联） */
        /* 这里越界的条件和复位值，注意参考后面 失联处理函数的判断时间 */
		connect_flag = 1251;
	}
}


void App_Flight_RC_Unlock(void)
{
	static uint8_t status = WAITING_1;
	static uint16_t unlock_count = 0;
	
	if (status == ENMERGENCY_0)
    {
        /* 为了安全，加一道判断，如果是紧急状态，直接退出 */
        status = EXIT;
    }
	
	switch(status)
	{
		case WAITING_1:
			if(remote_data.THR < 1030) // 不写太死
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
			if(remote_data.THR < 1030) // 不写太死
			{
				printf("=========W3=========\r\n");
				status = WAITING_4;
			}
			break;
		case WAITING_4:
			printf("=========W4=========\r\n");
			unlock_flag = 1; // 解锁
			status = PROCESS;
			break;
		case PROCESS:
			printf("=========P=========\r\n");
			if(remote_data.THR < 1030 && unlock_count++ > 1000) // 成功解锁后长时间没有动摇杆，又锁上
			{
				unlock_flag = 0;
				status = WAITING_1;
				unlock_count = 0;
			}
			/* 如果标志位在其他地方被清零，直接退出控制，进入退出状态 */
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
	if(connect_flag == 1) // 收到数据，connect_flag初始为0，解析数据后connect_flag为1
	{
		App_Flight_RC_Unlock();//判断是否解锁
	}
	else
	{
		/* 失联: 超过3s，判定失联，进行处理 */
		if(connect_flag > 1250)
		{
			printf("失联！！！！！！\r\n");
			//摇杆回到中点
			remote_data.PIT = 1500;
			remote_data.ROL = 1500;
			remote_data.YAW = 1500;
			
			if(remote_data.THR < 1200) // 失联时油门本来就小，就直接停
			{
				remote_data.THR = 1000;
				printf("重启2.4G!!!!!!!\r\n");
				while(NRF24L01_Check()); // 尝试重连
				NRF24L01_RX_Mode();
				printf("重启2.4G成功!!!!!!!\r\n");
			}
			else
			{
				// 如果失联时油门很大不能直接停，慢慢减小，直到减小到进入上面if
				if (thr_count++ > 100) // 假设10ms执行一次，100次=1s，每1s减小50
                {
                    printf("油门：%d -50\r\n", remote_data.THR);
                    remote_data.THR -= 50;
                    thr_count = 0;
                } 
			}
			remote_data.THR = LIMIT(remote_data.THR, 1000, 2000); // 防止油门数值越界
		}	
	}
}

void App_Flight_PID_Control(float dt)
{
    static uint8_t status = WAITING_1;

    switch (status)
    {
    case WAITING_1: /* 1、阶段一：根据标志位 判断是否解锁，则进入阶段2 */
        /* code */
        if (unlock_flag == 1)
        {
            status = WAITING_2;
        }
        break;
    case WAITING_2: /* 2、阶段二：复位PID，进入正式阶段 */
        /* code */
        ResetPID(pids, 6);
        status = PROCESS;
        break;
    case PROCESS: /* 3、正式阶段： PID计算 */
        /* 赋值角度的测量值 */
        pidPitch.measured = Angle.pitch;
        pidRoll.measured = Angle.roll;
        pidYaw.measured = Angle.yaw;
        /* 赋值角速度的测量值 */
        pidRateX.measured = MPU6050.gryoX * Gyro_G;
        pidRateY.measured = MPU6050.gryoY * Gyro_G;
        pidRateZ.measured = MPU6050.gryoZ * Gyro_G;
        /*
            俯仰角 ---》 Y轴角速度
            横滚角 ---》 X轴角速度
            偏航角 ---》 Z轴角速度
         */
        CasecadePID(&pidPitch, &pidRateY, dt);
        CasecadePID(&pidRoll, &pidRateX, dt);
        CasecadePID(&pidYaw, &pidRateZ, dt);

        break;

    default:
        break;
    }
}

// 电机控制
void App_Flight_Motor_Control(void)
{
	static uint8_t status = WAITING_1; /// 一定要static！！！不然每次进这函数status都会重新被设置为WAITING_1
	switch(status)
	{
		case WAITING_1:
			motor1 = 0;
			motor2 = 0;
			motor3 = 0;
			motor4 = 0;
			if(unlock_flag == 1) // 如果已经解锁，就进入第二阶段
			{
				status = WAITING_2 ;
			}
			break;
		case WAITING_2:
			if(remote_data.THR > 1100) //如果油门大于1100说明在操控，正式进入控制阶段
			{
				status = PROCESS;
			}
			break;
		case PROCESS:
			int16_t thr_temp;
			thr_temp = remote_data.THR - 1000; // 将油门数值映射到0-1000
			if(remote_data.THR < 1020) // 在操作过程中，油门如果非常低就直接清0，防止意外
			{
				motor1 = motor2 = motor3 = motor4 = 0;
				break;
			}
			/* 3、 预留100给PID控制 */
			motor1 = motor2 = motor3 = motor4 = LIMIT(thr_temp, 0, 900);

			/* 4、加上3个PID的值 */
			motor1 += +pidRateX.out + pidRateY.out + pidRateZ.out; // 右后
			motor2 += +pidRateX.out - pidRateY.out - pidRateZ.out; // 右前
			motor3 += -pidRateX.out - pidRateY.out + pidRateZ.out; // 左前
			motor4 += -pidRateX.out + pidRateY.out - pidRateZ.out; // 左后
			break;

		default:
			break;	
	}
	// 设置电机
//	 printf("motor1=%d,motor2=%d,motor3=%d,motor4=%d\r\n",motor1,motor2,motor3,motor4);
    /* 设置pwm */
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, LIMIT(motor3,0,1000)); // 左前
    __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, LIMIT(motor2,0,1000)); // 右前
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_4, LIMIT(motor4,0,1000)); // 左后
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, LIMIT(motor1,0,1000)); // 右后
}


/**
 * @description: 摇杆控制飞机前进后退、左右移动
 * @return {*}
 */
void App_Flight_Mode_Control(void)
{
    const float roll_pitch_ratio = 0.04f;
    /* PID计算会让飞机保持平衡，要让飞机移动需要让角度环的期望值非零 */
    pidPitch.desired = -(remote_data.PIT - 1500) * roll_pitch_ratio; // 摇杆控制
    pidRoll.desired = -(remote_data.ROL - 1500) * roll_pitch_ratio;   // 摇杆控制
    Angle.yaw = pidYaw.desired = pidYaw.measured = 0; // 锁定偏航角
}


/**
 * @description: 初始化PID系数
 * @return {*}
 */
void App_PID_Param_Init(void)
{
    /* 内环 */
    /*
        俯仰角： 内环 Y轴角速度
        横滚角： 内环 X轴角速度
        偏航角： 内环 Z轴角速度
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

    /* 外环 */
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

