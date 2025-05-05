#include "App_Remote.h"

struct _Rc rc;
struct _Offset offset;
extern uint16_t ADC_Value[4];
struct _Filter Filter_Thr, Filter_Pitch, Filter_Roll, Filter_Yaw;


/**
 * @description: 取最近N次的平均值
 * @param {_Rc} *rc
 * @return {*}
 */
void App_Remote_Window_Filter(struct _Rc *rc)
{
    static uint16_t count = 0;
    /* 1、丢掉旧数据 */
    Filter_Thr.sum -= Filter_Thr.old[count];
    Filter_Pitch.sum -= Filter_Pitch.old[count];
    Filter_Roll.sum -= Filter_Roll.old[count];
    Filter_Yaw.sum -= Filter_Yaw.old[count];

    /* 2、加上新数据:新数据要加到sum里，同时也要存到old里 */
    Filter_Thr.old[count] = rc->THR;
    Filter_Pitch.old[count] = rc->PIT;
    Filter_Roll.old[count] = rc->ROL;
    Filter_Yaw.old[count] = rc->YAW;

    Filter_Thr.sum += Filter_Thr.old[count];
    Filter_Pitch.sum += Filter_Pitch.old[count];
    Filter_Roll.sum += Filter_Roll.old[count];
    Filter_Yaw.sum += Filter_Yaw.old[count];

    /* 3、求平均值 */
    rc->THR = Filter_Thr.sum / Filter_Num;
    rc->PIT = Filter_Pitch.sum / Filter_Num;
    rc->ROL = Filter_Roll.sum / Filter_Num;
    rc->YAW = Filter_Yaw.sum / Filter_Num;

    /* 4、判断索引值的范围 */
    count++;
    if (count >= 10)
    {
        count = 0;
    }
}

void App_Remote_Mid_Offset(struct _Rc *rc)
{
	static uint16_t key_count = 0; // 记录按键长按的时间
	static uint32_t sum_thr = 0, sum_yaw = 0, sum_rol = 0, sum_pit = 0, count = 0; // 记录N次的摇杆值的和，以及目前记录的次数count
	if(!READ_KEY_LEFT_X) // 读案件，按下为低电平
	{
		key_count++; // 防止误触，设置长按进入校准
		if(key_count>=20)
		{
			printf("start offset...\r\n");
			if(count == 0) // 第一次进入，初始化
			{
				sum_thr = 0;
				sum_yaw = 0;
				sum_rol = 0; 
				sum_pit = 0; 
				count = 1; // 防止在进入初始化
				offset.THR = 0;
				offset.ROL = 0;
				offset.PIT = 0;
				offset.YAW = 0;
				return; //初始化完直接返回，进入下一次循环
			}
			else
			{
				sum_yaw += rc->YAW;
				sum_rol += rc->ROL;
				sum_pit += rc->PIT;
				sum_thr += rc->THR;
				count++;
			}
			
			if(count == 51) //因为第一次是初始化，总共累计50次
			{
				count--;//--是为了整除50次
				offset.PIT = sum_pit/count - 1500;
				offset.THR = sum_thr/count - 1000; // 油门是最低点校准，另外的是中点校准
				offset.ROL = sum_rol/count - 1500;
				offset.YAW = sum_yaw/count - 1500;
				
				 /* 获取到偏差值后，再去清零count、key_count */
				count = 0;
				key_count = 0;
			}
		}
		
	}
}

/**
 * @description: 最大值、最小值限幅，防止器件出现问题造成越界
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
 * @description: 中点限幅：认为在中间附近，就是要中值
 * @param {_Rc} *rc
 * @return {*}
 */
void App_Remote_Mid_Limit(struct _Rc *rc)
{
    /* 考虑到油门不会回弹，手动拉到完全的中间很难，范围给大一点 */
    if (rc->THR > 1450 && rc->THR < 1550)
    {
        rc->THR = 1500;
    }

    /* 其他三个摇杆会自动回中，范围可以给小一点 */
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
	//乘0.25是因为ADC读出来的原始值是0-4095，除以4范围缩到0-1000，2000-的作用是让遥感的极性翻转，不然遥感上下左右是反的，并让范围在1000-2000
	rc.THR = 2000 - (0.25f*ADC_Value[1]) - offset.THR;
	rc.YAW = 2000 - (0.25f*ADC_Value[0]) - offset.YAW;
	rc.ROL = 2000 - (0.25f*ADC_Value[2]) - offset.ROL;
	rc.PIT = 2000 - (0.25f*ADC_Value[3]) - offset.PIT;
	
	
	// 滑动窗口滤波，是数据变化平缓
	App_Remote_Window_Filter(&rc);
	// 校准
	App_Remote_Mid_Offset(&rc);
	// 最大最小限幅
	App_Remote_Limit(&rc);	
	// 中点限幅
	App_Remote_Mid_Limit(&rc);
}

void App_Remote_KeyPress(void)
{
	/* 
    整个标志位变量=
    if(按键按下&& 标志位==0)
    {
        延迟一下
        if(按键按下 && 标志位==0)
        {
            处理逻辑；
            标志=1；
        }

    }
    if(按键松开 && 标志位=1)
    {
        标志位=0；
    }
 */

    /* 1、判断哪个按键按下，对应的去微调 */
    if (!READ_KEY_U)
    {
        /* 前进按键按下，微调pitch */
        offset.PIT = offset.PIT - 10; // 前进用减（让pitch变大=减掉的偏差值变小）
    }
    /* 为什么不用else if:用户有可能多个按键同时按，都判定生效的话，单独if */
    if (!READ_KEY_D)
    {
        /* 前进按键按下，微调pitch */
        offset.PIT = offset.PIT + 10; // 后退用加（让pitch变小=减掉的偏差值变大）
    }
    if (!READ_KEY_L)
    {
        /* 左边按键按下，微调roll */
        offset.ROL = offset.ROL + 10; // 左移用加（让roll变小=减掉的偏差值变大）
    }
    if (!READ_KEY_R)
    {
        /* 左边按键按下，微调roll */
        offset.ROL = offset.ROL - 10; // 右移用减（让roll变大=减掉的偏差值变小）
    }
}

void App_Remote_Data(uint8_t *remote_data)
{
	uint8_t index = 0;
	uint32_t SUM = 0;
	
	remote_data[index++] = 0xAA;
	remote_data[index++] = 0XAF; //帧头两个字节
	
	remote_data[index++] = 0X03; //功能位
	
	remote_data[index++] = 0X00; //等待计算数据长度
	
	remote_data[index++] = (uint8_t)(rc.THR >> 8); //遥感数据16位，先传高位
	remote_data[index++] = (uint8_t)rc.THR;
	remote_data[index++] = (uint8_t)(rc.YAW >> 8); //遥感数据16位，先传高位
	remote_data[index++] = (uint8_t)rc.YAW;
	remote_data[index++] = (uint8_t)(rc.ROL >> 8); //遥感数据16位，先传高位
	remote_data[index++] = (uint8_t)rc.ROL;
	remote_data[index++] = (uint8_t)(rc.PIT >> 8); //遥感数据16位，先传高位
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
	
	
	//重新计算数据长度
	remote_data[3] = index - 4; // 4就是帧头，功能位，数据长度位，总共8个字节
	
	//校验和，计算SUM字段
	for(uint8_t i = 0; i < index; i++)
	{
		SUM += remote_data[i];
	}
	remote_data[index++] = (uint8_t)(SUM >> 24); // 不能直接将SUM填到remote_data[index++]，SUM是32位，remote_data是8位
	remote_data[index++] = (uint8_t)(SUM >> 16);
	remote_data[index++] = (uint8_t)(SUM >> 8);
	remote_data[index++] = (uint8_t)(SUM);
	
	
}




