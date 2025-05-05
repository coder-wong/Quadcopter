#include "NRF24L01.h"

uint8_t TX_ADDRESS[TX_ADR_WIDTH] = {0x0A, 0x01, 0x07, 0x0E, 0x01}; // 定义一个静态发送地址
uint8_t RX_ADDRESS[RX_ADR_WIDTH] = {0x0A, 0x01, 0x07, 0x0E, 0x01}; // 定义一个静态发送地址

/* 连接状态标志位，用于后续判断是否失联 */
uint16_t connect_flag = 1;

uint8_t TX_BUFF[TX_PLOAD_WIDTH];
uint8_t RX_BUFF[RX_PLOAD_WIDTH];

/*写寄存器*/
uint8_t NRF24L01_Write_Reg(uint8_t reg, uint8_t data)
{
    uint8_t status = 0;
    //1.片选拉低（看时序图）
    NRF24L01_CSN_LOW;
    //2.发送写寄存器指令以及寄存器地址
    status = Driver_SPI_SwapByte(reg); // 0010 0000 拼上寄存器地址（001是写命令）=》001A AAAA
    //3.发送数据
    Driver_SPI_SwapByte(data);
    //4.片选拉高，结束通讯
    NRF24L01_CSN_HIGH;
    
    return status; // Driver_SPI_SwapByte是交换数据
}


/*读寄存器*/
uint8_t NRF24L01_Read_Reg(uint8_t reg)
{
    uint8_t data = 0;
    //1.片选拉低（看时序图）
    NRF24L01_CSN_LOW;
    //2.发送写寄存器指令以及寄存器地址
    Driver_SPI_SwapByte(reg); // 0000 0000 拼上寄存器地址（000是读命令）=》000A AAAA
    //3.发送数据
    data = Driver_SPI_SwapByte(0); // 写什么无所谓，写了之后才会拿到返回的数据
    //4.片选拉高，结束通讯
    NRF24L01_CSN_HIGH;
    
    return data; // Driver_SPI_SwapByte是交换数据,返回读到的数据
}

/*写多个字节*/
uint8_t NRF24L01_Write_Len(uint8_t reg, uint8_t *buff, uint8_t len)
{
    uint8_t status = 0;
    //1.片选拉低（看时序图）
    NRF24L01_CSN_LOW;
    //2.发送写寄存器指令以及寄存器地址
    status = Driver_SPI_SwapByte(reg); // 0010 0000 拼上寄存器地址（001是写命令）=》001A AAAA
    //3.循环写入数据
    for(uint8_t i = 0; i < len; i++)
    {
        Driver_SPI_SwapByte(*buff++);
    }
    //4.片选拉高，结束通讯
    NRF24L01_CSN_HIGH;
    
    return status; // Driver_SPI_SwapByte是交换数据
}

/*读多个字节*/
uint8_t NRF24L01_READ_Len(uint8_t reg, uint8_t *buff, uint8_t len)
{
    //1.片选拉低（看时序图）
    NRF24L01_CSN_LOW;
    //2.发送读寄存器指令以及寄存器地址
    Driver_SPI_SwapByte(reg); // 0010 0000 拼上寄存器地址（001是写命令）=》001A AAAA
    //3.循环读数据
    for(uint8_t i = 0; i < len; i++)
    {
        *buff++ = Driver_SPI_SwapByte(0);
    }
    //4.片选拉高，结束通讯
    NRF24L01_CSN_HIGH;
    
    return 0; // Driver_SPI_SwapByte是交换数据
}

/*发送模式*/
void NRF24L01_TX_Mode(void)
{
    //1.不再是片选，而是控制芯片工作模式的引脚，看工作模式图
    NRF24L01_CE_LOW;
    //2.相关的配置：发送地址、接收管道0地址（一样）、ACK使能、使能管道0、功率、Config为发送模式
    NRF24L01_Write_Len(SPI_WRITE_REG + TX_ADDR, TX_ADDRESS, TX_ADR_WIDTH); // 配置发送地址
    NRF24L01_Write_Len(SPI_WRITE_REG + RX_ADDR_P0, RX_ADDRESS, RX_ADR_WIDTH);// 配置接受地址
    
    
    NRF24L01_Write_Reg(SPI_WRITE_REG + EN_AA, 0x01);// 使能接收通道0自动应答
    NRF24L01_Write_Reg(SPI_WRITE_REG + EN_RXADDR, 0x01);// 使能接收通道0
    // 自动重发延时等待250us+86us，自动重发10次
    NRF24L01_Write_Reg(SPI_WRITE_REG + RF_CH, 40);// 选择射频通道0x40
    NRF24L01_Write_Reg(SPI_WRITE_REG + RF_SETUP, 0x0f); // 数据传输率2Mbps，发射功率7dBm
    NRF24L01_Write_Reg(SPI_WRITE_REG + CONFIG, 0x0e);// CRC使能，16位CRC校验，上电,最后一步上电
    NRF24L01_CE_HIGH;
}

/*接收模式*/
void NRF24L01_RX_Mode(void)
{
    NRF24L01_CE_LOW;
    /*
        与发送的区别：
            1、不需要设置发送地址
            2、需要设置接收通道0的负载长度
            3、config配置的，bit0=1 为接收模式
     */
    NRF24L01_Write_Len(SPI_WRITE_REG + RX_ADDR_P0, RX_ADDRESS, RX_ADR_WIDTH); // 接收设备接收通道0使用和发送设备相同的发送地址
    NRF24L01_Write_Reg(SPI_WRITE_REG + EN_AA, 0x01);                          // 使能接收通道0自动应答
    NRF24L01_Write_Reg(SPI_WRITE_REG + EN_RXADDR, 0x01);                      // 使能接收通道0
    NRF24L01_Write_Reg(SPI_WRITE_REG + RF_CH, 40);                            // 选择射频通道40，直接传的十进制数，代表第四十个信道，十六进制就是0x28
    NRF24L01_Write_Reg(SPI_WRITE_REG + RX_PW_P0, TX_PLOAD_WIDTH);             // 接收通道0选择和发送通道相同有效数据宽度
    NRF24L01_Write_Reg(SPI_WRITE_REG + RF_SETUP, 0x0f);                       // 数据传输率2Mbps，发射功率7dBm
    NRF24L01_Write_Reg(SPI_WRITE_REG + CONFIG, 0x0f);                         // CRC使能，16位CRC校验，上电，接收模式
    NRF24L01_CE_HIGH;                                                         // 拉高CE启动接收设备
}

/*发送函数*/
uint8_t NRF24L01_TxPacket(uint8_t *txBuf)
{
    uint8_t state = 0;
    
    NRF24L01_CE_LOW;
    NRF24L01_Write_Len(WR_TX_PLOAD, txBuf, TX_PLOAD_WIDTH); //写FIFO不需要地址，就直接写指令就行
    NRF24L01_CE_HIGH; //防止其他程序，操作了CE，导致混乱，保险操作，因为需要CE处于高才能进入发送模式
    
    /*不一定能顺利发送，可能出现重复发送的情况。
    第一次能顺利进入，然后获取状态，判断是否发送完成或达到最大发送次数，
    如果是，那么state的bit4,5就为1，在&上（TX_OK | MAX_TX），就有值，再非以下，就退出循环
    */ 
    while(!(state &(TX_OK | MAX_TX)))
    {
        state = NRF24L01_Read_Reg(SPI_READ_REG + STATUS);
    }
    
    /*清空 发送或最大重发次数 中断标志位， 如果不清那么TX_OK和MAX_TX就一直为1，那么前面通过TX_OK和MAX_TX判断是否顺利发送就不可信 */
    NRF24L01_Write_Reg(SPI_WRITE_REG + STATUS, state);
    
    
    /*当重发次数达到最大，仍没有收到确认信号时，发送端产生MAX_RT中断。
    MAX_RT中断在清除之前不能进行下一步的数据发送*/ 
    if(state & MAX_TX) // 如果达到最大重发次数，STATUS寄存器的bit4会置1
    {
        //清FIFO
        if(state & 0x01)
        {
            NRF24L01_Write_Reg(FLUSH_TX,0Xff);
        }     
    }
    
    //如果发送成果
    if(state & TX_OK)
    {
        return 0;
    }
    
    return 1;//其他原因没有发送成功
}

/*接收函数*/
uint8_t NRF24L01_RxPacket(uint8_t *rxBuf)
{
//	printf("1\r\n");
    uint8_t state = 0;
    state = NRF24L01_Read_Reg(SPI_READ_REG + STATUS);
//    printf("2\r\n");
     /* 清除所有中断标志 ,避免之前产生过的中断的影响， 通过写1清除， 如果产生中断标志位已经为1，所以直接写state*/
    NRF24L01_Write_Reg(SPI_WRITE_REG + STATUS, state);
    
    //判断是否接受到了数据
    if(state & RX_OK)
    {
//		printf("3\r\n");
        //2.读RX FIFO
        NRF24L01_READ_Len(RD_RX_PLOAD, rxBuf, RX_PLOAD_WIDTH); // 只要一读FIFO，就会产生回复ACK
//        printf("4\r\n");
        //3.清FIFO,接收端需要自己主动清FIFO不然数据会被覆盖
        NRF24L01_Write_Reg(FLUSH_RX, 0xff);
		
		/* ！失联判断逻辑：加一个标志位！！！！！！ 0表示链接成功了 */
        connect_flag = 0;
//		printf("5\r\n");
//		/* =============测试：打印接收的数据================= */
        for (uint8_t i = 0; i < RX_PLOAD_WIDTH; i++)
        {
//			printf("3\r\n");
            printf("receive[%d]=%02x\r\n", i, rxBuf[i]); // 打印要在清FIFO后，如果在前会影响清FIFO造成数据被覆盖
        }
//        /* ================================================ */
		 
        return 0;//成功接受
    }
    return 1;//未成功接受
}
/*自检函数，判断是否能正常往一个寄存器里面写读值*/
uint8_t NRF24L01_Check(void)
{
    uint8_t w_buff[5] = {0xA1, 0xA1, 0xA1, 0xA1, 0xA1};
    uint8_t r_buff[5] = {0};
    uint8_t count = 0;
    
    NRF24L01_Write_Len(SPI_WRITE_REG + TX_ADDR, w_buff, 5); // TX_ADDR是随便选的，只要不影响其他程序
    NRF24L01_READ_Len(SPI_READ_REG + TX_ADDR, r_buff, 5);
    
    for(uint8_t i = 0; i < 5; i++)
    {
        if(w_buff[i] == r_buff[i])
        {
            count++;

        }
    }
    
    if(count == 5)
    {
        return 0; // 成功
    }
    else
    {
        return 1; // 失败
    }
}

