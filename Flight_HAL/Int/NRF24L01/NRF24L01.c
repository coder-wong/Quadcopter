#include "NRF24L01.h"

uint8_t TX_ADDRESS[TX_ADR_WIDTH] = {0x0A, 0x01, 0x07, 0x0E, 0x01}; // ����һ����̬���͵�ַ
uint8_t RX_ADDRESS[RX_ADR_WIDTH] = {0x0A, 0x01, 0x07, 0x0E, 0x01}; // ����һ����̬���͵�ַ

/* ����״̬��־λ�����ں����ж��Ƿ�ʧ�� */
uint16_t connect_flag = 1;

uint8_t TX_BUFF[TX_PLOAD_WIDTH];
uint8_t RX_BUFF[RX_PLOAD_WIDTH];

/*д�Ĵ���*/
uint8_t NRF24L01_Write_Reg(uint8_t reg, uint8_t data)
{
    uint8_t status = 0;
    //1.Ƭѡ���ͣ���ʱ��ͼ��
    NRF24L01_CSN_LOW;
    //2.����д�Ĵ���ָ���Լ��Ĵ�����ַ
    status = Driver_SPI_SwapByte(reg); // 0010 0000 ƴ�ϼĴ�����ַ��001��д���=��001A AAAA
    //3.��������
    Driver_SPI_SwapByte(data);
    //4.Ƭѡ���ߣ�����ͨѶ
    NRF24L01_CSN_HIGH;
    
    return status; // Driver_SPI_SwapByte�ǽ�������
}


/*���Ĵ���*/
uint8_t NRF24L01_Read_Reg(uint8_t reg)
{
    uint8_t data = 0;
    //1.Ƭѡ���ͣ���ʱ��ͼ��
    NRF24L01_CSN_LOW;
    //2.����д�Ĵ���ָ���Լ��Ĵ�����ַ
    Driver_SPI_SwapByte(reg); // 0000 0000 ƴ�ϼĴ�����ַ��000�Ƕ����=��000A AAAA
    //3.��������
    data = Driver_SPI_SwapByte(0); // дʲô����ν��д��֮��Ż��õ����ص�����
    //4.Ƭѡ���ߣ�����ͨѶ
    NRF24L01_CSN_HIGH;
    
    return data; // Driver_SPI_SwapByte�ǽ�������,���ض���������
}

/*д����ֽ�*/
uint8_t NRF24L01_Write_Len(uint8_t reg, uint8_t *buff, uint8_t len)
{
    uint8_t status = 0;
    //1.Ƭѡ���ͣ���ʱ��ͼ��
    NRF24L01_CSN_LOW;
    //2.����д�Ĵ���ָ���Լ��Ĵ�����ַ
    status = Driver_SPI_SwapByte(reg); // 0010 0000 ƴ�ϼĴ�����ַ��001��д���=��001A AAAA
    //3.ѭ��д������
    for(uint8_t i = 0; i < len; i++)
    {
        Driver_SPI_SwapByte(*buff++);
    }
    //4.Ƭѡ���ߣ�����ͨѶ
    NRF24L01_CSN_HIGH;
    
    return status; // Driver_SPI_SwapByte�ǽ�������
}

/*������ֽ�*/
uint8_t NRF24L01_READ_Len(uint8_t reg, uint8_t *buff, uint8_t len)
{
    //1.Ƭѡ���ͣ���ʱ��ͼ��
    NRF24L01_CSN_LOW;
    //2.���Ͷ��Ĵ���ָ���Լ��Ĵ�����ַ
    Driver_SPI_SwapByte(reg); // 0010 0000 ƴ�ϼĴ�����ַ��001��д���=��001A AAAA
    //3.ѭ��������
    for(uint8_t i = 0; i < len; i++)
    {
        *buff++ = Driver_SPI_SwapByte(0);
    }
    //4.Ƭѡ���ߣ�����ͨѶ
    NRF24L01_CSN_HIGH;
    
    return 0; // Driver_SPI_SwapByte�ǽ�������
}

/*����ģʽ*/
void NRF24L01_TX_Mode(void)
{
    //1.������Ƭѡ�����ǿ���оƬ����ģʽ�����ţ�������ģʽͼ
    NRF24L01_CE_LOW;
    //2.��ص����ã����͵�ַ�����չܵ�0��ַ��һ������ACKʹ�ܡ�ʹ�ܹܵ�0�����ʡ�ConfigΪ����ģʽ
    NRF24L01_Write_Len(SPI_WRITE_REG + TX_ADDR, TX_ADDRESS, TX_ADR_WIDTH); // ���÷��͵�ַ
    NRF24L01_Write_Len(SPI_WRITE_REG + RX_ADDR_P0, RX_ADDRESS, RX_ADR_WIDTH);// ���ý��ܵ�ַ
    
    
    NRF24L01_Write_Reg(SPI_WRITE_REG + EN_AA, 0x01);// ʹ�ܽ���ͨ��0�Զ�Ӧ��
    NRF24L01_Write_Reg(SPI_WRITE_REG + EN_RXADDR, 0x01);// ʹ�ܽ���ͨ��0
    // �Զ��ط���ʱ�ȴ�250us+86us���Զ��ط�10��
    NRF24L01_Write_Reg(SPI_WRITE_REG + RF_CH, 40);// ѡ����Ƶͨ��0x40
    NRF24L01_Write_Reg(SPI_WRITE_REG + RF_SETUP, 0x0f); // ���ݴ�����2Mbps�����书��7dBm
    NRF24L01_Write_Reg(SPI_WRITE_REG + CONFIG, 0x0e);// CRCʹ�ܣ�16λCRCУ�飬�ϵ�,���һ���ϵ�
    NRF24L01_CE_HIGH;
}

/*����ģʽ*/
void NRF24L01_RX_Mode(void)
{
    NRF24L01_CE_LOW;
    /*
        �뷢�͵�����
            1������Ҫ���÷��͵�ַ
            2����Ҫ���ý���ͨ��0�ĸ��س���
            3��config���õģ�bit0=1 Ϊ����ģʽ
     */
    NRF24L01_Write_Len(SPI_WRITE_REG + RX_ADDR_P0, RX_ADDRESS, RX_ADR_WIDTH); // �����豸����ͨ��0ʹ�úͷ����豸��ͬ�ķ��͵�ַ
    NRF24L01_Write_Reg(SPI_WRITE_REG + EN_AA, 0x01);                          // ʹ�ܽ���ͨ��0�Զ�Ӧ��
    NRF24L01_Write_Reg(SPI_WRITE_REG + EN_RXADDR, 0x01);                      // ʹ�ܽ���ͨ��0
    NRF24L01_Write_Reg(SPI_WRITE_REG + RF_CH, 40);                            // ѡ����Ƶͨ��40��ֱ�Ӵ���ʮ���������������ʮ���ŵ���ʮ�����ƾ���0x28
    NRF24L01_Write_Reg(SPI_WRITE_REG + RX_PW_P0, TX_PLOAD_WIDTH);             // ����ͨ��0ѡ��ͷ���ͨ����ͬ��Ч���ݿ��
    NRF24L01_Write_Reg(SPI_WRITE_REG + RF_SETUP, 0x0f);                       // ���ݴ�����2Mbps�����书��7dBm
    NRF24L01_Write_Reg(SPI_WRITE_REG + CONFIG, 0x0f);                         // CRCʹ�ܣ�16λCRCУ�飬�ϵ磬����ģʽ
    NRF24L01_CE_HIGH;                                                         // ����CE���������豸
}

/*���ͺ���*/
uint8_t NRF24L01_TxPacket(uint8_t *txBuf)
{
    uint8_t state = 0;
    
    NRF24L01_CE_LOW;
    NRF24L01_Write_Len(WR_TX_PLOAD, txBuf, TX_PLOAD_WIDTH); //дFIFO����Ҫ��ַ����ֱ��дָ�����
    NRF24L01_CE_HIGH; //��ֹ�������򣬲�����CE�����»��ң����ղ�������Ϊ��ҪCE���ڸ߲��ܽ��뷢��ģʽ
    
    /*��һ����˳�����ͣ����ܳ����ظ����͵������
    ��һ����˳�����룬Ȼ���ȡ״̬���ж��Ƿ�����ɻ�ﵽ����ʹ�����
    ����ǣ���ôstate��bit4,5��Ϊ1����&�ϣ�TX_OK | MAX_TX��������ֵ���ٷ����£����˳�ѭ��
    */ 
    while(!(state &(TX_OK | MAX_TX)))
    {
        state = NRF24L01_Read_Reg(SPI_READ_REG + STATUS);
    }
    
    /*��� ���ͻ�����ط����� �жϱ�־λ�� ���������ôTX_OK��MAX_TX��һֱΪ1����ôǰ��ͨ��TX_OK��MAX_TX�ж��Ƿ�˳�����;Ͳ����� */
    NRF24L01_Write_Reg(SPI_WRITE_REG + STATUS, state);
    
    
    /*���ط������ﵽ�����û���յ�ȷ���ź�ʱ�����Ͷ˲���MAX_RT�жϡ�
    MAX_RT�ж������֮ǰ���ܽ�����һ�������ݷ���*/ 
    if(state & MAX_TX) // ����ﵽ����ط�������STATUS�Ĵ�����bit4����1
    {
        //��FIFO
        if(state & 0x01)
        {
            NRF24L01_Write_Reg(FLUSH_TX,0Xff);
        }     
    }
    
    //������ͳɹ�
    if(state & TX_OK)
    {
        return 0;
    }
    
    return 1;//����ԭ��û�з��ͳɹ�
}

/*���պ���*/
uint8_t NRF24L01_RxPacket(uint8_t *rxBuf)
{
//	printf("1\r\n");
    uint8_t state = 0;
    state = NRF24L01_Read_Reg(SPI_READ_REG + STATUS);
//    printf("2\r\n");
     /* ��������жϱ�־ ,����֮ǰ���������жϵ�Ӱ�죬 ͨ��д1����� ��������жϱ�־λ�Ѿ�Ϊ1������ֱ��дstate*/
    NRF24L01_Write_Reg(SPI_WRITE_REG + STATUS, state);
    
    //�ж��Ƿ���ܵ�������
    if(state & RX_OK)
    {
//		printf("3\r\n");
        //2.��RX FIFO
        NRF24L01_READ_Len(RD_RX_PLOAD, rxBuf, RX_PLOAD_WIDTH); // ֻҪһ��FIFO���ͻ�����ظ�ACK
//        printf("4\r\n");
        //3.��FIFO,���ն���Ҫ�Լ�������FIFO��Ȼ���ݻᱻ����
        NRF24L01_Write_Reg(FLUSH_RX, 0xff);
		
		/* ��ʧ���ж��߼�����һ����־λ������������ 0��ʾ���ӳɹ��� */
        connect_flag = 0;
//		printf("5\r\n");
//		/* =============���ԣ���ӡ���յ�����================= */
        for (uint8_t i = 0; i < RX_PLOAD_WIDTH; i++)
        {
//			printf("3\r\n");
            printf("receive[%d]=%02x\r\n", i, rxBuf[i]); // ��ӡҪ����FIFO�������ǰ��Ӱ����FIFO������ݱ�����
        }
//        /* ================================================ */
		 
        return 0;//�ɹ�����
    }
    return 1;//δ�ɹ�����
}
/*�Լ캯�����ж��Ƿ���������һ���Ĵ�������д��ֵ*/
uint8_t NRF24L01_Check(void)
{
    uint8_t w_buff[5] = {0xA1, 0xA1, 0xA1, 0xA1, 0xA1};
    uint8_t r_buff[5] = {0};
    uint8_t count = 0;
    
    NRF24L01_Write_Len(SPI_WRITE_REG + TX_ADDR, w_buff, 5); // TX_ADDR�����ѡ�ģ�ֻҪ��Ӱ����������
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
        return 0; // �ɹ�
    }
    else
    {
        return 1; // ʧ��
    }
}

