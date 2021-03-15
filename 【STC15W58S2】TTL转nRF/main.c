/**********************************************************/
/* ��   �ƣ�NRF24L01+����                  
/* ��   �ܣ�nrf�ɷ��Ϳɽ��գ� ���յ�������ͨ�����ڷ��ͳ�ȥ�����°�����ͨ��nrf���Ͱ���ֵ��
			���������nrf����LED_LINK�����������򲻻�����
			������յ��ַ�1��LED�������յ��ַ�2��LED��
                    
���ڲ����ʣ�9600bps
����Ƶ�ʣ�11.0592MHz
/**********************************************************/
#include	"STC15.H"
#include 	"uart.h"
#include<intrins.h>
#define uchar unsigned char
#define uint  unsigned int

/**********  NRF24L01�Ĵ�����������  ***********/
#define READ_REG        0x00  //�����üĴ���,��5λΪ�Ĵ�����ַ
#define WRITE_REG       0x20  //д���üĴ���,��5λΪ�Ĵ�����ַ
#define RD_RX_PLOAD     0x61  //��RX��Ч����,1~32�ֽ�
#define WR_TX_PLOAD     0xA0  //дTX��Ч����,1~32�ֽ�
#define FLUSH_TX        0xE1  //���TX FIFO�Ĵ���.����ģʽ����
#define FLUSH_RX        0xE2  //���RX FIFO�Ĵ���.����ģʽ����
#define REUSE_TX_PL     0xE3  //����ʹ����һ������,CEΪ��,���ݰ������Ϸ���.
#define NOP             0xFF  //�ղ���,����������״̬�Ĵ���	 
/**********  NRF24L01�Ĵ�����ַ   *************/
#define CONFIG          0x00  //���üĴ�����ַ                             
#define EN_AA           0x01  //ʹ���Զ�Ӧ���� 
#define EN_RXADDR       0x02  //���յ�ַ����
#define SETUP_AW        0x03  //���õ�ַ���(��������ͨ��)
#define SETUP_RETR      0x04  //�����Զ��ط�
#define RF_CH           0x05  //RFͨ��
#define RF_SETUP        0x06  //RF�Ĵ���
#define STATUS          0x07  //״̬�Ĵ���
#define OBSERVE_TX      0x08  // ���ͼ��Ĵ���
#define CD              0x09  // �ز����Ĵ���
#define RX_ADDR_P0      0x0A  // ����ͨ��0���յ�ַ
#define RX_ADDR_P1      0x0B  // ����ͨ��1���յ�ַ
#define RX_ADDR_P2      0x0C  // ����ͨ��2���յ�ַ
#define RX_ADDR_P3      0x0D  // ����ͨ��3���յ�ַ
#define RX_ADDR_P4      0x0E  // ����ͨ��4���յ�ַ
#define RX_ADDR_P5      0x0F  // ����ͨ��5���յ�ַ
#define TX_ADDR         0x10  // ���͵�ַ�Ĵ���
#define RX_PW_P0        0x11  // ��������ͨ��0��Ч���ݿ��(1~32�ֽ�) 
#define RX_PW_P1        0x12  // ��������ͨ��1��Ч���ݿ��(1~32�ֽ�) 
#define RX_PW_P2        0x13  // ��������ͨ��2��Ч���ݿ��(1~32�ֽ�) 
#define RX_PW_P3        0x14  // ��������ͨ��3��Ч���ݿ��(1~32�ֽ�) 
#define RX_PW_P4        0x15  // ��������ͨ��4��Ч���ݿ��(1~32�ֽ�)
#define RX_PW_P5        0x16  // ��������ͨ��5��Ч���ݿ��(1~32�ֽ�)
#define FIFO_STATUS     0x17  // FIFO״̬�Ĵ���
/*����������������������������������������������������������������������������������������������������������������������������������������*/

/******   STATUS�Ĵ���bitλ����      *******/
#define MAX_TX  	0x10  	  //�ﵽ����ʹ����ж�
#define TX_OK   	0x20  	  //TX��������ж�
#define RX_OK   	0x40  	  //���յ������ж�
/*����������������������������������������������������������������������������������������������������*/

/*********     24L01���ͽ������ݿ�ȶ���	  ***********/
#define TX_ADR_WIDTH    5     //5�ֽڵ�ַ���
#define RX_ADR_WIDTH    5     //5�ֽڵ�ַ���
#define TX_PLOAD_WIDTH  32    //32�ֽ���Ч���ݿ��
#define RX_PLOAD_WIDTH  32    //32�ֽ���Ч���ݿ��

const uchar TX_ADDRESS[TX_ADR_WIDTH]={0xFF,0xFF,0xFF,0xFF,0xFF}; //���͵�ַ
const uchar RX_ADDRESS[RX_ADR_WIDTH]={0xFF,0xFF,0xFF,0xFF,0xFF}; //���յ�ַ

sbit NRF_IRQ  = P3^3;
sbit NRF_MISO = P1^4;
sbit NRF_MOSI = P1^3;
sbit NRF_SCK  = P1^5;
sbit NRF_CSN  = P1^2;
sbit NRF_CE   = P3^4;

sbit LED_LINK=P5^1;
sbit LED=P5^2;

sbit S1=P2^0;
sbit S2=P2^1;
sbit S3=P2^2;
sbit S4=P2^3;


uchar rece_buf[32];		//nrf���ջ���
uchar rec_buf[32];		//���ڽ��ջ���
uchar i=0,j;



void delay_us(uchar num)
{
	uchar i; 
	for(i=0;i<num;i++)
 	_nop_();
}
void delay_150us()
{
	uint i;
	for(i=0;i<1500;i++);
}
void delay(uint t)
{
	uchar k;
	while(t--)
	for(k=0;k<200;k++);
}
/***************************************************************/
//������ĺ�������� GPIO ģ�� SPI �Ĺ��ܡ�
//������ֽڣ�MOSI���� MSB ѭ�������
//ͬʱ�������ֽڣ�MISO���� LSB ѭ�����롣
//�����ض��룬�½��������
//���� SCK ����ʼ��Ϊ�͵�ƽ�����жϳ���
/*******************************************************************/
uchar SPI_RW(uchar byte)
{
	uchar bit_ctr;
	for(bit_ctr=0;bit_ctr<8;bit_ctr++)  // ���8λ
	{
		NRF_MOSI=(byte&0x80); 			// MSB TO MOSI
		byte=(byte<<1);					// shift next bit to MSB
		NRF_SCK=1;						//SCK��ʱ��
		byte|=NRF_MISO;	        		// capture current MISO bit
		NRF_SCK=0;
	}
	return byte;
}

/*********************************************/
/* �������ܣ���24L01�ļĴ���дֵ��һ���ֽڣ� */
/* ��ڲ�����reg   Ҫд�ļĴ�����ַ          */
/*           value ���Ĵ���д��ֵ            */
/* ���ڲ�����status ״ֵ̬                   */
/*********************************************/
uchar NRF24L01_Write_Reg(uchar reg,uchar value)
{
	uchar status;

	NRF_CSN=0;                  //CSN=0; Ƭѡ�źţ��͵�ƽ��Ч  
  	status = SPI_RW(reg);		//���ͼĴ�����ַ,����ȡ״ֵ̬
	SPI_RW(value);				//���Ĵ���д��ֵ
	NRF_CSN=1;                  //CSN=1;

	return status;
}
/*************************************************/
/* �������ܣ���24L01�ļĴ���ֵ ��һ���ֽڣ�      */
/* ��ڲ�����reg  Ҫ���ļĴ�����ַ               */
/* ���ڲ�����value �����Ĵ�����ֵ                */
/*************************************************/
uchar NRF24L01_Read_Reg(uchar reg)
{
 	uchar value;

	NRF_CSN=0;              //CSN=0;Ƭѡ�źţ��͵�ƽ��Ч   
  	SPI_RW(reg);			//���ͼĴ���ֵ(λ��),����ȡ״ֵ̬
	value = SPI_RW(NOP);
	NRF_CSN=1;             	//CSN=1;

	return value;
}
/*********************************************/
/* �������ܣ���24L01�ļĴ���ֵ������ֽڣ�   */
/* ��ڲ�����reg   �Ĵ�����ַ                */
/*           *pBuf �����Ĵ���ֵ�Ĵ������    */
/*           len   �����ֽڳ���              */
/* ���ڲ�����status ״ֵ̬                   */
/*********************************************/
uchar NRF24L01_Read_Buf(uchar reg,uchar *pBuf,uchar len)
{
	uchar status,u8_ctr;
	NRF_CSN=0;                   	//CSN=0       
  	status=SPI_RW(reg);				//���ͼĴ�����ַ,����ȡ״ֵ̬   	   
 	for(u8_ctr=0;u8_ctr<len;u8_ctr++)
	pBuf[u8_ctr]=SPI_RW(0XFF);		//��������
	NRF_CSN=1;                 		//CSN=1
  	return status;        			//���ض�����״ֵ̬
}
/**********************************************/
/* �������ܣ���24L01�ļĴ���дֵ������ֽڣ�  */
/* ��ڲ�����reg  Ҫд�ļĴ�����ַ            */
/*           *pBuf ֵ�Ĵ������               */
/*           len   �����ֽڳ���               */
/**********************************************/
uchar NRF24L01_Write_Buf(uchar reg, uchar *pBuf, uchar len)
{
	uchar status,u8_ctr;
	NRF_CSN=0;
  	status = SPI_RW(reg);			//���ͼĴ���ֵ(λ��),����ȡ״ֵ̬
  	for(u8_ctr=0; u8_ctr<len; u8_ctr++)
	SPI_RW(*pBuf++); 				//д������
	NRF_CSN=1;
  	return status;          		//���ض�����״ֵ̬
}							  					   

/*********************************************/
/* �������ܣ�24L01��������                   */
/* ��ڲ�����rxbuf ������������              */
/* ����ֵ�� 0   �ɹ��յ�����                 */
/*          1   û���յ�����                 */
/*********************************************/
uchar NRF24L01_RxPacket(uchar *rxbuf)
{
	uchar state;
	 
	state=NRF24L01_Read_Reg(STATUS);  			//��ȡ״̬�Ĵ�����ֵ    	 
	NRF24L01_Write_Reg(WRITE_REG+STATUS,state); //���TX_DS��MAX_RT�жϱ�־����state�����ص�״ֵ̬��д�뵽WRITE_REG+STATUS��ַ�ļĴ�����
	if(state&RX_OK)								//���յ�����
	{
		NRF_CE = 0;
		NRF24L01_Read_Buf(RD_RX_PLOAD,rxbuf,RX_PLOAD_WIDTH);//��ȡ����,�ڼĴ���RD_RX_PLOAD���ȡRX_PLOAD_WIDTH���ȵ����ݷŵ�rxbuf
		NRF24L01_Write_Reg(FLUSH_RX,0xff);					//���RX FIFO�Ĵ�������0xff����ֵ��д�뵽FLUSH_RX��ַ�ļĴ�����
		
		NRF_CE = 1;
		delay_150us(); 
		return 0; 
	}	   
	return 1;//û�յ��κ�����
}
/**********************************************/
/* �������ܣ�����24L01Ϊ����ģʽ              */
/* ��ڲ�����txbuf  ������������              */
/* ����ֵ�� 0x10    �ﵽ����ط�����������ʧ��*/
/*          0x20    �ɹ��������              */
/*          0xff    ����ʧ��                  */
/**********************************************/
uchar NRF24L01_TxPacket(uchar *txbuf)
{
	uchar state;
   
	NRF_CE=0;												//CE���ͣ�ʹ��24L01����
  	NRF24L01_Write_Buf(WR_TX_PLOAD,txbuf,TX_PLOAD_WIDTH);	//д���ݵ�TX BUF  32���ֽڣ�д�볤��ΪTX_PLOAD_WIDTH������txbuf���Ĵ���WR_TX_PLOAD��
 	NRF_CE=1;												//CE�øߣ�ʹ�ܷ���	   
	while(NRF_IRQ==1);										//�ȴ��������
	state=NRF24L01_Read_Reg(STATUS);  						//��ȡ״̬�Ĵ�����ֵ	   
	NRF24L01_Write_Reg(WRITE_REG+STATUS,state); 			//���TX_DS��MAX_RT�жϱ�־����state�����ص�״ֵ̬��д�뵽WRITE_REG+STATUS��ַ�ļĴ�����
	if(state&MAX_TX)										//�ﵽ����ط�����
	{
		NRF24L01_Write_Reg(FLUSH_TX,0xff);					//���TX FIFO�Ĵ�������0xff����ֵ��д�뵽FLUSH_RX��ַ�ļĴ����� 
		return MAX_TX; 
	}
	if(state&TX_OK)											//�������
	{
		return TX_OK;
	}
	return 0xff;											//����ʧ��
}

/********************************************/
/* �������ܣ����24L01�Ƿ����              */
/* ����ֵ��  0  ����                        */
/*           1  ������                      */
/********************************************/ 	  
uchar NRF24L01_Check(void)
{
	uchar check_in_buf[5]={0x11,0x22,0x33,0x44,0x55};
	uchar check_out_buf[5]={0x00};

	NRF_SCK=0;		//�����е㲻��
	NRF_CSN=1;    
	NRF_CE=0;

	NRF24L01_Write_Buf(WRITE_REG+TX_ADDR, check_in_buf, 5);

	NRF24L01_Read_Buf(READ_REG+TX_ADDR, check_out_buf, 5);

	if((check_out_buf[0] == 0x11)&&\
	   (check_out_buf[1] == 0x22)&&\
	   (check_out_buf[2] == 0x33)&&\
	   (check_out_buf[3] == 0x44)&&\
	   (check_out_buf[4] == 0x55))return 0;
	else return 1;
}			


void NRF24L01_RT_Init(void)			/*�����кܶ�Ĵ����������ܣ���NRF24L01_����.pdf��15ҳ����*/
{	
	NRF_CE=0;		  
  	NRF24L01_Write_Reg(WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);//ѡ��ͨ��0����Ч���ݿ��
	NRF24L01_Write_Reg(FLUSH_RX,0xff);					//���RX FIFO�Ĵ���    
  	NRF24L01_Write_Buf(WRITE_REG+TX_ADDR,(uchar*)TX_ADDRESS,TX_ADR_WIDTH);//дTX�ڵ��ַ 
  	NRF24L01_Write_Buf(WRITE_REG+RX_ADDR_P0,(uchar*)RX_ADDRESS,RX_ADR_WIDTH); //����TX�ڵ��ַ,��ҪΪ��ʹ��ACK	  
  	NRF24L01_Write_Reg(WRITE_REG+EN_AA,0x01);     //ʹ��ͨ��0���Զ�Ӧ��  		15ҳ
  	NRF24L01_Write_Reg(WRITE_REG+EN_RXADDR,0x01); //ʹ��ͨ��0�Ľ��յ�ַ  		15ҳ
  	NRF24L01_Write_Reg(WRITE_REG+SETUP_RETR,0x1a);//�����Զ��ط����ʱ��:500us + 86us;����Զ��ط�����:10��	   16ҳ
  	NRF24L01_Write_Reg(WRITE_REG+RF_CH,0);        //����RFͨ��Ϊ2.400GHz  Ƶ��=2.4+0GHz
  	NRF24L01_Write_Reg(WRITE_REG+RF_SETUP,0x0F);  //����TX�������,0db����,2Mbps,���������濪��   
  	NRF24L01_Write_Reg(WRITE_REG+CONFIG,0x0f);    //���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC,����ģʽ,���������ж�
	NRF_CE=1;									  //CE�øߣ�ʹ�ܷ���
}

void SEND_BUF(uchar *buf)
{
	NRF_CE=0;
	NRF24L01_Write_Reg(WRITE_REG+CONFIG,0x0e);		//0x0eΪ����ģʽ
	NRF_CE=1;

	delay_us(150);
	NRF24L01_TxPacket(buf);

	NRF_CE=0;
	NRF24L01_Write_Reg(WRITE_REG+CONFIG, 0x0f);		//0x0fΪ����ģʽ
	NRF_CE=1;	
}
void main()
{
	while(NRF24L01_Check()); // �ȴ���⵽NRF24L01������Ż�����ִ��
	NRF24L01_RT_Init();	   	 //��ʼ������	
	UartInit();
	
	LED_LINK=0;
	
	P2M0 = 0x00;		//P2ȫ������Ϊ׼˫��ڣ��ڲ���������
	P2M1 = 0x00;
	
	while(1)
	{
		if(NRF_IRQ==0)	 	// �������ģ����յ�����
		{		
			if(NRF24L01_RxPacket(rece_buf)==0)
			{	
				if(	rece_buf[1]=='1')		   	//��1λ�Ժ����յ����������ݣ�rece_buf[0]������λ������
					LED=0;
				if(	rece_buf[1]=='2')	 		//��1λ�Ժ����յ����������ݣ�rece_buf[0]������λ������
					LED=1;
				
				j = rece_buf[0];				//������������ǰ�ƶ�һλ��ȥ�����ݳ���λ���Ա㴮�ڷ���
				for(i=0;i<=j;i++)
				{
					rece_buf[i] = rece_buf[i+1];
				}
				SendString(rece_buf);			//���ڷ���nrf���յ�������
				
						
			}
		}
		
		if(S1==0)								//������һ�������ͷ�����һ��ֵ
		{
			delay_us(10);						
			if(S1==0)								
			{
				rece_buf[1]=0x53;					//S��ASCII��
				rece_buf[2]=0x31;					//1��ASCII��
	
				rece_buf[0]=2;						//һ��Ҫ����2���ֽڣ�rece_buf[0]������2������������
				SEND_BUF(rece_buf);					//nrf���ͳ�ȥ
				while(!S1);
			}	
		}
		if(S2==0)							
		{
			delay_us(10);						
			if(S2==0)								
			{
				rece_buf[1]=0x53;					//S
				rece_buf[2]=0x32;					//2

				rece_buf[0]=2;						//һ��Ҫ����2���ֽڣ�rece_buf[0]������2������������
				SEND_BUF(rece_buf);
				while(!S2);
			}	
		}
		if(S3==0)						
		{
			delay_us(10);						
			if(S3==0)								
			{
				rece_buf[1]=0x53;					//S
				rece_buf[2]=0x33;					//3

				rece_buf[0]=2;						//һ��Ҫ����2���ֽڣ�rece_buf[0]������2������������
				SEND_BUF(rece_buf);
				while(!S3);
			}	
		}
		if(S4==0)							
		{
			delay_us(10);					
			if(S4==0)								
			{
				rece_buf[1]=0x53;					//S
				rece_buf[2]=0x34;					//4
				
				rece_buf[0]=2;						//һ��Ҫ����2���ֽڣ�rece_buf[0]������2������������
				SEND_BUF(rece_buf);
				while(!S4);
			}	
		}
	}
}


