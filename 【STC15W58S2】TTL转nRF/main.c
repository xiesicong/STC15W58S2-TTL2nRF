/**********************************************************/
/* 名   称：NRF24L01+程序                  
/* 功   能：nrf可发送可接收， 接收到的数据通过串口发送出去，按下按键，通过nrf发送按键值。
			如果接上了nrf，则LED_LINK会亮，不接则不会亮。
			如果接收到字符1，LED亮，接收到字符2，LED灭。
                    
串口波特率：9600bps
晶振频率：11.0592MHz
/**********************************************************/
#include	"STC15.H"
#include 	"uart.h"
#include<intrins.h>
#define uchar unsigned char
#define uint  unsigned int

/**********  NRF24L01寄存器操作命令  ***********/
#define READ_REG        0x00  //读配置寄存器,低5位为寄存器地址
#define WRITE_REG       0x20  //写配置寄存器,低5位为寄存器地址
#define RD_RX_PLOAD     0x61  //读RX有效数据,1~32字节
#define WR_TX_PLOAD     0xA0  //写TX有效数据,1~32字节
#define FLUSH_TX        0xE1  //清除TX FIFO寄存器.发射模式下用
#define FLUSH_RX        0xE2  //清除RX FIFO寄存器.接收模式下用
#define REUSE_TX_PL     0xE3  //重新使用上一包数据,CE为高,数据包被不断发送.
#define NOP             0xFF  //空操作,可以用来读状态寄存器	 
/**********  NRF24L01寄存器地址   *************/
#define CONFIG          0x00  //配置寄存器地址                             
#define EN_AA           0x01  //使能自动应答功能 
#define EN_RXADDR       0x02  //接收地址允许
#define SETUP_AW        0x03  //设置地址宽度(所有数据通道)
#define SETUP_RETR      0x04  //建立自动重发
#define RF_CH           0x05  //RF通道
#define RF_SETUP        0x06  //RF寄存器
#define STATUS          0x07  //状态寄存器
#define OBSERVE_TX      0x08  // 发送检测寄存器
#define CD              0x09  // 载波检测寄存器
#define RX_ADDR_P0      0x0A  // 数据通道0接收地址
#define RX_ADDR_P1      0x0B  // 数据通道1接收地址
#define RX_ADDR_P2      0x0C  // 数据通道2接收地址
#define RX_ADDR_P3      0x0D  // 数据通道3接收地址
#define RX_ADDR_P4      0x0E  // 数据通道4接收地址
#define RX_ADDR_P5      0x0F  // 数据通道5接收地址
#define TX_ADDR         0x10  // 发送地址寄存器
#define RX_PW_P0        0x11  // 接收数据通道0有效数据宽度(1~32字节) 
#define RX_PW_P1        0x12  // 接收数据通道1有效数据宽度(1~32字节) 
#define RX_PW_P2        0x13  // 接收数据通道2有效数据宽度(1~32字节) 
#define RX_PW_P3        0x14  // 接收数据通道3有效数据宽度(1~32字节) 
#define RX_PW_P4        0x15  // 接收数据通道4有效数据宽度(1~32字节)
#define RX_PW_P5        0x16  // 接收数据通道5有效数据宽度(1~32字节)
#define FIFO_STATUS     0x17  // FIFO状态寄存器
/*————————————————————————————————————————————————————————————————————*/

/******   STATUS寄存器bit位定义      *******/
#define MAX_TX  	0x10  	  //达到最大发送次数中断
#define TX_OK   	0x20  	  //TX发送完成中断
#define RX_OK   	0x40  	  //接收到数据中断
/*——————————————————————————————————————————————————*/

/*********     24L01发送接收数据宽度定义	  ***********/
#define TX_ADR_WIDTH    5     //5字节地址宽度
#define RX_ADR_WIDTH    5     //5字节地址宽度
#define TX_PLOAD_WIDTH  32    //32字节有效数据宽度
#define RX_PLOAD_WIDTH  32    //32字节有效数据宽度

const uchar TX_ADDRESS[TX_ADR_WIDTH]={0xFF,0xFF,0xFF,0xFF,0xFF}; //发送地址
const uchar RX_ADDRESS[RX_ADR_WIDTH]={0xFF,0xFF,0xFF,0xFF,0xFF}; //接收地址

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


uchar rece_buf[32];		//nrf接收缓存
uchar rec_buf[32];		//串口接收缓存
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
//最基本的函数，完成 GPIO 模拟 SPI 的功能。
//将输出字节（MOSI）从 MSB 循环输出，
//同时将输入字节（MISO）从 LSB 循环移入。
//上升沿读入，下降沿输出。
//（从 SCK 被初始化为低电平可以判断出）
/*******************************************************************/
uchar SPI_RW(uchar byte)
{
	uchar bit_ctr;
	for(bit_ctr=0;bit_ctr<8;bit_ctr++)  // 输出8位
	{
		NRF_MOSI=(byte&0x80); 			// MSB TO MOSI
		byte=(byte<<1);					// shift next bit to MSB
		NRF_SCK=1;						//SCK是时钟
		byte|=NRF_MISO;	        		// capture current MISO bit
		NRF_SCK=0;
	}
	return byte;
}

/*********************************************/
/* 函数功能：给24L01的寄存器写值（一个字节） */
/* 入口参数：reg   要写的寄存器地址          */
/*           value 给寄存器写的值            */
/* 出口参数：status 状态值                   */
/*********************************************/
uchar NRF24L01_Write_Reg(uchar reg,uchar value)
{
	uchar status;

	NRF_CSN=0;                  //CSN=0; 片选信号，低电平有效  
  	status = SPI_RW(reg);		//发送寄存器地址,并读取状态值
	SPI_RW(value);				//给寄存器写的值
	NRF_CSN=1;                  //CSN=1;

	return status;
}
/*************************************************/
/* 函数功能：读24L01的寄存器值 （一个字节）      */
/* 入口参数：reg  要读的寄存器地址               */
/* 出口参数：value 读出寄存器的值                */
/*************************************************/
uchar NRF24L01_Read_Reg(uchar reg)
{
 	uchar value;

	NRF_CSN=0;              //CSN=0;片选信号，低电平有效   
  	SPI_RW(reg);			//发送寄存器值(位置),并读取状态值
	value = SPI_RW(NOP);
	NRF_CSN=1;             	//CSN=1;

	return value;
}
/*********************************************/
/* 函数功能：读24L01的寄存器值（多个字节）   */
/* 入口参数：reg   寄存器地址                */
/*           *pBuf 读出寄存器值的存放数组    */
/*           len   数组字节长度              */
/* 出口参数：status 状态值                   */
/*********************************************/
uchar NRF24L01_Read_Buf(uchar reg,uchar *pBuf,uchar len)
{
	uchar status,u8_ctr;
	NRF_CSN=0;                   	//CSN=0       
  	status=SPI_RW(reg);				//发送寄存器地址,并读取状态值   	   
 	for(u8_ctr=0;u8_ctr<len;u8_ctr++)
	pBuf[u8_ctr]=SPI_RW(0XFF);		//读出数据
	NRF_CSN=1;                 		//CSN=1
  	return status;        			//返回读到的状态值
}
/**********************************************/
/* 函数功能：给24L01的寄存器写值（多个字节）  */
/* 入口参数：reg  要写的寄存器地址            */
/*           *pBuf 值的存放数组               */
/*           len   数组字节长度               */
/**********************************************/
uchar NRF24L01_Write_Buf(uchar reg, uchar *pBuf, uchar len)
{
	uchar status,u8_ctr;
	NRF_CSN=0;
  	status = SPI_RW(reg);			//发送寄存器值(位置),并读取状态值
  	for(u8_ctr=0; u8_ctr<len; u8_ctr++)
	SPI_RW(*pBuf++); 				//写入数据
	NRF_CSN=1;
  	return status;          		//返回读到的状态值
}							  					   

/*********************************************/
/* 函数功能：24L01接收数据                   */
/* 入口参数：rxbuf 接收数据数组              */
/* 返回值： 0   成功收到数据                 */
/*          1   没有收到数据                 */
/*********************************************/
uchar NRF24L01_RxPacket(uchar *rxbuf)
{
	uchar state;
	 
	state=NRF24L01_Read_Reg(STATUS);  			//读取状态寄存器的值    	 
	NRF24L01_Write_Reg(WRITE_REG+STATUS,state); //清除TX_DS或MAX_RT中断标志，把state（返回的状态值）写入到WRITE_REG+STATUS地址的寄存器里
	if(state&RX_OK)								//接收到数据
	{
		NRF_CE = 0;
		NRF24L01_Read_Buf(RD_RX_PLOAD,rxbuf,RX_PLOAD_WIDTH);//读取数据,在寄存器RD_RX_PLOAD里读取RX_PLOAD_WIDTH长度的数据放到rxbuf
		NRF24L01_Write_Reg(FLUSH_RX,0xff);					//清除RX FIFO寄存器，把0xff（空值）写入到FLUSH_RX地址的寄存器里
		
		NRF_CE = 1;
		delay_150us(); 
		return 0; 
	}	   
	return 1;//没收到任何数据
}
/**********************************************/
/* 函数功能：设置24L01为发送模式              */
/* 入口参数：txbuf  发送数据数组              */
/* 返回值； 0x10    达到最大重发次数，发送失败*/
/*          0x20    成功发送完成              */
/*          0xff    发送失败                  */
/**********************************************/
uchar NRF24L01_TxPacket(uchar *txbuf)
{
	uchar state;
   
	NRF_CE=0;												//CE拉低，使能24L01配置
  	NRF24L01_Write_Buf(WR_TX_PLOAD,txbuf,TX_PLOAD_WIDTH);	//写数据到TX BUF  32个字节，写入长度为TX_PLOAD_WIDTH的数据txbuf到寄存器WR_TX_PLOAD中
 	NRF_CE=1;												//CE置高，使能发送	   
	while(NRF_IRQ==1);										//等待发送完成
	state=NRF24L01_Read_Reg(STATUS);  						//读取状态寄存器的值	   
	NRF24L01_Write_Reg(WRITE_REG+STATUS,state); 			//清除TX_DS或MAX_RT中断标志，把state（返回的状态值）写入到WRITE_REG+STATUS地址的寄存器里
	if(state&MAX_TX)										//达到最大重发次数
	{
		NRF24L01_Write_Reg(FLUSH_TX,0xff);					//清除TX FIFO寄存器，把0xff（空值）写入到FLUSH_RX地址的寄存器里 
		return MAX_TX; 
	}
	if(state&TX_OK)											//发送完成
	{
		return TX_OK;
	}
	return 0xff;											//发送失败
}

/********************************************/
/* 函数功能：检测24L01是否存在              */
/* 返回值；  0  存在                        */
/*           1  不存在                      */
/********************************************/ 	  
uchar NRF24L01_Check(void)
{
	uchar check_in_buf[5]={0x11,0x22,0x33,0x44,0x55};
	uchar check_out_buf[5]={0x00};

	NRF_SCK=0;		//这里有点不懂
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


void NRF24L01_RT_Init(void)			/*这里有很多寄存器参数功能，见NRF24L01_中文.pdf第15页左右*/
{	
	NRF_CE=0;		  
  	NRF24L01_Write_Reg(WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);//选择通道0的有效数据宽度
	NRF24L01_Write_Reg(FLUSH_RX,0xff);					//清除RX FIFO寄存器    
  	NRF24L01_Write_Buf(WRITE_REG+TX_ADDR,(uchar*)TX_ADDRESS,TX_ADR_WIDTH);//写TX节点地址 
  	NRF24L01_Write_Buf(WRITE_REG+RX_ADDR_P0,(uchar*)RX_ADDRESS,RX_ADR_WIDTH); //设置TX节点地址,主要为了使能ACK	  
  	NRF24L01_Write_Reg(WRITE_REG+EN_AA,0x01);     //使能通道0的自动应答  		15页
  	NRF24L01_Write_Reg(WRITE_REG+EN_RXADDR,0x01); //使能通道0的接收地址  		15页
  	NRF24L01_Write_Reg(WRITE_REG+SETUP_RETR,0x1a);//设置自动重发间隔时间:500us + 86us;最大自动重发次数:10次	   16页
  	NRF24L01_Write_Reg(WRITE_REG+RF_CH,0);        //设置RF通道为2.400GHz  频率=2.4+0GHz
  	NRF24L01_Write_Reg(WRITE_REG+RF_SETUP,0x0F);  //设置TX发射参数,0db增益,2Mbps,低噪声增益开启   
  	NRF24L01_Write_Reg(WRITE_REG+CONFIG,0x0f);    //配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,接收模式,开启所有中断
	NRF_CE=1;									  //CE置高，使能发送
}

void SEND_BUF(uchar *buf)
{
	NRF_CE=0;
	NRF24L01_Write_Reg(WRITE_REG+CONFIG,0x0e);		//0x0e为发射模式
	NRF_CE=1;

	delay_us(150);
	NRF24L01_TxPacket(buf);

	NRF_CE=0;
	NRF24L01_Write_Reg(WRITE_REG+CONFIG, 0x0f);		//0x0f为接收模式
	NRF_CE=1;	
}
void main()
{
	while(NRF24L01_Check()); // 等待检测到NRF24L01，程序才会向下执行
	NRF24L01_RT_Init();	   	 //初始化函数	
	UartInit();
	
	LED_LINK=0;
	
	P2M0 = 0x00;		//P2全部设置为准双向口，内部有弱上拉
	P2M1 = 0x00;
	
	while(1)
	{
		if(NRF_IRQ==0)	 	// 如果无线模块接收到数据
		{		
			if(NRF24L01_RxPacket(rece_buf)==0)
			{	
				if(	rece_buf[1]=='1')		   	//第1位以后是收到的命令数据，rece_buf[0]是数据位数长度
					LED=0;
				if(	rece_buf[1]=='2')	 		//第1位以后是收到的命令数据，rece_buf[0]是数据位数长度
					LED=1;
				
				j = rece_buf[0];				//把数组数据向前移动一位，去掉数据长度位，以便串口发送
				for(i=0;i<=j;i++)
				{
					rece_buf[i] = rece_buf[i+1];
				}
				SendString(rece_buf);			//串口发送nrf接收到的数据
				
						
			}
		}
		
		if(S1==0)								//按下哪一个按键就发送哪一个值
		{
			delay_us(10);						
			if(S1==0)								
			{
				rece_buf[1]=0x53;					//S的ASCII码
				rece_buf[2]=0x31;					//1的ASCII码
	
				rece_buf[0]=2;						//一共要发送2个字节，rece_buf[0]必须是2！！！！！！
				SEND_BUF(rece_buf);					//nrf发送出去
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

				rece_buf[0]=2;						//一共要发送2个字节，rece_buf[0]必须是2！！！！！！
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

				rece_buf[0]=2;						//一共要发送2个字节，rece_buf[0]必须是2！！！！！！
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
				
				rece_buf[0]=2;						//一共要发送2个字节，rece_buf[0]必须是2！！！！！！
				SEND_BUF(rece_buf);
				while(!S4);
			}	
		}
	}
}


