#include <stdint.h>

extern void BT_Send(uint8_t *buf,uint16_t len);

/**************↓配置区↓**************/
//串口发送语句，需实现将buf指向的len个字节通过串口发出
#define DEBUG_SEND(buf,len) BT_Send((buf),(len))
//读地址限制条件，若请求的地址addr不符合条件则返回0x00，无需限制可不定义
#define DEBUG_READ_ADDR_RANGE(addr) (addr>=0x3FC80000 && addr<=0x3FCDFFFF)
//写地址限制条件，若请求的地址addr不符合条件则不会写入，无需限制可不定义
#define DEBUG_WRITE_ADDR_RANGE(addr) (addr>=0x3FC80000 && addr<=0x3FCDFFFF)
//读写偏移地址，无偏移可不定义
#define DEBUG_ADDR_OFFSET 0x3C000020
/**************↑配置区↑**************/

//接收缓存区（循环队列）大小，不建议修改
#define DEBUG_RXBUF_SIZE 30
//发送缓存区大小，不建议修改
#define DEBUG_TXBUF_SIZE 30

//数据帧格式：帧头1B+帧长1B+命令码1B+数据
//固定帧头
#define DEBUG_FRAME_HEADER 0xDB
//命令码枚举，用于标记数据帧类型
typedef enum{
	SerialCMD_ReadMem, //读内存数据
	SerialCMD_WriteMem, //写内存数据
	SerialCMD_Reset //复位
}SerialCMD;

//串口发送缓存区
uint8_t debugTxBuf[DEBUG_TXBUF_SIZE];

//以下实现了一个循环队列用于串口接收，无溢出检查
struct{
	uint8_t buf[DEBUG_RXBUF_SIZE]; //缓存区
	uint16_t startPos,endPos; //队头队尾指针
}debugRxQueue={{0},0,0};
//入队一个字符
#define DEBUG_QUEUE_PUSH(ch) { \
	debugRxQueue.buf[debugRxQueue.endPos++]=(ch); \
	if(debugRxQueue.endPos>=DEBUG_RXBUF_SIZE) \
		debugRxQueue.endPos-=DEBUG_RXBUF_SIZE; \
}
//出队一个字符
#define DEBUG_QUEUE_POP() { \
	debugRxQueue.startPos++; \
	if(debugRxQueue.startPos>=DEBUG_RXBUF_SIZE) \
		debugRxQueue.startPos-=DEBUG_RXBUF_SIZE; \
}
//获取队头字符
#define DEBUG_QUEUE_TOP() (debugRxQueue.buf[debugRxQueue.startPos])
//获取队列大小
#define DEBUG_QUEUE_SIZE() \
	(debugRxQueue.startPos<=debugRxQueue.endPos? \
	debugRxQueue.endPos-debugRxQueue.startPos: \
	debugRxQueue.endPos+DEBUG_RXBUF_SIZE-debugRxQueue.startPos)
//获取队列第pos个元素
#define DEBUG_QUEUE_AT(pos) \
	(debugRxQueue.startPos+(pos)<DEBUG_RXBUF_SIZE? \
	debugRxQueue.buf[debugRxQueue.startPos+(pos)]: \
	debugRxQueue.buf[debugRxQueue.startPos+(pos)-DEBUG_RXBUF_SIZE])

//函数声明
void Debug_SerialRecv(uint8_t *buf,uint16_t len);
void Debug_ParseBuffer(void);

//串口收到数据后传入本函数进行解析，需被外部调用
void Debug_SerialRecv(uint8_t *buf,uint16_t len)
{
	for(uint16_t i=0;i<len;i++) //将收到的数据依次入队
		DEBUG_QUEUE_PUSH(buf[i]);
	Debug_ParseBuffer(); //进入解析
}

//解析串口数据
void Debug_ParseBuffer()
{
	if(DEBUG_QUEUE_AT(0)==DEBUG_FRAME_HEADER) //第一个字节为帧头，可以继续解析
	{
		if(DEBUG_QUEUE_SIZE()>2 && DEBUG_QUEUE_SIZE()>=DEBUG_QUEUE_AT(1)) //帧长足够，可以解析
		{
			uint16_t frameLen=DEBUG_QUEUE_AT(1);//读出帧长
			uint8_t cmd=DEBUG_QUEUE_AT(2); //读出命令码
			if(cmd==SerialCMD_ReadMem) //若要读取内存数据
			{
				uint8_t byteNum=DEBUG_QUEUE_AT(3); //要读取的字节数
				if(byteNum>DEBUG_TXBUF_SIZE-3) //限制读取的字节数不能使帧长超过发送缓冲区大小
					byteNum=DEBUG_TXBUF_SIZE-3;
				uint32_t addr=0; //计算目标地址
				for(uint8_t i=0;i<4;i++)
					addr|=((uint32_t)DEBUG_QUEUE_AT(4+i))<<(i*8);
				#ifdef DEBUG_ADDR_OFFSET
					addr+=DEBUG_ADDR_OFFSET;
				#endif
				debugTxBuf[0]=DEBUG_FRAME_HEADER; //构建发送数据帧
				debugTxBuf[1]=byteNum+3;
				debugTxBuf[2]=SerialCMD_ReadMem;
				for(uint8_t i=0;i<byteNum;i++) //依次写入指定地址的数据
				{
					uint8_t byte=0;
					#ifdef DEBUG_READ_ADDR_RANGE
					if(DEBUG_READ_ADDR_RANGE((addr+i)))
					#endif
						byte=*(uint8_t*)(addr+i);
					debugTxBuf[i+3]=byte;
				}
				DEBUG_SEND(debugTxBuf,byteNum+3); //串口发送
				for(uint8_t i=0;i<frameLen;i++) //将本帧出队
					DEBUG_QUEUE_POP();
			}
			else if(cmd==SerialCMD_WriteMem) //若要写入内存数据
			{
				uint8_t byteNum=frameLen-7; //要写入的字节数
				uint32_t addr=0; //计算目标地址
				for(uint8_t i=0;i<4;i++)
					addr|=((uint32_t)DEBUG_QUEUE_AT(3+i))<<(i*8);
				#ifdef DEBUG_ADDR_OFFSET
					addr+=DEBUG_ADDR_OFFSET;
				#endif
				for(uint8_t i=0;i<byteNum;i++) //依次写入数据
				{
					#ifdef DEBUG_WRITE_ADDR_RANGE
					if(DEBUG_WRITE_ADDR_RANGE((addr+i)))
					#endif
						*(uint8_t*)(addr+i)=DEBUG_QUEUE_AT(7+i);
				}
				for(uint8_t i=0;i<frameLen;i++) //将本帧出队
					DEBUG_QUEUE_POP();
			}
			else if(cmd==SerialCMD_Reset) //若要复位
			{
				#ifdef DEBUG_RESET
				DEBUG_RESET();
				#endif
			}
			if(DEBUG_QUEUE_SIZE()>0) //若后面还有数据，进行递归解析
				Debug_ParseBuffer();
		}
	}
	else //数据帧错误
	{
		while(DEBUG_QUEUE_AT(0)!=DEBUG_FRAME_HEADER && DEBUG_QUEUE_SIZE()>0) //将错误数据出队
			DEBUG_QUEUE_POP();
		if(DEBUG_QUEUE_SIZE()>0) //若后面还有数据，继续解析
			Debug_ParseBuffer();
	}
}
