# zModbus

![](https://img.shields.io/badge/release-v1.0-blue)

![](https://media0.giphy.com/media/pt0EKLDJmVvlS/giphy.gif?cid=ecf05e474joy90ovjejl3l2da4by199wwy5hhli1cikygw74&rid=giphy.gif&ct=g)

## 待完成事项

- [x] **基于Freemodbus使用uart实现通信**
- [x] **基于Freemodbus使用rs-485实现通信**
- [x] **根据实际需求，改写freemodbus代码**
- [x] **编写总结文档**



## 参考文档

[![](https://img.shields.io/badge/全能骑士涛锅锅-四种寄存器的数据读写操作-red)](http://search.maven.org/#artifactdetails%7Cio.github.yidasanqian.dynamicadddate%7Cdynamic-add-date%7C1.1.0%7Cjar)



## 总结

### :gem: modbus知识点

> 参考文档-Modbus协议中文版【完整版】

* **PDU与ADU的划分**

  ADU（全称应用数据单元）是指一个完整modbus帧，包含了地址域-功能码-数据-校验码。去除了地址域与校验码，只包含功能码和数据就是PDU（全称协议数据单元）

  ![image-20210627142746706](https://gitee.com/LucasXm/img/raw/master/img//image-20210627142746706.png)

* **modbus数据模型介绍**

  modbus包含离散输入量、线圈、输入寄存器、保持寄存器四个数据模型，其介绍及操作方式如下表。

  | 数据模型   | 对象类型 | 访问方式 | 简称  | 起始地址 | 结束地 | 功能码                                                       |
  | ---------- | -------- | -------- | ----- | -------- | ------ | ------------------------------------------------------------ |
  | 线圈寄存器 | 1个bit   | 读写     | 0xxxx | 00000    | 09999  | 0x01 读一组逻辑线圈**(参考文档P9)**<br>0x05 写单个线圈**(参考文档P17)**<br>0x0F 写多个线圈**(参考文档P21)** |
  | 离散寄存器 | 1个bit   | 只读     | 1xxxx | 10000    | 19999  | 0x02 读一组开关输入**(参考文档P11)**                         |
  | 输入寄存器 | 16个bit  | 只读     | 3xxxx | 30000    | 39999  | 0x04 读一个或多个输入寄存器**(参考文档P15)**                 |
  | 保持寄存器 | 16个bit  | 读写     | 4xxxx | 40000    | 49999  | 0x03 读一个或多个保持寄存器值**(参考文档P13)**<br>0x06 写单个保持寄存器**(参考文档P19)**<br>0x10 写多个保持寄存器**(参考文档P23)**<br>0x17 读/写多个寄存器**(参考文档P32)** |

### :pencil: freemodbus移植步骤

#### 需要准备的文件

* [freemodbus源码](https://www.embedded-solutions.at/zh-hans/freemodbus/)
* 一个串口工程

#### 移植步骤

* 在串口工程中创建一个freemodbus文件夹，在文件夹内加入一下内容

  * freemodbus-v1.6下的modbus文件夹
  * freemodbus-v1.6\demo\BARE\port下的port文件夹

  如下图所示

  ![image-20210627154643264](https://gitee.com/LucasXm/img/raw/master/img//image-20210627154643264.png)

* 在keil中添加.c文件，并且添加路径，如下图

  ![image-20210627155124418](https://gitee.com/LucasXm/img/raw/master/img//image-20210627155124418.png)![image-20210627155242402](https://gitee.com/LucasXm/img/raw/master/img//image-20210627155242402.png)

* 完善portserial.c文件

  看名字就知道是和串口相关的，需要完成串口使能，串口初始化，发生一个字节，接受一个字节、串口中断等功能等功能，完善后的代码如下

  ``` c
  #include "port.h"
  
  /* ----------------------- Modbus includes ----------------------------------*/
  #include "mb.h"
  #include "mbport.h"
  
  #include "usart.h"
  /* ----------------------- static functions ---------------------------------*/
  static void prvvUARTTxReadyISR( void );
  static void prvvUARTRxISR( void );
  
  /* ----------------------- Start implementation -----------------------------*/
  void
  vMBPortSerialEnable( BOOL xRxEnable, BOOL xTxEnable )
  {
      /* If xRXEnable enable serial receive interrupts. If xTxENable enable
       * transmitter empty interrupts.
       */
  		if(xRxEnable == TRUE)
  		{
  			USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
  			GPIO_WriteBit(GPIOG,GPIO_Pin_8,Bit_RESET);
  		}
  		else
  		{
  			USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);
  			GPIO_WriteBit(GPIOG,GPIO_Pin_8,Bit_SET);
  		}
  		
  		if(xTxEnable == TRUE)
  		{
  			USART_ITConfig(USART2, USART_IT_TC, ENABLE);
  		}
  		else
  		{
  			USART_ITConfig(USART2, USART_IT_TC, DISABLE);
  		}
  }
  
  BOOL
  xMBPortSerialInit( UCHAR ucPORT, ULONG ulBaudRate, UCHAR ucDataBits, eMBParity eParity )
  {
      uart_init(ulBaudRate);  
  		USART_NVIC();
  		return TRUE;
  }
  
  BOOL
  xMBPortSerialPutByte( CHAR ucByte )
  {
      /* Put a byte in the UARTs transmit buffer. This function is called
       * by the protocol stack if pxMBFrameCBTransmitterEmpty( ) has been
       * called. */
  		USART_SendData(USART2, ucByte);
      return TRUE;
  }
  
  BOOL
  xMBPortSerialGetByte( CHAR * pucByte )
  {
      /* Return the byte in the UARTs receive buffer. This function is called
       * by the protocol stack after pxMBFrameCBByteReceived( ) has been called.
       */
  		*pucByte = USART_ReceiveData(USART2); 
      return TRUE;
  }
  
  /* Create an interrupt handler for the transmit buffer empty interrupt
   * (or an equivalent) for your target processor. This function should then
   * call pxMBFrameCBTransmitterEmpty( ) which tells the protocol stack that
   * a new character can be sent. The protocol stack will then call 
   * xMBPortSerialPutByte( ) to send the character.
   */
  static void prvvUARTTxReadyISR( void )
  {
      pxMBFrameCBTransmitterEmpty(  );
  }
  
  /* Create an interrupt handler for the receive interrupt for your target
   * processor. This function should then call pxMBFrameCBByteReceived( ). The
   * protocol stack will then call xMBPortSerialGetByte( ) to retrieve the
   * character.
   */
  static void prvvUARTRxISR( void )
  {
      pxMBFrameCBByteReceived(  );
  }
  
  
  
  
  /**
    * @brief  This function handles USART2 Handler.
    * @param  None
    * @retval None
    */
  void USART2_IRQHandler(void)
  {
    //发生接收中断
    if(USART_GetITStatus(USART2, USART_IT_RXNE) == SET)
    {
      prvvUARTRxISR(); 
      //清除中断标志位    
      USART_ClearITPendingBit(USART2, USART_IT_RXNE);   
    }
  	
  	if(USART_GetITStatus(USART2, USART_IT_ORE) == SET)
    {  
      USART_ClearITPendingBit(USART2, USART_IT_ORE);
  		prvvUARTRxISR(); 	
    }
    
    //发生完成中断
    if(USART_GetITStatus(USART2, USART_IT_TC) == SET)
    {
      prvvUARTTxReadyISR();
      //清除中断标志
      USART_ClearITPendingBit(USART2, USART_IT_TC);
    }
  }
  
  ```
  
  其中uart_init(ulBaudRate)函数 和 USART_NVIC()函数如下所示
  
  ``` c
  //初始化IO 串口2 
  //bound:波特率
  void uart_init(u32 bound)
  {
     //GPIO端口设置
    GPIO_InitTypeDef GPIO_InitStructure;
  	USART_InitTypeDef USART_InitStructure;
  
  	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //使能GPIOA时钟
  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);//使能USART2时钟
  	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG,ENABLE);
  	
  	//rs485 控制引脚 高发 低收
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8; //GPIOG8
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//输出
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
  	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽输出
  	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
  	GPIO_Init(GPIOG,&GPIO_InitStructure); //初始化PG8
  	GPIO_WriteBit(GPIOG,GPIO_Pin_8,Bit_RESET);
  	
  	//串口2对应引脚复用映射
  	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2); //GPIOA9复用为USART1
  	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2); //GPIOA10复用为USART1
  	
  	//USART1端口配置
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3; //GPIOA9与GPIOA10
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
  	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
  	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
  	GPIO_Init(GPIOA,&GPIO_InitStructure); //初始化PA9，PA10
  
     //USART1 初始化设置
  	USART_InitStructure.USART_BaudRate = bound;//波特率设置
  	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
  	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
  	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
  	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
  	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
    USART_Init(USART2, &USART_InitStructure); //初始化串口2
  	
    USART_Cmd(USART2, ENABLE);  //使能串口2 
  }
  
  
  /**
    * @brief  USART1 中断 配置
    * @param  无
    * @retval 无
    */
  void USART_NVIC(void)
  {
    NVIC_InitTypeDef NVIC_InitStructure;
    
    /* Configure one bit for preemption priority */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
    
    /* 配置中断源 */
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
  }
  ```
  
* 完善porttimer.c文件

  modbus工作时需要一个定时器，所以这里配置一个定时器，定时器时基为50us，具体如下

  ```c
  /* ----------------------- Platform includes --------------------------------*/
  #include "port.h"
  
  /* ----------------------- Modbus includes ----------------------------------*/
  #include "mb.h"
  #include "mbport.h"
  
  #include "timer.h"
  
  /* ----------------------- static functions ---------------------------------*/
  static void prvvTIMERExpiredISR( void );
  
  /* ----------------------- Start implementation -----------------------------*/
  BOOL
  xMBPortTimersInit( USHORT usTim1Timerout50us )
  {
      timer2_init(usTim1Timerout50us);
  		timer2_nvic();
  		return TRUE;
  }
  
  
  void
  vMBPortTimersEnable(  )
  {
      /* Enable the timer with the timeout passed to xMBPortTimersInit( ) */
  		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
  		TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
  		TIM_SetCounter(TIM2,0x0000); 
  		TIM_Cmd(TIM2, ENABLE);
  }
  
  void
  vMBPortTimersDisable(  )
  {
      /* Disable any pending timers. */
  		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
  		TIM_ITConfig(TIM2, TIM_IT_Update, DISABLE);
  		TIM_SetCounter(TIM2,0x0000); 
  		TIM_Cmd(TIM2, DISABLE);
  }
  
  /* Create an ISR which is called whenever the timer has expired. This function
   * must then call pxMBPortCBTimerExpired( ) to notify the protocol stack that
   * the timer has expired.
   */
  static void prvvTIMERExpiredISR( void )
  {
      ( void )pxMBPortCBTimerExpired(  );
  }
  
  
  void TIM2_IRQHandler(void)
  {
  	if(TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
  	{
  		prvvTIMERExpiredISR();
  		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
  	}
  }
  
  ```

  其中 timer2_init(usTim1Timerout50us) 和 timer2_nvic() 是timer2初始化函数，内容如下：

  ``` c
  void timer2_init(uint16_t period)
  {
  	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    TIM_DeInit(TIM2);
  	TIM_TimeBaseStructure.TIM_Period = period;
    TIM_TimeBaseStructure.TIM_Prescaler = (4200 - 1);	
  	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
  	TIM_Cmd(TIM2, ENABLE);
  }
  
  void timer2_nvic(void)
  {
  	NVIC_InitTypeDef NVIC_InitStructure;
  	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	
  	NVIC_Init(&NVIC_InitStructure);	
  }
  
  ```

* 移植线圈寄存器、离散寄存器、输入寄存器、保持寄存器的读写操作函数

  ``` c
  #define REG_INPUT_START 0x0010 //设置起始地址（PLC地址，base 1）
  #define REG_INPUT_NREGS 50   //设置寄存器数量
  static USHORT usRegInputBuf[REG_INPUT_NREGS];
  
  
  #define REG_HOLDING_START 0x0010 //设置起始地址（PLC地址，base 1）
  #define REG_HOLDING_NREGS 200  //设置寄存器数量
  static USHORT usRegHoldingBuf[REG_HOLDING_NREGS];
  
  
  #define REG_DISCRETE_START		0x0010 //设置起始地址（PLC地址，base 1）
  #define REG_DISCRETE_NREGS		20   //设置寄存器数量
  #define REG_DISCRETE_SIZE		REG_DISCRETE_NREGS * 8 //由于线圈/离散寄存器表示的是开关量，因此1个字节可以保存8个开关量
  static UCHAR ucRegDiscreteBuf[REG_DISCRETE_NREGS];
  
  
  #define REG_COILS_START 0x0010 //设置起始地址（PLC地址，base 1）
  #define REG_COILS_NREGS 20 //设置寄存器数量
  #define REG_COILS_SIZE REG_COILS_NREGS * 8 //由于线圈/离散寄存器表示的是开关量，因此1个字节可以保存8个开关量
  static UCHAR ucRegCoilsBuf[REG_COILS_NREGS];
  
  /**
   * @brief 读输入寄存器的服务函数
   * 		在mbfuncinput.c的eMBFuncReadInputRegister函数中被调用。
   * 		其功能可以简单理解为将业务程序中的usRegInputBuf数组的值copy到pucRegBuffer数组中。
   * @param pucRegBuffer: 保存输入寄存器值的缓存
   * @param usAddress: 输入寄存器的起始地址
   * @param usNRegs: 输入寄存器的数量
   * @return modbus执行的错误状态码
   */
  eMBErrorCode eMBRegInputCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs )
  {
      eMBErrorCode    eStatus = MB_ENOERR;
      int             iRegIndex;
  
      if( ( usAddress >= REG_INPUT_START )
          && ( usAddress + usNRegs <= REG_INPUT_START + REG_INPUT_NREGS ) )
      {
          iRegIndex = ( int )( usAddress - REG_INPUT_START );
          while( usNRegs > 0 )
          {
              *pucRegBuffer++ = ( unsigned char )( usRegInputBuf[iRegIndex] >> 8 );
              *pucRegBuffer++ = ( unsigned char )( usRegInputBuf[iRegIndex] & 0xFF );
              iRegIndex++;
              usNRegs--;
          }
      }
      else
      {
          eStatus = MB_ENOREG;
      }
  
      return eStatus;
  }
  
  
  
  
  /**
   * @brief 读保持寄存器的服务函数
   * 		在mbfuncholding.c的eMBFuncReadHoldingRegister, eMBFuncReadWriteMultipleHoldingRegister,
   * 		eMBFuncWriteHoldingRegister, eMBFuncWriteMultipleHoldingRegister四个函数中被调用。
   * 		其功能可以简单理解为将业务程序中的usRegInputBuf数组的值copy到pucRegBuffer数组中。
   * @param pucRegBuffer: 保存保持寄存器值的缓存
   * @param usAddress: 保存寄存器的起始地址
   * @param usNRegs: 保持寄存器的数量
   * @param eMode: 操作保持寄存器的模式（读 or 写）
   * @return modbus执行的错误状态码
   */
  eMBErrorCode eMBRegHoldingCB(UCHAR * pucRegBuffer, uint16_t usAddress, uint16_t usNRegs, eMBRegisterMode eMode)
  {
  	//错误状态
  	eMBErrorCode eStatus = MB_ENOERR;
  	//偏移量
  	int16_t iRegIndex;
  
  	//判断寄存器是不是在范围内
  	if (((int16_t) usAddress >= REG_HOLDING_START) && (usAddress + usNRegs <= REG_HOLDING_START + REG_HOLDING_NREGS))
  	{
  		//计算偏移量
  		iRegIndex = (int16_t) (usAddress - REG_HOLDING_START);
  
  		switch (eMode)
  		{
  		//读处理函数
  		case MB_REG_READ:
  			while (usNRegs > 0)
  			{
  				*pucRegBuffer++ = (uint8_t) (usRegHoldingBuf[iRegIndex] >> 8);
  				*pucRegBuffer++ = (uint8_t) (usRegHoldingBuf[iRegIndex] & 0xFF);
  				iRegIndex++;
  				usNRegs--;
  			}
  			break;
  
  			//写处理函数
  		case MB_REG_WRITE:
  			while (usNRegs > 0)
  			{
  				usRegHoldingBuf[iRegIndex] = *pucRegBuffer++ << 8;
  				usRegHoldingBuf[iRegIndex] |= *pucRegBuffer++;
  				/**
  				 * Sync holding register data to storable data.
  				 * Save holding register data to eeprom.
  				 */
  				iRegIndex++;
  				usNRegs--;
  			}
  			break;
  		default:
  			eStatus = MB_EINVAL;
  			break;
  		}
  	}
  	else
  	{
  		//返回错误状态
  		eStatus = MB_ENOREG;
  	}
  
  	return eStatus;
  }
  
  /**
   * @brief 读写线圈寄存器的服务函数
   * @param pucRegBuffer: 操作线圈寄存器值的缓存
   * @param usAddress: 线圈寄存器的首地址
   * @param usNCoils: 线圈寄存器的数量（字节数组数量 x 8）
   * @param eMode: 操作线圈寄存器的模式（读 or 写）
   * @return modbus执行的错误状态码
   */
  eMBErrorCode eMBRegCoilsCB(UCHAR * pucRegBuffer, uint16_t usAddress, uint16_t usNCoils, eMBRegisterMode eMode)
  {
  	//错误状态
  	eMBErrorCode eStatus = MB_ENOERR;
  	//寄存器个数
  	int16_t iNCoils = (int16_t) usNCoils;
  	//寄存器偏移量
  	int16_t usBitOffset;
  
  	//检查寄存器是否在指定范围内
  	if (((int16_t) usAddress >= REG_COILS_START) && (usAddress + usNCoils <= REG_COILS_START + REG_COILS_SIZE))
  	{
  		//计算寄存器偏移量
  		usBitOffset = (int16_t) (usAddress - REG_COILS_START);
  		switch (eMode)
  		{
  		//读操作
  		case MB_REG_READ:
  			while (iNCoils > 0)
  			{
  				*pucRegBuffer++ = xMBUtilGetBits(ucRegCoilsBuf, usBitOffset, (uint8_t) (iNCoils > 8 ? 8 : iNCoils));
  				iNCoils -= 8;
  				usBitOffset += 8;
  			}
  			break;
  
  			//写操作
  		case MB_REG_WRITE:
  			while (iNCoils > 0)
  			{
  				xMBUtilSetBits(ucRegCoilsBuf, usBitOffset, (uint8_t) (iNCoils > 8 ? 8 : iNCoils), *pucRegBuffer++);
  				iNCoils -= 8;
  			}
  			break;
  		default:
  			eStatus = MB_EINVAL;
  			break;
  		}
  	}
  	else
  	{
  		eStatus = MB_ENOREG;
  	}
  	return eStatus;
  }
  
  /**
   * @brief 读离散寄存器的服务函数
   * @param pucRegBuffer: 操作离散寄存器值的缓存
   * @param usAddress: 离散寄存器的首地址
   * @param usNDiscrete: 离散寄存器的数量（字节数组数量 x 8）
   * @return modbus执行的错误状态码
   */
  eMBErrorCode eMBRegDiscreteCB(UCHAR * pucRegBuffer, uint16_t usAddress, uint16_t usNDiscrete)
  {
  	//错误状态
  	eMBErrorCode eStatus = MB_ENOERR;
  	//寄存器个数
  	int16_t iNDiscrete = (int16_t) usNDiscrete;
  	//偏移量
  	uint16_t usBitOffset;  
  
  	//判断寄存器时候再制定范围内
  	if (((int16_t) usAddress >= REG_DISCRETE_START)
  			&& (usAddress + usNDiscrete <= REG_DISCRETE_START + REG_DISCRETE_SIZE))
  	{
  		//获得偏移量
  		usBitOffset = (uint16_t) (usAddress - REG_DISCRETE_START);
  
  		while (iNDiscrete > 0)
  		{
  			*pucRegBuffer++ = xMBUtilGetBits(ucRegDiscreteBuf, usBitOffset, (uint8_t) (iNDiscrete > 8 ? 8 : iNDiscrete));
  			iNDiscrete -= 8;
  			usBitOffset += 8;
  		}
  	}
  	else
  	{
  		eStatus = MB_ENOREG;
  	}
  	return eStatus;
  }
  ```

* 完成上述操作后，我们需要在main函数中去调用freemodbus接口，具体如下

  ``` c
  int main(void)
  { 
  	usRegInputBuf[0] = 0xaabb;
  	usRegInputBuf[1] = 0xccdd;
  	usRegInputBuf[2] = 0xeeff;
  	usRegInputBuf[3] = 0x1122;
  	usRegInputBuf[4] = 0x3344;
  	usRegInputBuf[5] = 0x5566;
  	
  	ucRegDiscreteBuf[0] = 0x55;
  	ucRegDiscreteBuf[1] = 0x0f;
  	ucRegDiscreteBuf[2] = 0x00;
  	ucRegDiscreteBuf[3] = 0x00;
  	
  	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);//设置系统中断优先级分组2
  	delay_init(168);		//延时初始化 
  
  	eMBInit(MB_RTU, 0x01, 0x01, 9600, MB_PAR_NONE);
  	eMBEnable();  
  
  	while(1)
  	{
  		(void)eMBPoll();
  	}
  }
  ```

#### 其他优化型操作

至此，freemodbus的移植操作就全部完成了，还有三个需要优化的点，需要注意。

* 由于freemodbus中使用了断言操作，所以我们需要对其进行函数定义，具体代码如下

  ``` c
  #ifdef USE_FULL_ASSERT
  void assert_failed(uint8_t *file,uint32_t line)
  {
  	while(1)
  }
  
  #else
  void __aeabi_assert(const char *x1,const char *x2,int x3)
  {
  	
  }
  #endif
  ```

* 测试发现，freemodbus移植完成后，只会返回一次数据。修改mbrtu.c文件的**eMBRTUSend**函数解决问题，具体代码如下

  ``` c
  eMBErrorCode
  eMBRTUSend( UCHAR ucSlaveAddress, const UCHAR * pucFrame, USHORT usLength )
  {
      eMBErrorCode    eStatus = MB_ENOERR;
      USHORT          usCRC16;
  
      ENTER_CRITICAL_SECTION(  );
  
      /* Check if the receiver is still in idle state. If not we where to
       * slow with processing the received frame and the master sent another
       * frame on the network. We have to abort sending the frame.
       */
      if( eRcvState == STATE_RX_IDLE )
      {
          /* First byte before the Modbus-PDU is the slave address. */
          pucSndBufferCur = ( UCHAR * ) pucFrame - 1;
          usSndBufferCount = 1;
  
          /* Now copy the Modbus-PDU into the Modbus-Serial-Line-PDU. */
          pucSndBufferCur[MB_SER_PDU_ADDR_OFF] = ucSlaveAddress;
          usSndBufferCount += usLength;
  
          /* Calculate CRC16 checksum for Modbus-Serial-Line-PDU. */
          usCRC16 = usMBCRC16( ( UCHAR * ) pucSndBufferCur, usSndBufferCount );
          ucRTUBuf[usSndBufferCount++] = ( UCHAR )( usCRC16 & 0xFF );
          ucRTUBuf[usSndBufferCount++] = ( UCHAR )( usCRC16 >> 8 );
  
          /* Activate the transmitter. */
          eSndState = STATE_TX_XMIT;
  				
  		//启动第一次发送，这样才可以进入发送完成中断
          xMBPortSerialPutByte( ( CHAR )*pucSndBufferCur );
          pucSndBufferCur++;  /* next byte in sendbuffer. */
          usSndBufferCount--;
  			
          vMBPortSerialEnable( FALSE, TRUE );
      }
      else
      {
          eStatus = MB_EIO;
      }
      EXIT_CRITICAL_SECTION(  );
      return eStatus;
  }
  ```

* 测试发现，线圈寄存器，离散寄存器，输入寄存器，保持寄存器的读写操作均不能操作到数据第一个字节（如定义数组buf[5],buf[0]永远不会被操作），注释掉mbfunccoils.c、mbfuncdisc.c、mbfuncholding.c、mbfuncinput.c文件的中`usRegAddress++;`即可解决，具体代码如下

  mbfunccoils.c

``` c
#include "stdlib.h"
#include "string.h"

/* ----------------------- Platform includes --------------------------------*/
#include "port.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbframe.h"
#include "mbproto.h"
#include "mbconfig.h"

/* ----------------------- Defines ------------------------------------------*/
#define MB_PDU_FUNC_READ_ADDR_OFF           ( MB_PDU_DATA_OFF )
#define MB_PDU_FUNC_READ_COILCNT_OFF        ( MB_PDU_DATA_OFF + 2 )
#define MB_PDU_FUNC_READ_SIZE               ( 4 )
#define MB_PDU_FUNC_READ_COILCNT_MAX        ( 0x07D0 )

#define MB_PDU_FUNC_WRITE_ADDR_OFF          ( MB_PDU_DATA_OFF )
#define MB_PDU_FUNC_WRITE_VALUE_OFF         ( MB_PDU_DATA_OFF + 2 )
#define MB_PDU_FUNC_WRITE_SIZE              ( 4 )

#define MB_PDU_FUNC_WRITE_MUL_ADDR_OFF      ( MB_PDU_DATA_OFF )
#define MB_PDU_FUNC_WRITE_MUL_COILCNT_OFF   ( MB_PDU_DATA_OFF + 2 )
#define MB_PDU_FUNC_WRITE_MUL_BYTECNT_OFF   ( MB_PDU_DATA_OFF + 4 )
#define MB_PDU_FUNC_WRITE_MUL_VALUES_OFF    ( MB_PDU_DATA_OFF + 5 )
#define MB_PDU_FUNC_WRITE_MUL_SIZE_MIN      ( 5 )
#define MB_PDU_FUNC_WRITE_MUL_COILCNT_MAX   ( 0x07B0 )

/* ----------------------- Static functions ---------------------------------*/
eMBException    prveMBError2Exception( eMBErrorCode eErrorCode );

/* ----------------------- Start implementation -----------------------------*/

#if MB_FUNC_READ_COILS_ENABLED > 0

eMBException
eMBFuncReadCoils( UCHAR * pucFrame, USHORT * usLen )
{
    USHORT          usRegAddress;
    USHORT          usCoilCount;
    UCHAR           ucNBytes;
    UCHAR          *pucFrameCur;

    eMBException    eStatus = MB_EX_NONE;
    eMBErrorCode    eRegStatus;

    if( *usLen == ( MB_PDU_FUNC_READ_SIZE + MB_PDU_SIZE_MIN ) )
    {
        usRegAddress = ( USHORT )( pucFrame[MB_PDU_FUNC_READ_ADDR_OFF] << 8 );
        usRegAddress |= ( USHORT )( pucFrame[MB_PDU_FUNC_READ_ADDR_OFF + 1] );
//        usRegAddress++;

        usCoilCount = ( USHORT )( pucFrame[MB_PDU_FUNC_READ_COILCNT_OFF] << 8 );
        usCoilCount |= ( USHORT )( pucFrame[MB_PDU_FUNC_READ_COILCNT_OFF + 1] );

        /* Check if the number of registers to read is valid. If not
         * return Modbus illegal data value exception. 
         */
        if( ( usCoilCount >= 1 ) &&
            ( usCoilCount < MB_PDU_FUNC_READ_COILCNT_MAX ) )
        {
            /* Set the current PDU data pointer to the beginning. */
            pucFrameCur = &pucFrame[MB_PDU_FUNC_OFF];
            *usLen = MB_PDU_FUNC_OFF;

            /* First byte contains the function code. */
            *pucFrameCur++ = MB_FUNC_READ_COILS;
            *usLen += 1;

            /* Test if the quantity of coils is a multiple of 8. If not last
             * byte is only partially field with unused coils set to zero. */
            if( ( usCoilCount & 0x0007 ) != 0 )
            {
                ucNBytes = ( UCHAR )( usCoilCount / 8 + 1 );
            }
            else
            {
                ucNBytes = ( UCHAR )( usCoilCount / 8 );
            }
            *pucFrameCur++ = ucNBytes;
            *usLen += 1;

            eRegStatus =
                eMBRegCoilsCB( pucFrameCur, usRegAddress, usCoilCount,
                               MB_REG_READ );

            /* If an error occured convert it into a Modbus exception. */
            if( eRegStatus != MB_ENOERR )
            {
                eStatus = prveMBError2Exception( eRegStatus );
            }
            else
            {
                /* The response contains the function code, the starting address
                 * and the quantity of registers. We reuse the old values in the 
                 * buffer because they are still valid. */
                *usLen += ucNBytes;;
            }
        }
        else
        {
            eStatus = MB_EX_ILLEGAL_DATA_VALUE;
        }
    }
    else
    {
        /* Can't be a valid read coil register request because the length
         * is incorrect. */
        eStatus = MB_EX_ILLEGAL_DATA_VALUE;
    }
    return eStatus;
}

#if MB_FUNC_WRITE_COIL_ENABLED > 0
eMBException
eMBFuncWriteCoil( UCHAR * pucFrame, USHORT * usLen )
{
    USHORT          usRegAddress;
    UCHAR           ucBuf[2];

    eMBException    eStatus = MB_EX_NONE;
    eMBErrorCode    eRegStatus;

    if( *usLen == ( MB_PDU_FUNC_WRITE_SIZE + MB_PDU_SIZE_MIN ) )
    {
        usRegAddress = ( USHORT )( pucFrame[MB_PDU_FUNC_WRITE_ADDR_OFF] << 8 );
        usRegAddress |= ( USHORT )( pucFrame[MB_PDU_FUNC_WRITE_ADDR_OFF + 1] );
//        usRegAddress++;

        if( ( pucFrame[MB_PDU_FUNC_WRITE_VALUE_OFF + 1] == 0x00 ) &&
            ( ( pucFrame[MB_PDU_FUNC_WRITE_VALUE_OFF] == 0xFF ) ||
              ( pucFrame[MB_PDU_FUNC_WRITE_VALUE_OFF] == 0x00 ) ) )
        {
            ucBuf[1] = 0;
            if( pucFrame[MB_PDU_FUNC_WRITE_VALUE_OFF] == 0xFF )
            {
                ucBuf[0] = 1;
            }
            else
            {
                ucBuf[0] = 0;
            }
            eRegStatus =
                eMBRegCoilsCB( &ucBuf[0], usRegAddress, 1, MB_REG_WRITE );

            /* If an error occured convert it into a Modbus exception. */
            if( eRegStatus != MB_ENOERR )
            {
                eStatus = prveMBError2Exception( eRegStatus );
            }
        }
        else
        {
            eStatus = MB_EX_ILLEGAL_DATA_VALUE;
        }
    }
    else
    {
        /* Can't be a valid write coil register request because the length
         * is incorrect. */
        eStatus = MB_EX_ILLEGAL_DATA_VALUE;
    }
    return eStatus;
}

#endif

#if MB_FUNC_WRITE_MULTIPLE_COILS_ENABLED > 0
eMBException
eMBFuncWriteMultipleCoils( UCHAR * pucFrame, USHORT * usLen )
{
    USHORT          usRegAddress;
    USHORT          usCoilCnt;
    UCHAR           ucByteCount;
    UCHAR           ucByteCountVerify;

    eMBException    eStatus = MB_EX_NONE;
    eMBErrorCode    eRegStatus;

    if( *usLen > ( MB_PDU_FUNC_WRITE_SIZE + MB_PDU_SIZE_MIN ) )
    {
        usRegAddress = ( USHORT )( pucFrame[MB_PDU_FUNC_WRITE_MUL_ADDR_OFF] << 8 );
        usRegAddress |= ( USHORT )( pucFrame[MB_PDU_FUNC_WRITE_MUL_ADDR_OFF + 1] );
//        usRegAddress++;

        usCoilCnt = ( USHORT )( pucFrame[MB_PDU_FUNC_WRITE_MUL_COILCNT_OFF] << 8 );
        usCoilCnt |= ( USHORT )( pucFrame[MB_PDU_FUNC_WRITE_MUL_COILCNT_OFF + 1] );

        ucByteCount = pucFrame[MB_PDU_FUNC_WRITE_MUL_BYTECNT_OFF];

        /* Compute the number of expected bytes in the request. */
        if( ( usCoilCnt & 0x0007 ) != 0 )
        {
            ucByteCountVerify = ( UCHAR )( usCoilCnt / 8 + 1 );
        }
        else
        {
            ucByteCountVerify = ( UCHAR )( usCoilCnt / 8 );
        }

        if( ( usCoilCnt >= 1 ) &&
            ( usCoilCnt <= MB_PDU_FUNC_WRITE_MUL_COILCNT_MAX ) &&
            ( ucByteCountVerify == ucByteCount ) )
        {
            eRegStatus =
                eMBRegCoilsCB( &pucFrame[MB_PDU_FUNC_WRITE_MUL_VALUES_OFF],
                               usRegAddress, usCoilCnt, MB_REG_WRITE );

            /* If an error occured convert it into a Modbus exception. */
            if( eRegStatus != MB_ENOERR )
            {
                eStatus = prveMBError2Exception( eRegStatus );
            }
            else
            {
                /* The response contains the function code, the starting address
                 * and the quantity of registers. We reuse the old values in the 
                 * buffer because they are still valid. */
                *usLen = MB_PDU_FUNC_WRITE_MUL_BYTECNT_OFF;
            }
        }
        else
        {
            eStatus = MB_EX_ILLEGAL_DATA_VALUE;
        }
    }
    else
    {
        /* Can't be a valid write coil register request because the length
         * is incorrect. */
        eStatus = MB_EX_ILLEGAL_DATA_VALUE;
    }
    return eStatus;
}

#endif

#endif
```

mbfuncdisc.c

``` c
/* ----------------------- System includes ----------------------------------*/
#include "stdlib.h"
#include "string.h"

/* ----------------------- Platform includes --------------------------------*/
#include "port.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbframe.h"
#include "mbproto.h"
#include "mbconfig.h"

/* ----------------------- Defines ------------------------------------------*/
#define MB_PDU_FUNC_READ_ADDR_OFF           ( MB_PDU_DATA_OFF )
#define MB_PDU_FUNC_READ_DISCCNT_OFF        ( MB_PDU_DATA_OFF + 2 )
#define MB_PDU_FUNC_READ_SIZE               ( 4 )
#define MB_PDU_FUNC_READ_DISCCNT_MAX        ( 0x07D0 )

/* ----------------------- Static functions ---------------------------------*/
eMBException    prveMBError2Exception( eMBErrorCode eErrorCode );

/* ----------------------- Start implementation -----------------------------*/

#if MB_FUNC_READ_COILS_ENABLED > 0

eMBException
eMBFuncReadDiscreteInputs( UCHAR * pucFrame, USHORT * usLen )
{
    USHORT          usRegAddress;
    USHORT          usDiscreteCnt;
    UCHAR           ucNBytes;
    UCHAR          *pucFrameCur;

    eMBException    eStatus = MB_EX_NONE;
    eMBErrorCode    eRegStatus;

    if( *usLen == ( MB_PDU_FUNC_READ_SIZE + MB_PDU_SIZE_MIN ) )
    {
        usRegAddress = ( USHORT )( pucFrame[MB_PDU_FUNC_READ_ADDR_OFF] << 8 );
        usRegAddress |= ( USHORT )( pucFrame[MB_PDU_FUNC_READ_ADDR_OFF + 1] );
//        usRegAddress++;

        usDiscreteCnt = ( USHORT )( pucFrame[MB_PDU_FUNC_READ_DISCCNT_OFF] << 8 );
        usDiscreteCnt |= ( USHORT )( pucFrame[MB_PDU_FUNC_READ_DISCCNT_OFF + 1] );

        /* Check if the number of registers to read is valid. If not
         * return Modbus illegal data value exception. 
         */
        if( ( usDiscreteCnt >= 1 ) &&
            ( usDiscreteCnt < MB_PDU_FUNC_READ_DISCCNT_MAX ) )
        {
            /* Set the current PDU data pointer to the beginning. */
            pucFrameCur = &pucFrame[MB_PDU_FUNC_OFF];
            *usLen = MB_PDU_FUNC_OFF;

            /* First byte contains the function code. */
            *pucFrameCur++ = MB_FUNC_READ_DISCRETE_INPUTS;
            *usLen += 1;

            /* Test if the quantity of coils is a multiple of 8. If not last
             * byte is only partially field with unused coils set to zero. */
            if( ( usDiscreteCnt & 0x0007 ) != 0 )
            {
                ucNBytes = ( UCHAR ) ( usDiscreteCnt / 8 + 1 );
            }
            else
            {
                ucNBytes = ( UCHAR ) ( usDiscreteCnt / 8 );
            }
            *pucFrameCur++ = ucNBytes;
            *usLen += 1;

            eRegStatus =
                eMBRegDiscreteCB( pucFrameCur, usRegAddress, usDiscreteCnt );

            /* If an error occured convert it into a Modbus exception. */
            if( eRegStatus != MB_ENOERR )
            {
                eStatus = prveMBError2Exception( eRegStatus );
            }
            else
            {
                /* The response contains the function code, the starting address
                 * and the quantity of registers. We reuse the old values in the 
                 * buffer because they are still valid. */
                *usLen += ucNBytes;;
            }
        }
        else
        {
            eStatus = MB_EX_ILLEGAL_DATA_VALUE;
        }
    }
    else
    {
        /* Can't be a valid read coil register request because the length
         * is incorrect. */
        eStatus = MB_EX_ILLEGAL_DATA_VALUE;
    }
    return eStatus;
}

#endif
```

mbfuncholding.c

``` c
/* ----------------------- System includes ----------------------------------*/
#include "stdlib.h"
#include "string.h"

/* ----------------------- Platform includes --------------------------------*/
#include "port.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbframe.h"
#include "mbproto.h"
#include "mbconfig.h"

/* ----------------------- Defines ------------------------------------------*/
#define MB_PDU_FUNC_READ_ADDR_OFF               ( MB_PDU_DATA_OFF + 0)
#define MB_PDU_FUNC_READ_REGCNT_OFF             ( MB_PDU_DATA_OFF + 2 )
#define MB_PDU_FUNC_READ_SIZE                   ( 4 )
#define MB_PDU_FUNC_READ_REGCNT_MAX             ( 0x007D )

#define MB_PDU_FUNC_WRITE_ADDR_OFF              ( MB_PDU_DATA_OFF + 0)
#define MB_PDU_FUNC_WRITE_VALUE_OFF             ( MB_PDU_DATA_OFF + 2 )
#define MB_PDU_FUNC_WRITE_SIZE                  ( 4 )

#define MB_PDU_FUNC_WRITE_MUL_ADDR_OFF          ( MB_PDU_DATA_OFF + 0 )
#define MB_PDU_FUNC_WRITE_MUL_REGCNT_OFF        ( MB_PDU_DATA_OFF + 2 )
#define MB_PDU_FUNC_WRITE_MUL_BYTECNT_OFF       ( MB_PDU_DATA_OFF + 4 )
#define MB_PDU_FUNC_WRITE_MUL_VALUES_OFF        ( MB_PDU_DATA_OFF + 5 )
#define MB_PDU_FUNC_WRITE_MUL_SIZE_MIN          ( 5 )
#define MB_PDU_FUNC_WRITE_MUL_REGCNT_MAX        ( 0x0078 )

#define MB_PDU_FUNC_READWRITE_READ_ADDR_OFF     ( MB_PDU_DATA_OFF + 0 )
#define MB_PDU_FUNC_READWRITE_READ_REGCNT_OFF   ( MB_PDU_DATA_OFF + 2 )
#define MB_PDU_FUNC_READWRITE_WRITE_ADDR_OFF    ( MB_PDU_DATA_OFF + 4 )
#define MB_PDU_FUNC_READWRITE_WRITE_REGCNT_OFF  ( MB_PDU_DATA_OFF + 6 )
#define MB_PDU_FUNC_READWRITE_BYTECNT_OFF       ( MB_PDU_DATA_OFF + 8 )
#define MB_PDU_FUNC_READWRITE_WRITE_VALUES_OFF  ( MB_PDU_DATA_OFF + 9 )
#define MB_PDU_FUNC_READWRITE_SIZE_MIN          ( 9 )

/* ----------------------- Static functions ---------------------------------*/
eMBException    prveMBError2Exception( eMBErrorCode eErrorCode );

/* ----------------------- Start implementation -----------------------------*/

#if MB_FUNC_WRITE_HOLDING_ENABLED > 0

eMBException
eMBFuncWriteHoldingRegister( UCHAR * pucFrame, USHORT * usLen )
{
    USHORT          usRegAddress;
    eMBException    eStatus = MB_EX_NONE;
    eMBErrorCode    eRegStatus;

    if( *usLen == ( MB_PDU_FUNC_WRITE_SIZE + MB_PDU_SIZE_MIN ) )
    {
        usRegAddress = ( USHORT )( pucFrame[MB_PDU_FUNC_WRITE_ADDR_OFF] << 8 );
        usRegAddress |= ( USHORT )( pucFrame[MB_PDU_FUNC_WRITE_ADDR_OFF + 1] );
//        usRegAddress++;

        /* Make callback to update the value. */
        eRegStatus = eMBRegHoldingCB( &pucFrame[MB_PDU_FUNC_WRITE_VALUE_OFF],
                                      usRegAddress, 1, MB_REG_WRITE );

        /* If an error occured convert it into a Modbus exception. */
        if( eRegStatus != MB_ENOERR )
        {
            eStatus = prveMBError2Exception( eRegStatus );
        }
    }
    else
    {
        /* Can't be a valid request because the length is incorrect. */
        eStatus = MB_EX_ILLEGAL_DATA_VALUE;
    }
    return eStatus;
}
#endif

#if MB_FUNC_WRITE_MULTIPLE_HOLDING_ENABLED > 0
eMBException
eMBFuncWriteMultipleHoldingRegister( UCHAR * pucFrame, USHORT * usLen )
{
    USHORT          usRegAddress;
    USHORT          usRegCount;
    UCHAR           ucRegByteCount;

    eMBException    eStatus = MB_EX_NONE;
    eMBErrorCode    eRegStatus;

    if( *usLen >= ( MB_PDU_FUNC_WRITE_MUL_SIZE_MIN + MB_PDU_SIZE_MIN ) )
    {
        usRegAddress = ( USHORT )( pucFrame[MB_PDU_FUNC_WRITE_MUL_ADDR_OFF] << 8 );
        usRegAddress |= ( USHORT )( pucFrame[MB_PDU_FUNC_WRITE_MUL_ADDR_OFF + 1] );
//        usRegAddress++;

        usRegCount = ( USHORT )( pucFrame[MB_PDU_FUNC_WRITE_MUL_REGCNT_OFF] << 8 );
        usRegCount |= ( USHORT )( pucFrame[MB_PDU_FUNC_WRITE_MUL_REGCNT_OFF + 1] );

        ucRegByteCount = pucFrame[MB_PDU_FUNC_WRITE_MUL_BYTECNT_OFF];

        if( ( usRegCount >= 1 ) &&
            ( usRegCount <= MB_PDU_FUNC_WRITE_MUL_REGCNT_MAX ) &&
            ( ucRegByteCount == ( UCHAR ) ( 2 * usRegCount ) ) )
        {
            /* Make callback to update the register values. */
            eRegStatus =
                eMBRegHoldingCB( &pucFrame[MB_PDU_FUNC_WRITE_MUL_VALUES_OFF],
                                 usRegAddress, usRegCount, MB_REG_WRITE );

            /* If an error occured convert it into a Modbus exception. */
            if( eRegStatus != MB_ENOERR )
            {
                eStatus = prveMBError2Exception( eRegStatus );
            }
            else
            {
                /* The response contains the function code, the starting
                 * address and the quantity of registers. We reuse the
                 * old values in the buffer because they are still valid.
                 */
                *usLen = MB_PDU_FUNC_WRITE_MUL_BYTECNT_OFF;
            }
        }
        else
        {
            eStatus = MB_EX_ILLEGAL_DATA_VALUE;
        }
    }
    else
    {
        /* Can't be a valid request because the length is incorrect. */
        eStatus = MB_EX_ILLEGAL_DATA_VALUE;
    }
    return eStatus;
}
#endif

#if MB_FUNC_READ_HOLDING_ENABLED > 0

eMBException
eMBFuncReadHoldingRegister( UCHAR * pucFrame, USHORT * usLen )
{
    USHORT          usRegAddress;
    USHORT          usRegCount;
    UCHAR          *pucFrameCur;

    eMBException    eStatus = MB_EX_NONE;
    eMBErrorCode    eRegStatus;

    if( *usLen == ( MB_PDU_FUNC_READ_SIZE + MB_PDU_SIZE_MIN ) )
    {
        usRegAddress = ( USHORT )( pucFrame[MB_PDU_FUNC_READ_ADDR_OFF] << 8 );
        usRegAddress |= ( USHORT )( pucFrame[MB_PDU_FUNC_READ_ADDR_OFF + 1] );
//        usRegAddress++;

        usRegCount = ( USHORT )( pucFrame[MB_PDU_FUNC_READ_REGCNT_OFF] << 8 );
        usRegCount = ( USHORT )( pucFrame[MB_PDU_FUNC_READ_REGCNT_OFF + 1] );

        /* Check if the number of registers to read is valid. If not
         * return Modbus illegal data value exception. 
         */
        if( ( usRegCount >= 1 ) && ( usRegCount <= MB_PDU_FUNC_READ_REGCNT_MAX ) )
        {
            /* Set the current PDU data pointer to the beginning. */
            pucFrameCur = &pucFrame[MB_PDU_FUNC_OFF];
            *usLen = MB_PDU_FUNC_OFF;

            /* First byte contains the function code. */
            *pucFrameCur++ = MB_FUNC_READ_HOLDING_REGISTER;
            *usLen += 1;

            /* Second byte in the response contain the number of bytes. */
            *pucFrameCur++ = ( UCHAR ) ( usRegCount * 2 );
            *usLen += 1;

            /* Make callback to fill the buffer. */
            eRegStatus = eMBRegHoldingCB( pucFrameCur, usRegAddress, usRegCount, MB_REG_READ );
            /* If an error occured convert it into a Modbus exception. */
            if( eRegStatus != MB_ENOERR )
            {
                eStatus = prveMBError2Exception( eRegStatus );
            }
            else
            {
                *usLen += usRegCount * 2;
            }
        }
        else
        {
            eStatus = MB_EX_ILLEGAL_DATA_VALUE;
        }
    }
    else
    {
        /* Can't be a valid request because the length is incorrect. */
        eStatus = MB_EX_ILLEGAL_DATA_VALUE;
    }
    return eStatus;
}

#endif

#if MB_FUNC_READWRITE_HOLDING_ENABLED > 0

eMBException
eMBFuncReadWriteMultipleHoldingRegister( UCHAR * pucFrame, USHORT * usLen )
{
    USHORT          usRegReadAddress;
    USHORT          usRegReadCount;
    USHORT          usRegWriteAddress;
    USHORT          usRegWriteCount;
    UCHAR           ucRegWriteByteCount;
    UCHAR          *pucFrameCur;

    eMBException    eStatus = MB_EX_NONE;
    eMBErrorCode    eRegStatus;

    if( *usLen >= ( MB_PDU_FUNC_READWRITE_SIZE_MIN + MB_PDU_SIZE_MIN ) )
    {
        usRegReadAddress = ( USHORT )( pucFrame[MB_PDU_FUNC_READWRITE_READ_ADDR_OFF] << 8U );
        usRegReadAddress |= ( USHORT )( pucFrame[MB_PDU_FUNC_READWRITE_READ_ADDR_OFF + 1] );
//        usRegReadAddress++;

        usRegReadCount = ( USHORT )( pucFrame[MB_PDU_FUNC_READWRITE_READ_REGCNT_OFF] << 8U );
        usRegReadCount |= ( USHORT )( pucFrame[MB_PDU_FUNC_READWRITE_READ_REGCNT_OFF + 1] );

        usRegWriteAddress = ( USHORT )( pucFrame[MB_PDU_FUNC_READWRITE_WRITE_ADDR_OFF] << 8U );
        usRegWriteAddress |= ( USHORT )( pucFrame[MB_PDU_FUNC_READWRITE_WRITE_ADDR_OFF + 1] );
//        usRegWriteAddress++;

        usRegWriteCount = ( USHORT )( pucFrame[MB_PDU_FUNC_READWRITE_WRITE_REGCNT_OFF] << 8U );
        usRegWriteCount |= ( USHORT )( pucFrame[MB_PDU_FUNC_READWRITE_WRITE_REGCNT_OFF + 1] );

        ucRegWriteByteCount = pucFrame[MB_PDU_FUNC_READWRITE_BYTECNT_OFF];

        if( ( usRegReadCount >= 1 ) && ( usRegReadCount <= 0x7D ) &&
            ( usRegWriteCount >= 1 ) && ( usRegWriteCount <= 0x79 ) &&
            ( ( 2 * usRegWriteCount ) == ucRegWriteByteCount ) )
        {
            /* Make callback to update the register values. */
            eRegStatus = eMBRegHoldingCB( &pucFrame[MB_PDU_FUNC_READWRITE_WRITE_VALUES_OFF],
                                          usRegWriteAddress, usRegWriteCount, MB_REG_WRITE );

            if( eRegStatus == MB_ENOERR )
            {
                /* Set the current PDU data pointer to the beginning. */
                pucFrameCur = &pucFrame[MB_PDU_FUNC_OFF];
                *usLen = MB_PDU_FUNC_OFF;

                /* First byte contains the function code. */
                *pucFrameCur++ = MB_FUNC_READWRITE_MULTIPLE_REGISTERS;
                *usLen += 1;

                /* Second byte in the response contain the number of bytes. */
                *pucFrameCur++ = ( UCHAR ) ( usRegReadCount * 2 );
                *usLen += 1;

                /* Make the read callback. */
                eRegStatus =
                    eMBRegHoldingCB( pucFrameCur, usRegReadAddress, usRegReadCount, MB_REG_READ );
                if( eRegStatus == MB_ENOERR )
                {
                    *usLen += 2 * usRegReadCount;
                }
            }
            if( eRegStatus != MB_ENOERR )
            {
                eStatus = prveMBError2Exception( eRegStatus );
            }
        }
        else
        {
            eStatus = MB_EX_ILLEGAL_DATA_VALUE;
        }
    }
    return eStatus;
}

#endif
```

mbfuncinput.c

``` c
/* ----------------------- System includes ----------------------------------*/
#include "stdlib.h"
#include "string.h"

/* ----------------------- Platform includes --------------------------------*/
#include "port.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbframe.h"
#include "mbproto.h"
#include "mbconfig.h"

/* ----------------------- Defines ------------------------------------------*/
#define MB_PDU_FUNC_READ_ADDR_OFF           ( MB_PDU_DATA_OFF )
#define MB_PDU_FUNC_READ_REGCNT_OFF         ( MB_PDU_DATA_OFF + 2 )
#define MB_PDU_FUNC_READ_SIZE               ( 4 )
#define MB_PDU_FUNC_READ_REGCNT_MAX         ( 0x007D )

#define MB_PDU_FUNC_READ_RSP_BYTECNT_OFF    ( MB_PDU_DATA_OFF )

/* ----------------------- Static functions ---------------------------------*/
eMBException    prveMBError2Exception( eMBErrorCode eErrorCode );

/* ----------------------- Start implementation -----------------------------*/
#if MB_FUNC_READ_INPUT_ENABLED > 0

eMBException
eMBFuncReadInputRegister( UCHAR * pucFrame, USHORT * usLen )
{
    USHORT          usRegAddress;
    USHORT          usRegCount;
    UCHAR          *pucFrameCur;

    eMBException    eStatus = MB_EX_NONE;
    eMBErrorCode    eRegStatus;

    if( *usLen == ( MB_PDU_FUNC_READ_SIZE + MB_PDU_SIZE_MIN ) )
    {
        usRegAddress = ( USHORT )( pucFrame[MB_PDU_FUNC_READ_ADDR_OFF] << 8 );
        usRegAddress |= ( USHORT )( pucFrame[MB_PDU_FUNC_READ_ADDR_OFF + 1] );
//        usRegAddress++;

        usRegCount = ( USHORT )( pucFrame[MB_PDU_FUNC_READ_REGCNT_OFF] << 8 );
        usRegCount |= ( USHORT )( pucFrame[MB_PDU_FUNC_READ_REGCNT_OFF + 1] );

        /* Check if the number of registers to read is valid. If not
         * return Modbus illegal data value exception. 
         */
        if( ( usRegCount >= 1 )
            && ( usRegCount < MB_PDU_FUNC_READ_REGCNT_MAX ) )
        {
            /* Set the current PDU data pointer to the beginning. */
            pucFrameCur = &pucFrame[MB_PDU_FUNC_OFF];
            *usLen = MB_PDU_FUNC_OFF;

            /* First byte contains the function code. */
            *pucFrameCur++ = MB_FUNC_READ_INPUT_REGISTER;
            *usLen += 1;

            /* Second byte in the response contain the number of bytes. */
            *pucFrameCur++ = ( UCHAR )( usRegCount * 2 );
            *usLen += 1;

            eRegStatus =
                eMBRegInputCB( pucFrameCur, usRegAddress, usRegCount );

            /* If an error occured convert it into a Modbus exception. */
            if( eRegStatus != MB_ENOERR )
            {
                eStatus = prveMBError2Exception( eRegStatus );
            }
            else
            {
                *usLen += usRegCount * 2;
            }
        }
        else
        {
            eStatus = MB_EX_ILLEGAL_DATA_VALUE;
        }
    }
    else
    {
        /* Can't be a valid read input register request because the length
         * is incorrect. */
        eStatus = MB_EX_ILLEGAL_DATA_VALUE;
    }
    return eStatus;
}

#endif
```

至此，移植和优化操作均已完成

### :house: 效果展示

* 输入寄存器

  假设输入寄存器起始地址为0x0010，寄存器数量为50，寄存器数据如下

  ``` c
  usRegInputBuf[0] = 0xaabb;
  usRegInputBuf[1] = 0xccdd;
  usRegInputBuf[2] = 0xeeff;
  usRegInputBuf[3] = 0x1122;
  usRegInputBuf[4] = 0x3344;
  usRegInputBuf[5] = 0x5566;
  ```

  * 如果我们要从地址0x0010读取一个寄存器值，对应的输入输出如下（绿色输入，蓝色输出）

    ![image-20210627163131765](https://gitee.com/LucasXm/img/raw/master/img//image-20210627163131765.png)

  * 如果我们要从地址0x0012读取3个寄存器值，对应的输入输出如下（绿色输入，蓝色输出）

    ![image-20210627163221242](https://gitee.com/LucasXm/img/raw/master/img//image-20210627163221242.png)

* 保持寄存器

  假设保持寄存器起始地址为0x0010，寄存器数量为50，寄存器值初始时为全空

  * 初始时，从地址0x0010读取5个寄存器值，对应的输入输出如下（绿色输入，蓝色输出）

    ![image-20210627163501871](https://gitee.com/LucasXm/img/raw/master/img//image-20210627163501871.png)

  * 从地址0x0011写一个寄存器值（0x1234），对应的输入输出如下（绿色输入，蓝色输出）

    ![image-20210627163608645](https://gitee.com/LucasXm/img/raw/master/img//image-20210627163608645.png)

  * 从地址0x0012写3个寄存器值（0x1234，0x5678，0x9abc），对应的输入输出如下（绿色输入，蓝色输出）

    ![image-20210627163848159](https://gitee.com/LucasXm/img/raw/master/img//image-20210627163848159.png)

  * 从地址0x0010读取5个寄存器值，对应的输入输出如下（绿色输入，蓝色输出）

    ![image-20210627163944065](https://gitee.com/LucasXm/img/raw/master/img//image-20210627163944065.png)

  ***需要注意的时，当前这些数据都是保持在RAM中的，掉电并不会保存。实际使用的时候，还需要把有效数据，保存到flash中***

 * 线圈寄存器和离散寄存器的操作方法和上述基本一样，对应着《Modbus协议中文版【完整版】》的请求PDU及响应PDU去测试就OK

   

## 变更日志

### v1.0（2021-06-27）

 * :star:完成了移植步骤总结
 * :star:完成了优化方法总结
 * :star:完成了效果展示总结

### v0.2（2021-06-27）

 * 完成了“**基于Freemodbus使用rs485实现通信**”任务

### v0.1（2021-06-20）

 * 完成了“**基于Freemodbus使用uart实现通信**”任务







