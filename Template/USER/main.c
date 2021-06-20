#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "beep.h"
#include "key.h"
#include "mb.h"
#include "mbutils.h"

////输入寄存器起始地址
//#define REG_INPUT_START 0x0000

////输入寄存器数量
//#define REG_INPUT_NREGS 8

////保持寄存器起始地址
//#define REG_HOLDING_START 0x0000

////保持寄存器数量
//#define REG_HOLDING_NREGS 8

////线圈起始地址
//#define REG_COILS_START 0x0000

////线圈数量
//#define REG_COILS_SIZE 16

////开关寄存器起始地址
//#define REG_DISCRETE_START 0x0000

////开关寄存器数量
//#define REG_DISCRETE_SIZE 16



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


