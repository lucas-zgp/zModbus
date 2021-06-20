#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "beep.h"
#include "key.h"
#include "mb.h"
#include "mbutils.h"

////����Ĵ�����ʼ��ַ
//#define REG_INPUT_START 0x0000

////����Ĵ�������
//#define REG_INPUT_NREGS 8

////���ּĴ�����ʼ��ַ
//#define REG_HOLDING_START 0x0000

////���ּĴ�������
//#define REG_HOLDING_NREGS 8

////��Ȧ��ʼ��ַ
//#define REG_COILS_START 0x0000

////��Ȧ����
//#define REG_COILS_SIZE 16

////���ؼĴ�����ʼ��ַ
//#define REG_DISCRETE_START 0x0000

////���ؼĴ�������
//#define REG_DISCRETE_SIZE 16



#define REG_INPUT_START 0x0010 //������ʼ��ַ��PLC��ַ��base 1��
#define REG_INPUT_NREGS 50   //���üĴ�������
static USHORT usRegInputBuf[REG_INPUT_NREGS];


#define REG_HOLDING_START 0x0010 //������ʼ��ַ��PLC��ַ��base 1��
#define REG_HOLDING_NREGS 200  //���üĴ�������
static USHORT usRegHoldingBuf[REG_HOLDING_NREGS];


#define REG_DISCRETE_START		0x0010 //������ʼ��ַ��PLC��ַ��base 1��
#define REG_DISCRETE_NREGS		20   //���üĴ�������
#define REG_DISCRETE_SIZE		REG_DISCRETE_NREGS * 8 //������Ȧ/��ɢ�Ĵ�����ʾ���ǿ����������1���ֽڿ��Ա���8��������
static UCHAR ucRegDiscreteBuf[REG_DISCRETE_NREGS];


#define REG_COILS_START 0x0010 //������ʼ��ַ��PLC��ַ��base 1��
#define REG_COILS_NREGS 20 //���üĴ�������
#define REG_COILS_SIZE REG_COILS_NREGS * 8 //������Ȧ/��ɢ�Ĵ�����ʾ���ǿ����������1���ֽڿ��Ա���8��������
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
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);//����ϵͳ�ж����ȼ�����2
	delay_init(168);		//��ʱ��ʼ�� 

	eMBInit(MB_RTU, 0x01, 0x01, 9600, MB_PAR_NONE);
	eMBEnable();  

	while(1)
	{
		(void)eMBPoll();
	}
}





/**
 * @brief ������Ĵ����ķ�����
 * 		��mbfuncinput.c��eMBFuncReadInputRegister�����б����á�
 * 		�书�ܿ��Լ����Ϊ��ҵ������е�usRegInputBuf�����ֵcopy��pucRegBuffer�����С�
 * @param pucRegBuffer: ��������Ĵ���ֵ�Ļ���
 * @param usAddress: ����Ĵ�������ʼ��ַ
 * @param usNRegs: ����Ĵ���������
 * @return modbusִ�еĴ���״̬��
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
 * @brief �����ּĴ����ķ�����
 * 		��mbfuncholding.c��eMBFuncReadHoldingRegister, eMBFuncReadWriteMultipleHoldingRegister,
 * 		eMBFuncWriteHoldingRegister, eMBFuncWriteMultipleHoldingRegister�ĸ������б����á�
 * 		�书�ܿ��Լ����Ϊ��ҵ������е�usRegInputBuf�����ֵcopy��pucRegBuffer�����С�
 * @param pucRegBuffer: ���汣�ּĴ���ֵ�Ļ���
 * @param usAddress: ����Ĵ�������ʼ��ַ
 * @param usNRegs: ���ּĴ���������
 * @param eMode: �������ּĴ�����ģʽ���� or д��
 * @return modbusִ�еĴ���״̬��
 */
eMBErrorCode eMBRegHoldingCB(UCHAR * pucRegBuffer, uint16_t usAddress, uint16_t usNRegs, eMBRegisterMode eMode)
{
	//����״̬
	eMBErrorCode eStatus = MB_ENOERR;
	//ƫ����
	int16_t iRegIndex;

	//�жϼĴ����ǲ����ڷ�Χ��
	if (((int16_t) usAddress >= REG_HOLDING_START) && (usAddress + usNRegs <= REG_HOLDING_START + REG_HOLDING_NREGS))
	{
		//����ƫ����
		iRegIndex = (int16_t) (usAddress - REG_HOLDING_START);

		switch (eMode)
		{
		//��������
		case MB_REG_READ:
			while (usNRegs > 0)
			{
				*pucRegBuffer++ = (uint8_t) (usRegHoldingBuf[iRegIndex] >> 8);
				*pucRegBuffer++ = (uint8_t) (usRegHoldingBuf[iRegIndex] & 0xFF);
				iRegIndex++;
				usNRegs--;
			}
			break;

			//д������
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
		//���ش���״̬
		eStatus = MB_ENOREG;
	}

	return eStatus;
}





/**
 * @brief ��д��Ȧ�Ĵ����ķ�����
 * @param pucRegBuffer: ������Ȧ�Ĵ���ֵ�Ļ���
 * @param usAddress: ��Ȧ�Ĵ������׵�ַ
 * @param usNCoils: ��Ȧ�Ĵ������������ֽ��������� x 8��
 * @param eMode: ������Ȧ�Ĵ�����ģʽ���� or д��
 * @return modbusִ�еĴ���״̬��
 */
eMBErrorCode eMBRegCoilsCB(UCHAR * pucRegBuffer, uint16_t usAddress, uint16_t usNCoils, eMBRegisterMode eMode)
{
	//����״̬
	eMBErrorCode eStatus = MB_ENOERR;
	//�Ĵ�������
	int16_t iNCoils = (int16_t) usNCoils;
	//�Ĵ���ƫ����
	int16_t usBitOffset;

	//���Ĵ����Ƿ���ָ����Χ��
	if (((int16_t) usAddress >= REG_COILS_START) && (usAddress + usNCoils <= REG_COILS_START + REG_COILS_SIZE))
	{
		//����Ĵ���ƫ����
		usBitOffset = (int16_t) (usAddress - REG_COILS_START);
		switch (eMode)
		{
		//������
		case MB_REG_READ:
			while (iNCoils > 0)
			{
				*pucRegBuffer++ = xMBUtilGetBits(ucRegCoilsBuf, usBitOffset, (uint8_t) (iNCoils > 8 ? 8 : iNCoils));
				iNCoils -= 8;
				usBitOffset += 8;
			}
			break;

			//д����
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
 * @brief ����ɢ�Ĵ����ķ�����
 * @param pucRegBuffer: ������ɢ�Ĵ���ֵ�Ļ���
 * @param usAddress: ��ɢ�Ĵ������׵�ַ
 * @param usNDiscrete: ��ɢ�Ĵ������������ֽ��������� x 8��
 * @return modbusִ�еĴ���״̬��
 */
eMBErrorCode eMBRegDiscreteCB(UCHAR * pucRegBuffer, uint16_t usAddress, uint16_t usNDiscrete)
{
	//����״̬
	eMBErrorCode eStatus = MB_ENOERR;
	//�Ĵ�������
	int16_t iNDiscrete = (int16_t) usNDiscrete;
	//ƫ����
	uint16_t usBitOffset;  

	//�жϼĴ���ʱ�����ƶ���Χ��
	if (((int16_t) usAddress >= REG_DISCRETE_START)
			&& (usAddress + usNDiscrete <= REG_DISCRETE_START + REG_DISCRETE_SIZE))
	{
		//���ƫ����
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


