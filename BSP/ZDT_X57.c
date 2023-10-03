#include "ZDT_X57.h"

/**********************************************************
***	ZDT_X57�����ջ���������
***	��д���ߣ�ZHANGDATOU
***	����֧�֣��Ŵ�ͷ�ջ��ŷ�
***	�Ա����̣�https://zhangdatou.taobao.com
***	CSDN���ͣ�http s://blog.csdn.net/zhangdatou666
***	qq����Ⱥ��262438510
**********************************************************/

/**
	*	@brief		����ǰλ������
	*	@param		addr	�������ַ
	*	@retval		��ַ + ������ + ����״̬ + У���ֽ�
	*/
void ZDT_X57_Reset_CurPos_To_Zero(uint8_t addr)
{
	__IO uint8_t cmd[16] = {0};
	
	// װ������
	cmd[0] =  addr;												// ��ַ
	cmd[1] =  0x0A;												// ������
	cmd[2] =  0x6D;												// ������
	cmd[3] =  0x6B;												// У���ֽ�
	
	// ��������
	can_SendCmd(cmd, 4);
}

/**
	*	@brief		�����ѹ����/��������/���ȱ���/��ת����
	*	@param		addr	�������ַ
	*	@retval		��ַ + ������ + ����״̬ + У���ֽ�
	*/
void ZDT_X57_Reset_Clog_Pro(uint8_t addr)
{
	__IO uint8_t cmd[16] = {0};
	
	// װ������
	cmd[0] =  addr;												// ��ַ
	cmd[1] =  0x0E;												// ������
	cmd[2] =  0x52;												// ������
	cmd[3] =  0x6B;												// У���ֽ�
	
	// ��������
	can_SendCmd(cmd, 4);
}

/**
	*	@brief		��ȡϵͳ����
	*	@param		addr	�������ַ
	*	@param		s			��ϵͳ��������
	*	@retval		��ַ + ������ + ����״̬ + У���ֽ�
	*/
void ZDT_X57_Read_Sys_Params(uint8_t addr, SysParams_t s)
{
	__IO uint8_t cmd[16] = {0};
	
	// װ������
	cmd[0] =  addr;												// ��ַ

	switch(s)															// ������
	{
		case S_VER	: cmd[1] = 0x1F; break;
		case S_RL		: cmd[1] = 0x20; break;
		case S_PID	: cmd[1] = 0x21; break;
		case S_VBUS	: cmd[1] = 0x24; break;
		case S_CBUS	: cmd[1] = 0x26; break;
		case S_CPHA	: cmd[1] = 0x27; break;
		case S_ENC	: cmd[1] = 0x30; break;
		case S_ENCL	: cmd[1] = 0x31; break;
		case S_TPOS	: cmd[1] = 0x33; break;
		case S_VEL	: cmd[1] = 0x35; break;
		case S_CPOS	: cmd[1] = 0x36; break;
		case S_PERR	: cmd[1] = 0x37; break;
		case S_TEMP	: cmd[1] = 0x39; break;
		case S_FLAG	: cmd[1] = 0x3A; break;
		case S_Conf	: cmd[1] = 0x42; break;
		case S_State: cmd[1] = 0x43; break;
		default: break;
	}

	cmd[2] =  0x6B;												// У���ֽ�
	
	// ��������
	can_SendCmd(cmd, 3);
}


/**
	*	@brief		��������λ��ģʽ
	*	@param		addr	�������ַ
	*	@param		dir			������										��0ΪCW������ֵΪCCW
	*	@param		acc			�����ټ��ٶ�(RPM/s)			��0ΪCW������ֵΪCCW
	*	@param		dec			�����ټ��ٶ�(RPM/s)			��0ΪCW������ֵΪCCW
	*	@param		velocity������ٶ�(RPM)					����Χ0.0 - 4000.0RPM
	*	@param		position��λ��(��)								����Χ0.0��- (2^32 - 1)��
	*	@param		raf			����λλ��/����λ�ñ�־	��0Ϊ���λ�ã�����ֵΪ����λ��
	*	@param		multi		�����ͬ����־						��0Ϊ�����ã�����ֵ����
	*	@retval		��ַ + ������ + ����״̬ + У���ֽ�
	*/
void ZDT_X57_Traj_Position_Control(uint8_t addr, uint8_t dir, uint16_t acc, uint16_t dec, float velocity, float position, uint8_t raf, uint8_t multi)
{
	__IO uint8_t cmd[16] = {0}; __IO uint16_t vel = 0; __IO uint32_t pos = 0;

	// ���ٶȺ�λ�÷Ŵ�10�����͹�ȥ
	vel = (uint16_t)ABS(velocity * 10.0f); pos = (uint32_t)ABS(position * 10.0f);

	// װ������
	cmd[0]  =  addr;											// ��ַ
	cmd[1]  =  0xFD;											// ������
	cmd[2]  =  dir;												// ���ţ�����
	cmd[3]  =  (uint8_t)(acc >> 8);				// ���ټ��ٶ�(RPM/s)��8λ�ֽ�
	cmd[4]  =  (uint8_t)(acc >> 0);				// ���ټ��ٶ�(RPM/s)��8λ�ֽ�	
	cmd[5]  =  (uint8_t)(dec >> 8);				// ���ټ��ٶ�(RPM/s)��8λ�ֽ�
	cmd[6]  =  (uint8_t)(dec >> 0);				// ���ټ��ٶ�(RPM/s)��8λ�ֽ�	
	cmd[7]  =  (uint8_t)(vel >> 8);				// ����ٶ�(RPM)��8λ�ֽ�
	cmd[8]  =  (uint8_t)(vel >> 0);				// ����ٶ�(RPM)��8λ�ֽ�	
	cmd[9]  =  (uint8_t)(pos >> 24);			// λ��(bit24 - bit31)
	cmd[10] =  (uint8_t)(pos >> 16);			// λ��(bit16 - bit23)
	cmd[11] =  (uint8_t)(pos >> 8);				// λ��(bit8  - bit15)
	cmd[12] =  (uint8_t)(pos >> 0);				// λ��(bit0  - bit7 )
	cmd[13] =  raf;												// ��λλ��/����λ�ñ�־
	cmd[14] =  multi;											// ���ͬ���˶���־
	cmd[15] =  0x6B;											// У���ֽ�
	
	// ��������
	can_SendCmd(cmd, 16);
}

/**
	*	@brief		����ֹͣ�����п���ģʽ��ͨ�ã�
	*	@param		addr	�������ַ
	*	@param		multi	�����ͬ����־��0Ϊ�����ã�����ֵ����
	*	@retval		��ַ + ������ + ����״̬ + У���ֽ�
	*/
void ZDT_X57_Stop_Now(uint8_t addr, uint8_t multi)
{
	__IO uint8_t cmd[16] = {0};
	
	// װ������
	cmd[0] =  addr;												// ��ַ
	cmd[1] =  0xFE;												// ������
	cmd[2] =  0x98;												// ������
	cmd[3] =  multi;											// ���ͬ���˶���־
	cmd[4] =  0x6B;												// У���ֽ�
	
	// ��������
	can_SendCmd(cmd, 5);
}

/**
	*	@brief		���ͬ���˶�
	*	@param		addr	�������ַ
	*	@retval		��ַ + ������ + ����״̬ + У���ֽ�
	*/
void ZDT_X57_Synchronous_motion(uint8_t addr)
{
	__IO uint8_t cmd[16] = {0};
	
	// װ������
	cmd[0] =  addr;												// ��ַ
	cmd[1] =  0xFF;												// ������
	cmd[2] =  0x66;												// ������
	cmd[3] =  0x6B;												// У���ֽ�
	
	// ��������
	can_SendCmd(cmd, 4);
}
