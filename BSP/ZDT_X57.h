#ifndef __ZDT_X57_H
#define __ZDT_X57_H

#include "can.h"

/**********************************************************
***	ZDT_X57�����ջ���������
***	��д���ߣ�ZHANGDATOU
***	����֧�֣��Ŵ�ͷ�ջ��ŷ�
***	�Ա����̣�https://zhangdatou.taobao.com
***	CSDN���ͣ�http s://blog.csdn.net/zhangdatou666
***	qq����Ⱥ��262438510
**********************************************************/

#define		ABS(x)		((x) > 0 ? (x) : -(x)) 

typedef enum {
	S_VER   = 0,			/* ��ȡ�̼��汾�Ͷ�Ӧ��Ӳ���汾 */
	S_RL    = 1,			/* ��ȡ��ȡ���������� */
	S_PID   = 2,			/* ��ȡPID���� */
	S_VBUS  = 3,			/* ��ȡ���ߵ�ѹ */
	S_CBUS  = 4,			/* ��ȡ����ƽ������ */
	S_CPHA  = 5,			/* ��ȡ����� */
	S_ENC   = 6,			/* ��ȡ������ԭʼֵ */
	S_ENCL  = 7,			/* ��ȡ�������Ի�У׼��ı�����ֵ */
	S_TPOS  = 8,			/* ��ȡ���Ŀ��λ�� */
	S_VEL   = 9,			/* ��ȡ���ʵʱת�� */
	S_CPOS  = 10,			/* ��ȡ���ʵʱλ�� */
	S_PERR  = 11,			/* ��ȡ���λ����� */
	S_TEMP  = 12,			/* ��ȡ���ʵʱ�¶� */
	S_FLAG  = 13,			/* ��ȡ״̬��־λ */
	S_Conf  = 14,			/* ��ȡ�������� */
	S_State = 15,			/* ��ȡϵͳ״̬���� */
}SysParams_t;

void ZDT_X57_Reset_CurPos_To_Zero(uint8_t addr);
void ZDT_X57_Reset_Clog_Pro(uint8_t addr);
void ZDT_X57_Read_Sys_Params(uint8_t addr, SysParams_t s);
void ZDT_X57_Traj_Position_Control(uint8_t addr, uint8_t dir, uint16_t acc, uint16_t dec, float velocity, float position, uint8_t raf, uint8_t multi);
void ZDT_X57_Stop_Now(uint8_t addr, uint8_t multi);
void ZDT_X57_Synchronous_motion(uint8_t addr);

#endif
