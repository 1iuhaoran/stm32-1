#ifndef __CAN_H
#define __CAN_H

#include "board.h"
#include "fifo.h"

/**********************************************************
***	ZDT_X57�����ջ���������
***	��д���ߣ�ZHANGDATOU
***	����֧�֣��Ŵ�ͷ�ջ��ŷ�
***	�Ա����̣�https://zhangdatou.taobao.com
***	CSDN���ͣ�http s://blog.csdn.net/zhangdatou666
***	qq����Ⱥ��262438510
**********************************************************/

typedef struct {
	__IO CanRxMsg CAN_RxMsg;
	__IO CanTxMsg CAN_TxMsg;

	__IO bool rxFrameFlag;
}CAN_t;

void can_SendCmd(__IO uint8_t *cmd, uint8_t len);

extern __IO CAN_t can;

#endif
