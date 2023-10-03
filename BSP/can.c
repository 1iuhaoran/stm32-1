#include "can.h"

/**********************************************************
***	ZDT_X57�����ջ���������
***	��д���ߣ�ZHANGDATOU
***	����֧�֣��Ŵ�ͷ�ջ��ŷ�
***	�Ա����̣�https://zhangdatou.taobao.com
***	CSDN���ͣ�http s://blog.csdn.net/zhangdatou666
***	qq����Ⱥ��262438510
**********************************************************/

__IO CAN_t can = {0};

/**
	* @brief   CAN1_RX0�����ж�
	* @param   ��
	* @retval  ��
	*/
void USB_LP_CAN1_RX0_IRQHandler(void)
{
	// ����һ������
	CAN_Receive(CAN1, CAN_FIFO0, (CanRxMsg *)(&can.CAN_RxMsg));
	
	// һ֡���ݽ�����ɣ���λ֡��־λ
	can.rxFrameFlag = true;
}

/**
	* @brief   CAN���Ͷ���ֽ�
	* @param   ��
	* @retval  ��
	*/
void can_SendCmd(__IO uint8_t *cmd, uint8_t len)
{
	__IO uint8_t i = 0, j = 0, k = 0, l = 0, packNum = 0;

	// ��ȥID��ַ�͹����������ݳ���
	j = len - 2;

	// ��������
	while(i < j)
	{
		// ���ݸ���
		k = j - i;

		// ��仺��
		can.CAN_TxMsg.StdId = 0x00;
		can.CAN_TxMsg.ExtId = ((uint32_t)cmd[0] << 8) | (uint32_t)packNum;
		can.CAN_TxMsg.Data[0] = cmd[1];
		can.CAN_TxMsg.IDE = CAN_Id_Extended;
		can.CAN_TxMsg.RTR = CAN_RTR_Data;

		// С��8�ֽ�����
		if(k < 8)
		{
			for(l=0; l < k; l++,i++) { can.CAN_TxMsg.Data[l + 1] = cmd[i + 2]; } can.CAN_TxMsg.DLC = k + 1;
		}
		// ����8�ֽ�����ְ����ͣ�ÿ��������෢��8���ֽ�
		else
		{
			for(l=0; l < 7; l++,i++) { can.CAN_TxMsg.Data[l + 1] = cmd[i + 2]; } can.CAN_TxMsg.DLC = 8;
		}

		// ��������
		CAN_Transmit(CAN1, (CanTxMsg *)(&can.CAN_TxMsg));

		// ��¼���͵ĵڼ���������
		++packNum;
	}
}
