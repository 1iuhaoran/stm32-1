#include "board.h"

/**********************************************************
***	ZDT_X57�����ջ���������
***	��д���ߣ�ZHANGDATOU
***	����֧�֣��Ŵ�ͷ�ջ��ŷ�
***	�Ա����̣�https://zhangdatou.taobao.com
***	CSDN���ͣ�http s://blog.csdn.net/zhangdatou666
***	qq����Ⱥ��262438510
**********************************************************/

/**
	* @brief   ����NVIC������
	* @param   ��
	* @retval  ��
	*/
void nvic_init(void)
{	
	// 4bit��ռ���ȼ�λ
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
	NVIC_Init(&NVIC_InitStructure);
}

/**
	*	@brief		����ʱ�ӳ�ʼ��
	*	@param		��
	*	@retval		��
	*/
void clock_init(void)
{
	// ʹ��GPIOA��GPIOB��AFIO����ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);

	// ʹ��CAN1����ʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

	// ����JTAG
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
}

/**
	* @brief   ��ʼ��CAN
	* @param   ��
	* @retval  ��
	*/
void can_init(void)
{
/**********************************************************
***	��ʼ��CAN���ţ��ο����ӣ�http://news.eeworld.com.cn/mcu/ic621750.html��
**********************************************************/
	// PB9 - CAN_TX
	GPIO_InitTypeDef  GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;				/* ����������� */
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	// PB8 - CAN_RX
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;					/* �������� */
	GPIO_Init(GPIOB, &GPIO_InitStructure);

/**********************************************************
***	��ӳ��CAN����
**********************************************************/
	GPIO_PinRemapConfig(GPIO_Remap1_CAN1, ENABLE);

/**********************************************************
***	��ʼ��CAN
**********************************************************/
	// ��ʼ��CAN����
	CAN_InitTypeDef	CAN_InitStructure;
	CAN_StructInit(&CAN_InitStructure);
	CAN_InitStructure.CAN_TTCM = DISABLE;					// �ر�ʱ�䴥��ͨѶģʽ
	CAN_InitStructure.CAN_ABOM = DISABLE;					// �����Զ����߹���
	CAN_InitStructure.CAN_AWUM = DISABLE;					// �ر��Զ�����ģʽ
	CAN_InitStructure.CAN_NART = DISABLE;					// �رշ��Զ��ش�ģʽ	DISABLE-�Զ��ش�
	CAN_InitStructure.CAN_RFLM = DISABLE;					// ����FIFO����ģʽ		DISABLE-���ʱ�±��ĻḲ��ԭ�б���
	CAN_InitStructure.CAN_TXFP = DISABLE;					// ����FIFO���ȼ�			DISABLE-���ȼ�ȡ���ڱ��ı�ʶ��
	CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;	// ��������ģʽ
	CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;			// ����ͬ����Ծ���ΪSJW + 1��ʱ�䵥λ
	CAN_InitStructure.CAN_BS1 = CAN_BS1_9tq;
	CAN_InitStructure.CAN_BS2 = CAN_BS2_2tq;
	CAN_InitStructure.CAN_Prescaler = 6;					// ������ = 36M / 6 / (1 + 9 + 2) = 0.5����500K
	CAN_Init(CAN1, &CAN_InitStructure);
	// ����STM32��֡ID - ��չ֡��ʽ - �������κ�����֡
	__IO uint8_t id_o, im_o; __IO uint16_t id_l, id_h, im_l, im_h;
	id_o = (0x00);
	id_h = (uint16_t)((uint16_t)id_o >> 5);										 // ��3λ
	id_l = (uint16_t)((uint16_t)id_o << 11) | CAN_Id_Extended; // ��5λ
	im_o = (0x00);
	im_h = (uint16_t)((uint16_t)im_o >> 5);
	im_l = (uint16_t)((uint16_t)im_o << 11) | CAN_Id_Extended;
	// ��ʼ��CAN������
	CAN_FilterInitTypeDef	CAN_FilterInitStructure;
	CAN_FilterInitStructure.CAN_FilterNumber = 1;											// ������1
	CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;		// ����ģʽ
	CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;	// 32λ������λ��
	CAN_FilterInitStructure.CAN_FilterIdHigh = id_h;									// ��������ʶ���ĸ�16λֵ
	CAN_FilterInitStructure.CAN_FilterIdLow = id_l;										// ��������ʶ���ĵ�16λֵ
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh = im_h;							// ���������α�ʶ���ĸ�16λֵ
	CAN_FilterInitStructure.CAN_FilterMaskIdLow = im_l;								// ���������α�ʶ���ĵ�16λֵ
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_FIFO0;			// ָ���������FIFOΪ0
	CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;						// ʹ�ܹ�����
	CAN_FilterInit(&CAN_FilterInitStructure);
	// ��ʼ��CAN�ж�
	CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);													// ʹ��RX0һ�����ݽ����ж�
}

/**
	*	@brief		���س�ʼ��
	*	@param		��
	*	@retval		��
	*/
void board_init(void)
{
	nvic_init();
	clock_init();
	can_init();
}
