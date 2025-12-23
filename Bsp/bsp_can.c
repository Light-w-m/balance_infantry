#include "bsp_can.h"
#include "memory.h"
#include "stdlib.h"
#include "bsp_dwt.h"
#include "chassisR_task.h"

// extern chassis_t chassis_move;

FDCAN_RxHeaderTypeDef RxHeader1;
uint8_t g_Can1RxData[64];

static CANInstance *can_instance[CAN_MX_REGISTER_CNT] = {NULL};
static uint8_t idx; // 全局CAN实例索引,每次有新的模块注册会自增

static void CANAddFilter(CANInstance *_instance)
{
	
#ifdef FDCAN
	static uint8_t can1_filter_idx = 0, can2_filter_idx = 0;
	//检查是否超出过滤器设定数量上限
	if(can1_filter_idx >= hfdcan1.Init.StdFiltersNbr || can2_filter_idx >= hfdcan2.Init.StdFiltersNbr)
	{
		while(1)
		{
			//报错
		}
	}
	uint8_t *filter_idx_p;

	if(_instance->can_handle==&hfdcan1)
	{
		filter_idx_p=&can1_filter_idx;
	}
	else if(_instance->can_handle==&hfdcan2)
	{
		filter_idx_p=&can2_filter_idx;
	}
	else
	{
		while(1)
		{
			//报错
		}
	}

	FDCAN_FilterTypeDef fdcan_filter_conf;
	fdcan_filter_conf.FilterIndex=(*filter_idx_p)++;
	//使用单个ID模式
	// fdcan_filter_conf.FilterType=FDCAN_FILTER_DUAL;
	fdcan_filter_conf.FilterType=FDCAN_FILTER_MASK;
	fdcan_filter_conf.FilterConfig=(_instance->tx_id & 1) ? FDCAN_FILTER_TO_RXFIFO0 : FDCAN_FILTER_TO_RXFIFO1;//奇数id的模块会被分配到FIFO0,偶数id的模块会被分配到FIFO1
	fdcan_filter_conf.FilterID1=_instance->rx_id;
	// fdcan_filter_conf.FilterID2=_instance->rx_id;
	fdcan_filter_conf.FilterID2=0x7FF;
	fdcan_filter_conf.IdType=FDCAN_STANDARD_ID;
	fdcan_filter_conf.IsCalibrationMsg=0;
	//fdcan_filter_conf.RxBufferIndex=0;

	HAL_FDCAN_ConfigFilter(_instance->can_handle, &fdcan_filter_conf);

#endif
}

void CANServiceInit()
{
#ifdef FDCAN
	//可能不需要这么多中断
	uint32_t FDCAN_RXActiveITs = FDCAN_IT_RX_FIFO0_NEW_MESSAGE|FDCAN_IT_RX_FIFO0_FULL\
			|FDCAN_IT_RX_FIFO0_WATERMARK|FDCAN_IT_RX_FIFO0_MESSAGE_LOST \
			|FDCAN_IT_RX_FIFO1_NEW_MESSAGE| FDCAN_IT_RX_FIFO1_FULL\
			|FDCAN_IT_RX_FIFO1_WATERMARK|FDCAN_IT_RX_FIFO1_MESSAGE_LOST;


	//HAL_FDCAN_ConfigClockCalibration()
	HAL_FDCAN_ConfigRxFifoOverwrite(&hfdcan1,FDCAN_RX_FIFO0,FDCAN_RX_FIFO_OVERWRITE);
	HAL_FDCAN_ConfigRxFifoOverwrite(&hfdcan1,FDCAN_RX_FIFO1,FDCAN_RX_FIFO_OVERWRITE);
	HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);//全局过滤器设置
	HAL_FDCAN_Start(&hfdcan1);
	HAL_FDCAN_ActivateNotification(&hfdcan1,FDCAN_RXActiveITs, 0);

	HAL_FDCAN_ConfigRxFifoOverwrite(&hfdcan2,FDCAN_RX_FIFO0,FDCAN_RX_FIFO_OVERWRITE);
	HAL_FDCAN_ConfigRxFifoOverwrite(&hfdcan2,FDCAN_RX_FIFO1,FDCAN_RX_FIFO_OVERWRITE);
	HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
	HAL_FDCAN_Start(&hfdcan2);
	HAL_FDCAN_ActivateNotification(&hfdcan2,FDCAN_RXActiveITs, 0);

#endif

}

CANInstance *CANRegister(CAN_Init_Config_s *config)
{
    if (!idx)
    {
        CANServiceInit(); // 第一次注册,先进行硬件初始化
        // LOGINFO("[bsp_can] CAN Service Init");
    }
    if (idx >= CAN_MX_REGISTER_CNT) // 超过最大实例数
    {
        while (1)
        {
        	// LOGERROR("[bsp_can] CAN instance exceeded MAX num, consider balance the load of CAN bus");
        }

    }
    for (size_t i = 0; i < idx; i++)
    { // 重复注册 | id重复
        if (can_instance[i]->rx_id == config->rx_id && can_instance[i]->can_handle == config->can_handle)
        {
            while (1)
            {
            	// LOGERROR("[}bsp_can] CAN id crash ,tx [%d] or rx [%d] already registered", &config->tx_id, &config->rx_id);
            }

        }
    }

    CANInstance *instance = (CANInstance *)malloc(sizeof(CANInstance)); // 分配空间
    memset(instance, 0, sizeof(CANInstance));                           // 分配的空间未必是0,所以要先清空
    // 进行发送报文的配置
#ifdef FDCAN
    instance->txconf.Identifier = config->tx_id; 				// 发送id
    instance->txconf.IdType = FDCAN_STANDARD_ID;  				// 使用标准id,扩展id则使用CAN_ID_EXT(目前没有需求)
    instance->txconf.TxFrameType = FDCAN_DATA_FRAME,    		// 发送数据帧
    instance->txconf.DataLength = FDCAN_DLC_BYTES_8,    		// 数据长度为8字节
	instance->txconf.ErrorStateIndicator = FDCAN_ESI_ACTIVE,	// 兼容CAN2.0,错误状态指示器设为主动
	instance->txconf.BitRateSwitch = FDCAN_BRS_OFF,         	// 兼容CAN2.0禁用位速率切换
	instance->txconf.FDFormat = FDCAN_CLASSIC_CAN,          	// 使用经典CAN格式
	instance->txconf.TxEventFifoControl = FDCAN_NO_TX_EVENTS,	// 不需要，禁用事件FIFO
	instance->txconf.MessageMarker = 0;                     	// 不使用消息标记
#endif
    // 设置回调函数和接收发送id
    instance->can_handle = config->can_handle;
    instance->tx_id = config->tx_id; // 好像没用,可以删掉
    instance->rx_id = config->rx_id;
    instance->can_module_callback = config->can_module_callback;
    instance->id = config->id;

    CANAddFilter(instance);         // 添加CAN过滤器规则
    can_instance[idx++] = instance; // 将实例保存到can_instance中

    return instance; // 返回can实例指针
}

/* @todo 目前似乎封装过度,应该添加一个指向tx_buff的指针,tx_buff不应该由CAN instance保存 */
/* 如果让CANinstance保存txbuff,会增加一次复制的开销 */
uint8_t CANTransmit(CANInstance *_instance, float timeout)
{
    static uint32_t busy_count;
    static volatile float wait_time __attribute__((unused)); // for cancel warning
    float dwt_start = DWT_GetTimeline_ms();
#ifdef FDCAN
    while(HAL_FDCAN_GetTxFifoFreeLevel(_instance->can_handle)==0)
#endif
    {
        if (DWT_GetTimeline_ms() - dwt_start > timeout) // 超时
        {
            // LOGWARNING("[bsp_can] CAN MAILbox full! failed to add msg to mailbox. Cnt [%d]", busy_count);
            busy_count++;
            return 0;
        }
    }
    wait_time = DWT_GetTimeline_ms() - dwt_start;

#ifdef FDCAN
    if (HAL_FDCAN_AddMessageToTxFifoQ(_instance->can_handle, &_instance->txconf, _instance->tx_buff))
#endif
    {
        // LOGWARNING("[bsp_can] CAN bus BUSY! cnt:%d", busy_count);
        busy_count++;
        return 0;
    }
    return 1; // 发送成功
}

void CANSetDLC(CANInstance *_instance, uint8_t length)
{
    // 发送长度错误!检查调用参数是否出错,或出现野指针/越界访问
    if (length > 8 || length == 0) // 安全检查
        while (1)
        {
        	// LOGERROR("[bsp_can] CAN DLC error! check your code or wild pointer");
        }

    _instance->txconf.DataLength = length;
}

uint8_t canx_send_data(hcan_t* hcan, uint16_t id, uint8_t *data, uint32_t len)
{
	FDCAN_TxHeaderTypeDef TxHeader;

	TxHeader.Identifier = id;                 			// CAN ID
	TxHeader.IdType =  FDCAN_STANDARD_ID ;        
	TxHeader.TxFrameType = FDCAN_DATA_FRAME;
	TxHeader.DataLength = FDCAN_DLC_BYTES_8;     		// 发送长度：8byte				
	TxHeader.ErrorStateIndicator =  FDCAN_ESI_ACTIVE;
	TxHeader.BitRateSwitch = FDCAN_BRS_OFF;				//比特率切换关闭，不适用于经典CAN
	TxHeader.FDFormat =  FDCAN_CLASSIC_CAN;            	// CANFD
	TxHeader.TxEventFifoControl =  FDCAN_NO_TX_EVENTS;  
	TxHeader.MessageMarker = 0;//消息标记

	HAL_FDCAN_AddMessageToTxFifoQ(hcan, &TxHeader, data);
	return 0;
}

#ifdef FDCAN
/**
 * @brief 此函数会被下面两个函数调用,用于处理FIFO0和FIFO1溢出中断(说明收到了新的数据)
 *        所有的实例都会被遍历,找到can_handle和rx_id相等的实例时,调用该实例的回调函数
 *
 * @param _fdhcan
 * @param fifox passed to HAL_CAN_GetRxMessage() to get mesg from a specific fifo
 */
static void FDCANFIFOxCallback(FDCAN_HandleTypeDef *_hfdcan, uint32_t fifox)
{
    static FDCAN_RxHeaderTypeDef rxconf; // 同上
	static uint16_t DataLength = 0;
    static uint8_t fdcan_rx_buff[8];
    while (HAL_FDCAN_GetRxFifoFillLevel(_hfdcan, fifox)) // FIFO不为空,有可能在其他中断时有多帧数据进入
    {
        HAL_FDCAN_GetRxMessage(_hfdcan, fifox, &rxconf, fdcan_rx_buff); // 从FIFO中获取数据
		//解析数据长度，@Todo 此处在用新版本重新生成后可能得修改，DataLength可能不需要右移，具体情况具体看	！
		if(((rxconf.DataLength >> 16) & 0xF)>=0 && ((rxconf.DataLength >> 16) & 0xF)<=8)
		{
			DataLength=(rxconf.DataLength >> 16) & 0xF; // 保存接收到的数据长度
		}
		else
		{
			DataLength=0;
		}
        if(rxconf.RxFrameType==FDCAN_DATA_FRAME && rxconf.IdType==FDCAN_STANDARD_ID)
        {
        	for (size_t i = 0; i < idx; ++i)
			{
        		// 两者相等说明这是要找的实例
				if (_hfdcan == can_instance[i]->can_handle && rxconf.Identifier == can_instance[i]->rx_id)
				{
					if (can_instance[i]->can_module_callback != NULL) // 回调函数不为空就调用
					{
						can_instance[i]->rx_len = DataLength;               // 保存接收到的数据长度
						memcpy(can_instance[i]->rx_buff, fdcan_rx_buff, can_instance[i]->rx_len); // 消息拷贝到对应实例
						can_instance[i]->can_module_callback(can_instance[i]);     // 触发回调进行数据解析和处理
					}
					return;
				}
			}
        }
    }
}

#endif

/*********************************回调********************* */

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	/* 检查Rx FIFO 0中是否有消息丢失 */
	if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_MESSAGE_LOST) != 0)
	{
		//报错
	}
	/* 检查是否有新消息写入Rx FIFO 0或到达一定阈值 */
	if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE)||(RxFifo0ITs & FDCAN_IT_RX_FIFO0_FULL)||(RxFifo0ITs & FDCAN_IT_RX_FIFO0_WATERMARK))
	{
		FDCANFIFOxCallback(hfdcan, FDCAN_RX_FIFO0); // 调用我们自己写的函数来处理消息
	}
}

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
	/* 检查Rx FIFO 1中是否有消息丢失 */
	if ((RxFifo1ITs & FDCAN_IT_RX_FIFO1_MESSAGE_LOST) != 0)
	{
		//报错
	}
	/* 检查是否有新消息写入Rx FIFO 1或到达一定阈值 */
	if ((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE)||(RxFifo1ITs & FDCAN_IT_RX_FIFO1_FULL)||(RxFifo1ITs & FDCAN_IT_RX_FIFO1_WATERMARK))
	{
		FDCANFIFOxCallback(hfdcan, FDCAN_RX_FIFO1); // 调用我们自己写的函数来处理消息
	}
}
