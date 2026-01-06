#pragma once

#include <stdint.h>
#include "esp_err.h"

#include "esp_wifi.h"
#include "esp_mac.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "espnow.h"
#include "espnow_storage.h"
#include "espnow_utils.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"



#ifdef __cplusplus
extern "C" {
#endif



extern uint32_t seq ;

extern QueueHandle_t slave_evt_queue;    //从设备事件句柄

/*----------------------传输信息类型----------------------*/
typedef enum {
    CONNECTION_REQUEST,          //主设备发送的连接请求
    CONNECTION_SLAVE_CONFIRM,      //从设备接受主设备连接请求后，发回给主设备的连接确认
    CONNECTION_MASTER_CONFIRM,   //主设备接受从设备连接确认后，发回给从设备的连接确认
    STATUS_CHANGE,               //主设备->从设备：状态切换
    STATUS_CONFIRM,              //从设备->主设备：状态确认
    EXERCISE_DATA,               //锻炼数据

    TEST,                        //测试

} msg_type;


typedef struct {
    msg_type type;
    uint32_t seq;
    uint32_t  data;
} esp_now_data;



/*----------------------从设备连接状态机----------------------*/
typedef enum {
    SLAVE_IDLE,
    SLAVE_WAIT_MAIN_CONFIRM,
    SLAVE_READY,
    SLAVE_RUNNING,
} slave_state_t;


/*----------------------状态机事件----------------------*/
typedef enum {
    EVT_PAIR_START,          // 开始配对
    EVT_SLAVE_RESP,          // 收到从设备确认
    EVT_PAIR_TIMEOUT,        // 配对超时
    EVT_MASTER_RESP,         // 所有从设备确认完成，主设备发送最终确认
    EVT_START_WORK,          // 启动工作
    EVT_STOP_WORK,           // 停止工作
    EVT_ERROR,               // 错误
} master_event_t;


typedef enum {
    EVT_RECEIVE_REQ,         // 从设备收到主设备请求
    EVT_RECEIVE_MASTER_ACK,  // 从设备收到主设备最终确认
    EVT_SLAVE_START_WORK,          // 启动工作
    EVT_SLAVE_STOP_WORK,           // 停止工作
    EVT_SLAVE_ERROR,         // 从设备出错
} slave_event_t;


/*----------------------MAC地址----------------------*/
typedef struct {
    uint8_t addr[6];
} mac_addr_t;


typedef struct {
    slave_event_t event;
    uint8_t master_mac[6];
} slave_evt_msg_t;



esp_err_t slave_receive_handle(uint8_t *src_addr, void *data,
                                       size_t size, wifi_pkt_rx_ctrl_t *rx_ctrl);


slave_state_t  slave_state_machine(slave_state_t cur_state, slave_event_t event);




#ifdef __cplusplus
}
#endif
