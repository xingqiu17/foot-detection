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



#ifdef __cplusplus
extern "C" {
#endif


extern uint32_t seq ;

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


typedef struct{
    msg_type type;
    uint32_t seq;
    uint32_t data;

}esp_now_data;



/*----------------------MAC地址----------------------*/
typedef struct {
    uint8_t addr[6];
} mac_addr_t;





esp_err_t app_uart_write_handle(uint8_t *src_addr, void *data,
                                       size_t size, wifi_pkt_rx_ctrl_t *rx_ctrl);







#ifdef __cplusplus
}
#endif
