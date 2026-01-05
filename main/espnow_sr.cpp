#include "espnow_sr.h"
#include "esp_wifi.h"
#include "espnow.h"
#include "espnow_utils.h"
#include "esp_mac.h"
#include "esp_system.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"


static const char *TAG = "esp_now";

esp_err_t app_uart_write_handle(uint8_t *src_addr,
                                       void *data,
                                       size_t size,
                                       wifi_pkt_rx_ctrl_t *rx_ctrl)
{
  static uint32_t count = 0;
  const esp_now_data *pkt = (const esp_now_data *)data;

  switch(pkt->type){
    case CONNECTION_REQUEST:{
      ESP_LOGI(TAG,"Recevice request");
      ESP_LOGI(TAG,
             "recv<%" PRIu32 "> src=%02X:%02X:%02X:%02X:%02X:%02X ch=%d rssi=%d len=%u",
             count++,
             src_addr[0], src_addr[1], src_addr[2],
             src_addr[3], src_addr[4], src_addr[5],
             rx_ctrl->channel,
             rx_ctrl->rssi,
             (unsigned)size
            );
    }break;



    default:{

    }break;

  }


  

  return ESP_OK;
}