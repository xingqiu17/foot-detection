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
#include "freertos/event_groups.h"




static const char *TAG = "esp_now";

QueueHandle_t slave_evt_queue = NULL;



esp_err_t slave_receive_handle(uint8_t *src_addr,
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
                  // 非阻塞投递
      slave_evt_msg_t msg = {
          .event = EVT_RECEIVE_REQ,
      };
      memcpy(msg.master_mac, src_addr, 6);
      xQueueSend(slave_evt_queue, &msg, 0);

    }break;






    default:{

    }break;

  }


  

  return ESP_OK;
}






slave_state_t slave_state_machine(slave_state_t cur_state, slave_event_t event)
{
    slave_state_t next_state = cur_state;

    switch (cur_state) {
        case SLAVE_IDLE:
            if (event == EVT_RECEIVE_REQ) {
                next_state = SLAVE_WAIT_MAIN_CONFIRM;
                ESP_LOGI(TAG, "SLAVE: received master request -> SLAVE_WAIT_MAIN_CONFIRM");
            }
            break;

        case SLAVE_WAIT_MAIN_CONFIRM:
            if (event == EVT_RECEIVE_MASTER_ACK) {
                next_state = SLAVE_READY;
                ESP_LOGI(TAG, "SLAVE: received master final confirm -> SLAVE_READY");
            }
            break;

        case SLAVE_READY:
            if (event == EVT_SLAVE_START_WORK) {
                next_state = SLAVE_RUNNING;
                ESP_LOGI(TAG, "SLAVE: start work -> SLAVE_RUNNING");
            }
            break;

        case SLAVE_RUNNING:
            if (event == EVT_SLAVE_STOP_WORK) {
                next_state = SLAVE_IDLE;
                ESP_LOGI(TAG, "SLAVE: stop work -> SLAVE_IDLE");
            }
            break;

        default:
            if (event == EVT_SLAVE_ERROR) {
                next_state = SLAVE_IDLE;
                ESP_LOGE(TAG, "SLAVE: error occurred -> SLAVE_IDLE");
            }
            break;
    }

    return next_state;
}