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




static const char *TAG = "receive handle";

QueueHandle_t slave_evt_queue = NULL;
static bool s_pairing_locked = false;
static uint8_t s_pairing_master_mac[6] = {0};
static bool s_powered_on = false;
static bool s_power_owner_valid = false;
static uint8_t s_power_owner_mac[6] = {0};

void slave_set_pairing_lock(const uint8_t *master_mac)
{
    if (!master_mac) {
        return;
    }
    memcpy(s_pairing_master_mac, master_mac, 6);
    s_pairing_locked = true;
}

void slave_clear_pairing_lock(void)
{
    memset(s_pairing_master_mac, 0, sizeof(s_pairing_master_mac));
    s_pairing_locked = false;
}

void slave_set_powered_on(bool powered_on)
{
    s_powered_on = powered_on;
}

bool slave_is_powered_on(void)
{
    return s_powered_on;
}

void slave_set_power_owner(const uint8_t *master_mac)
{
    if (!master_mac) {
        return;
    }
    memcpy(s_power_owner_mac, master_mac, sizeof(s_power_owner_mac));
    s_power_owner_valid = true;
}

void slave_clear_power_owner(void)
{
    memset(s_power_owner_mac, 0, sizeof(s_power_owner_mac));
    s_power_owner_valid = false;
}

bool slave_is_power_owner(const uint8_t *src_addr)
{
    if (!src_addr || !s_power_owner_valid) {
        return false;
    }
    return memcmp(src_addr, s_power_owner_mac, sizeof(s_power_owner_mac)) == 0;
}



esp_err_t slave_receive_handle(uint8_t *src_addr,
                                       void *data,
                                       size_t size,
                                       wifi_pkt_rx_ctrl_t *rx_ctrl)
{
  static uint32_t count = 0;
    if (size < sizeof(esp_now_data)) {
        ESP_LOGW(TAG, "Packet too short: %u", (unsigned)size);
        return ESP_ERR_INVALID_SIZE;
    }

  const esp_now_data *pkt = (const esp_now_data *)data;

    if (!slave_is_powered_on() && pkt->type != POWER_MANAGE) {
            ESP_LOGD(TAG, "Drop packet type=%d while powered off", pkt->type);
            return ESP_OK;
    }

  switch(pkt->type){

    //接受配对请求，发送配对确认
    case CONNECTION_REQUEST:{
            if (s_pairing_locked) {
                ESP_LOGD(TAG, "Ignore CONNECTION_REQUEST while pairing locked");
                break;
            }

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
      slave_evt_msg_t msg{};
      msg.event = EVT_RECEIVE_REQ;
      msg.data = pkt->data;
      memcpy(msg.master_mac, src_addr, 6);
      xQueueSend(slave_evt_queue, &msg, 0);

    }break;


    //接收主设备确认，使从设备进入Ready状态，保存主设备mac地址
    case CONNECTION_MASTER_CONFIRM:{
            if (s_pairing_locked && memcmp(src_addr, s_pairing_master_mac, 6) != 0) {
                ESP_LOGW(TAG, "Ignore master ACK from unknown peer");
                break;
            }

      ESP_LOGI(TAG,"Recevice Master ACK");
      slave_evt_msg_t msg{};
      msg.event = EVT_RECEIVE_MASTER_ACK;
      msg.data = pkt->data;
      memcpy(msg.master_mac, src_addr, 6);
      xQueueSend(slave_evt_queue, &msg, 0);

    }break;


    //接收主设备状态切换控制
    case STATUS_CHANGE:{

      ESP_LOGI(TAG,"Recevice Status Change Signal");
      slave_evt_msg_t msg{} ;
      //非0则启动，不然则停止
      if(pkt->data){msg.event = EVT_SLAVE_START_WORK;}
      else{ msg.event = EVT_SLAVE_STOP_WORK;ESP_LOGI(TAG,"Start Failed");}
      msg.data = pkt->data;
      
      xQueueSend(slave_evt_queue, &msg, 0);
      

    }break;

        // 接收主设备心跳回复，data=1
        case KEEP_ALIVE: {
            if (pkt->data == 1) {
                slave_evt_msg_t msg{};
                msg.event = EVT_HEARTBEAT_ACK;
                msg.data = pkt->data;
                memcpy(msg.master_mac, src_addr, 6);
                xQueueSend(slave_evt_queue, &msg, 0);
            }
        } break;

        case POWER_MANAGE: {
            if (pkt->data == 1) {
                if (slave_is_powered_on()) {
                    ESP_LOGI(TAG, "Ignore POWER_ON request while already powered on");
                    break;
                }

                slave_evt_msg_t msg{};
                msg.event = EVT_POWER_ON_REQ;
                msg.data = pkt->data;
                memcpy(msg.master_mac, src_addr, 6);
                xQueueSend(slave_evt_queue, &msg, 0);
            } else if (pkt->data == 0) {
                if (!slave_is_powered_on()) {
                    ESP_LOGI(TAG, "Ignore POWER_OFF request while already powered off");
                    break;
                }
                if (!slave_is_power_owner(src_addr)) {
                    ESP_LOGW(TAG, "Ignore POWER_OFF request from non-owner");
                    break;
                }

                slave_evt_msg_t msg{};
                msg.event = EVT_POWER_OFF_REQ;
                msg.data = pkt->data;
                memcpy(msg.master_mac, src_addr, 6);
                xQueueSend(slave_evt_queue, &msg, 0);
            } else {
                ESP_LOGW(TAG, "Unknown POWER_MANAGE data=%" PRIu32, pkt->data);
            }
        } break;




    default:{
      ESP_LOGW(TAG,
             "Unknown packet type: %d, len=%u from %02X:%02X:%02X:%02X:%02X:%02X",
             pkt->type,
             (unsigned)size,
             src_addr[0], src_addr[1], src_addr[2],
             src_addr[3], src_addr[4], src_addr[5]);
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
            } else if (event == EVT_WAIT_MASTER_ACK_TIMEOUT) {
                next_state = SLAVE_IDLE;
                ESP_LOGW(TAG, "SLAVE: wait master confirm timeout -> SLAVE_IDLE");
            }
            break;

        case SLAVE_READY:
            if (event == EVT_SLAVE_START_WORK) {
                next_state = SLAVE_RUNNING;
                ESP_LOGI(TAG, "SLAVE: start work -> SLAVE_RUNNING");
            } else if (event == EVT_MASTER_LOST) {
                next_state = SLAVE_IDLE;
                ESP_LOGW(TAG, "SLAVE: master lost -> SLAVE_IDLE");
            }
            break;

        case SLAVE_RUNNING:
            if (event == EVT_SLAVE_STOP_WORK) {
                next_state = SLAVE_READY;
                ESP_LOGI(TAG, "SLAVE: stop work -> SLAVE_READY");
            } else if (event == EVT_MASTER_LOST) {
                next_state = SLAVE_IDLE;
                ESP_LOGW(TAG, "SLAVE: master lost -> SLAVE_IDLE");
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