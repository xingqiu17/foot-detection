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
#include <string.h>




static const char *TAG = "receive handle";

QueueHandle_t slave_evt_queue = NULL;
static bool s_pairing_locked = false;
static uint8_t s_pairing_master_mac[6] = {0};
static bool s_powered_on = false;
static bool s_powering_on = false;
static bool s_power_owner_valid = false;
static uint8_t s_power_owner_mac[6] = {0};
static QueueHandle_t s_test_reply_queue = NULL;
static TaskHandle_t s_test_reply_task_handle = NULL;
static bool s_test_reply_enabled = (SLAVE_TEST_REPLY_DEFAULT_ENABLE != 0);

typedef struct {
    uint8_t dst_mac[6];
    uint32_t request_seq;
} slave_test_reply_msg_t;

static void slave_test_reply_task(void *arg)
{
    slave_test_reply_msg_t msg;

    while (true) {
        if (xQueueReceive(s_test_reply_queue, &msg, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        if (!s_test_reply_enabled) {
            continue;
        }

        espnow_add_peer(msg.dst_mac, NULL);

        espnow_frame_head_t frame_head{};
        frame_head.retransmit_count = 3;
        frame_head.broadcast = false;

        esp_now_data test_reply = {
            .type = TEST,
            .seq = seq++,
            .data = 1,
        };

        esp_err_t err = espnow_send(ESPNOW_DATA_TYPE_DATA,
            msg.dst_mac,
            &test_reply,
            sizeof(test_reply),
            &frame_head,
            pdMS_TO_TICKS(100));

        if (err != ESP_OK) {
            ESP_LOGW(TAG, "TEST reply failed: %s", esp_err_to_name(err));
        } else {
            ESP_LOGD(TAG, "TEST reply sent for request seq=%" PRIu32, msg.request_seq);
        }
    }
}

esp_err_t slave_test_reply_task_start(void)
{
    if (s_test_reply_task_handle) {
        return ESP_OK;
    }

    if (!s_test_reply_queue) {
        s_test_reply_queue = xQueueCreate(8, sizeof(slave_test_reply_msg_t));
        if (!s_test_reply_queue) {
            ESP_LOGE(TAG, "test reply queue create failed");
            return ESP_ERR_NO_MEM;
        }
    }

    BaseType_t ret = xTaskCreate(slave_test_reply_task,
        "test_reply",
        3072,
        NULL,
        4,
        &s_test_reply_task_handle);

    if (ret != pdPASS) {
        ESP_LOGE(TAG, "test reply task create failed");
        vQueueDelete(s_test_reply_queue);
        s_test_reply_queue = NULL;
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "TEST reply task started, enabled=%d", s_test_reply_enabled);
    return ESP_OK;
}

void slave_test_reply_task_stop(void)
{
    if (s_test_reply_task_handle) {
        vTaskDelete(s_test_reply_task_handle);
        s_test_reply_task_handle = NULL;
    }

    if (s_test_reply_queue) {
        vQueueDelete(s_test_reply_queue);
        s_test_reply_queue = NULL;
    }
}

void slave_test_reply_set_enabled(bool enabled)
{
    s_test_reply_enabled = enabled;
}

bool slave_test_reply_is_enabled(void)
{
    return s_test_reply_enabled;
}

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

void slave_set_powering_on(bool powering_on)
{
    s_powering_on = powering_on;
}

bool slave_is_powering_on(void)
{
    return s_powering_on;
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

    if (!slave_is_powered_on()
        && pkt->type != POWER_MANAGE
        && !(pkt->type == TEST && slave_test_reply_is_enabled())) {
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

                if (slave_is_powering_on()) {
                    ESP_LOGI(TAG, "Ignore POWER_ON request while initializing");
                    break;
                }

                slave_set_powering_on(true);

                slave_evt_msg_t msg{};
                msg.event = EVT_POWER_ON_REQ;
                msg.data = pkt->data;
                memcpy(msg.master_mac, src_addr, 6);
                if (xQueueSend(slave_evt_queue, &msg, 0) != pdTRUE) {
                    slave_set_powering_on(false);
                    ESP_LOGW(TAG, "Drop POWER_ON request because event queue is full");
                }
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

        case TEST: {
            if (!slave_test_reply_is_enabled()) {
                ESP_LOGD(TAG, "Drop TEST packet because test reply is disabled");
                break;
            }

            if (pkt->data != 0) {
                ESP_LOGD(TAG, "Ignore TEST packet with data=%" PRIu32, pkt->data);
                break;
            }

            if (!s_test_reply_queue) {
                ESP_LOGW(TAG, "Drop TEST packet because reply task is not started");
                break;
            }

            slave_test_reply_msg_t msg{};
            memcpy(msg.dst_mac, src_addr, sizeof(msg.dst_mac));
            msg.request_seq = pkt->seq;

            if (xQueueSend(s_test_reply_queue, &msg, 0) != pdTRUE) {
                ESP_LOGW(TAG, "Drop TEST packet because reply queue is full");
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
