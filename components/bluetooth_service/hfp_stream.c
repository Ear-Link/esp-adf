/*
 * ESPRESSIF MIT License
 *
 * Copyright (c) 2019 <ESPRESSIF SYSTEMS (SHANGHAI) CO., LTD>
 *
 * Permission is hereby granted for use on all ESPRESSIF SYSTEMS products, in which case,
 * it is free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software is furnished
 * to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */

#include <stdio.h>
#include <inttypes.h>
#include <string.h>
#include "esp_log.h"
#include "raw_stream.h"
#include "audio_element.h"
#include "audio_mem.h"
#include "sdkconfig.h"
#include "hfp_stream.h"

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#if (defined CONFIG_CLASSIC_BT_ENABLED)
static const char *TAG = "HFP_STREAM";

#define ESP_HFP_RINGBUF_SIZE     3600
#define ESP_HFP_TASK_SIZE        2048
#define ESP_HFP_TASK_PRIORITY    23
static bool is_get_data = true;
static hfp_stream_user_callback_t  hfp_stream_user_callback;
static audio_element_handle_t hfp_incoming_stream = NULL;
static audio_element_handle_t hfp_outgoing_stream = NULL;

#ifdef CONFIG_DEBUG_AUDIO
static uint32_t hfp_client_audio_packet_count_outgoing = 0;
static uint32_t hfp_client_audio_packet_count_incoming = 0;
static uint32_t hfp_ag_audio_packet_count_incoming = 0;
static uint32_t hfp_ag_audio_packet_count_outgoing = 0;
#endif

const char *c_hf_evt_str[] = {
    "CONNECTION_STATE_EVT",              /*!< connection state changed event */
    "AUDIO_STATE_EVT",                   /*!< audio connection state change event */
    "VR_STATE_CHANGE_EVT",                /*!< voice recognition state changed */
    "CALL_IND_EVT",                      /*!< call indication event */
    "CALL_SETUP_IND_EVT",                /*!< call setup indication event */
    "CALL_HELD_IND_EVT",                 /*!< call held indicator event */
    "NETWORK_STATE_EVT",                 /*!< network state change event */
    "SIGNAL_STRENGTH_IND_EVT",           /*!< signal strength indication event */
    "ROAMING_STATUS_IND_EVT",            /*!< roaming status indication event */
    "BATTERY_LEVEL_IND_EVT",             /*!< battery level indication event */
    "CURRENT_OPERATOR_EVT",              /*!< current operator name event */
    "RESP_AND_HOLD_EVT",                 /*!< response and hold event */
    "CLIP_EVT",                          /*!< Calling Line Identification notification event */
    "CALL_WAITING_EVT",                  /*!< call waiting notification */
    "CLCC_EVT",                          /*!< listing current calls event */
    "VOLUME_CONTROL_EVT",                /*!< audio volume control event */
    "AT_RESPONSE",                       /*!< audio volume control event */
    "SUBSCRIBER_INFO_EVT",               /*!< subscriber information event */
    "INBAND_RING_TONE_EVT",              /*!< in-band ring tone settings */
    "LAST_VOICE_TAG_NUMBER_EVT",         /*!< requested number from AG event */
    "RING_IND_EVT",                      /*!< ring indication event */
    "PKT_STAT_NUMS_GET_EVT",             /*!< requested number of packet different status */
    "PROF_STATE_EVT",                    /*!< Indicate HF CLIENT init or deinit complete */
};

// esp_hf_client_connection_state_t
const char *c_connection_state_str[] = {
    "disconnected",
    "connecting",
    "connected",
    "slc_connected",
    "disconnecting",
};

// esp_hf_client_audio_state_t
const char *c_audio_state_str[] = {
    "disconnected",
    "connecting",
    "connected",
    "connected_msbc",
};

/// esp_hf_vr_state_t
const char *c_vr_state_str[] = {
    "disabled",
    "enabled",
};

// esp_hf_service_availability_status_t
const char *c_service_availability_status_str[] = {
    "unavailable",
    "available",
};

// esp_hf_roaming_status_t
const char *c_roaming_status_str[] = {
    "inactive",
    "active",
};

// esp_hf_client_call_state_t
const char *c_call_str[] = {
    "NO call in progress",
    "call in progress",
};

// esp_hf_client_callsetup_t
const char *c_call_setup_str[] = {
    "NONE",
    "INCOMING",
    "OUTGOING_DIALING",
    "OUTGOING_ALERTING"
};

// esp_hf_client_callheld_t
const char *c_call_held_str[] = {
    "NONE held",
    "Held and Active",
    "Held",
};

// esp_hf_response_and_hold_status_t
const char *c_resp_and_hold_str[] = {
    "HELD",
    "HELD ACCEPTED",
    "HELD REJECTED",
};

// esp_hf_client_call_direction_t
const char *c_call_dir_str[] = {
    "outgoing",
    "incoming",
};

// esp_hf_client_call_state_t
const char *c_call_state_str[] = {
    "active",
    "held",
    "dialing",
    "alerting",
    "incoming",
    "waiting",
    "held_by_resp_hold",
};

// esp_hf_current_call_mpty_type_t
const char *c_call_mpty_type_str[] = {
    "single",
    "multi",
};

// esp_hf_volume_control_target_t
const char *c_volume_control_target_str[] = {
    "SPEAKER",
    "MICROPHONE"
};

// esp_hf_at_response_code_t
const char *c_at_response_code_str[] = {
    "OK",
    "ERROR"
    "ERR_NO_CARRIER",
    "ERR_BUSY",
    "ERR_NO_ANSWER",
    "ERR_DELAYED",
    "ERR_BLACKLILSTED",
    "ERR_CME",
};

// esp_hf_subscriber_service_type_t
const char *c_subscriber_service_type_str[] = {
    "unknown",
    "voice",
    "fax",
};

// esp_hf_client_in_band_ring_state_t
const char *c_inband_ring_state_str[] = {
    "NOT provided",
    "Provided",
};

esp_err_t hfp_open_and_close_evt_cb_register(esp_hf_audio_open_t open_cb, esp_hf_audio_close_t close_cb)
{
    if ((open_cb == NULL)||(close_cb == NULL)) {
        return ESP_FAIL;
    }
    hfp_stream_user_callback.user_hfp_open_cb = open_cb;
    hfp_stream_user_callback.user_hfp_close_cb = close_cb;
    return ESP_OK;
}

static uint32_t bt_app_hf_client_outgoing_cb(uint8_t *p_buf, uint32_t sz)
{
    int out_len_bytes = 0;
    if (is_get_data) {
        out_len_bytes = audio_element_input(hfp_outgoing_stream, (char *)p_buf, sz);
    }
#ifdef CONFIG_DEBUG_AUDIO
    if (hfp_client_audio_packet_count_outgoing++ % 100 == 0) {
        ESP_LOGI(TAG, "bt_app_hf_client_outgoing_cb: %lu, sz: %lu", hfp_client_audio_packet_count_outgoing, sz);
    }
#endif
    if (out_len_bytes == sz) {
        is_get_data = false;
        return sz;
    } else {
        is_get_data = true;
        return 0;
    }
}

static void bt_app_hf_client_incoming_cb(const uint8_t *buf, uint32_t sz)
{
    if (hfp_incoming_stream) {
        if (audio_element_get_state(hfp_incoming_stream) == AEL_STATE_RUNNING) {
            audio_element_output(hfp_incoming_stream, (char *)buf, sz);
            esp_hf_client_outgoing_data_ready();
        }
#ifdef CONFIG_DEBUG_AUDIO
        if (hfp_client_audio_packet_count_incoming++ % 100 == 0) {
            ESP_LOGI(TAG, "bt_app_hf_client_incoming_cb: %lu, sz: %lu", hfp_client_audio_packet_count_incoming, sz);
        }
#endif
    }
}

/* callback for HF_CLIENT */
void bt_hf_client_cb(uint16_t _event, void* _param)
{
    esp_hf_client_cb_event_t event = (esp_hf_client_cb_event_t) _event;
    esp_hf_client_cb_param_t *param = (esp_hf_client_cb_param_t *)(_param);

    if (event < ESP_HF_CLIENT_EVT_COUNT) {
        ESP_LOGI(TAG, "APP HFP event: %s", c_hf_evt_str[event]);
    } else {
        ESP_LOGE(TAG, "APP HFP invalid event %d", event);
    }

    switch (event) {
    case ESP_HF_CLIENT_CONNECTION_STATE_EVT:
        ESP_LOGI(TAG, "--Connection state %s, peer feats 0x%" PRIx32 ", chld_feats 0x%" PRIx32,
                 c_connection_state_str[param->conn_stat.state],
                 param->conn_stat.peer_feat,
                 param->conn_stat.chld_feat);
        break;
    case ESP_HF_CLIENT_AUDIO_STATE_EVT:
        ESP_LOGI(TAG, "--Audio state %s",
                 c_audio_state_str[param->audio_stat.state]);
        if ((param->audio_stat.state == ESP_HF_CLIENT_AUDIO_STATE_CONNECTED)
            || (param->audio_stat.state == ESP_HF_CLIENT_AUDIO_STATE_CONNECTED_MSBC)) {
            if(hfp_stream_user_callback.user_hfp_open_cb != NULL) {
                if (param->audio_stat.state == ESP_HF_CLIENT_AUDIO_STATE_CONNECTED) {
                    hfp_stream_user_callback.user_hfp_open_cb(HF_DATA_CVSD);
                } else {
                    hfp_stream_user_callback.user_hfp_open_cb(HF_DATA_MSBC);
                }
            }
            esp_hf_client_register_data_callback(bt_app_hf_client_incoming_cb,
                                                 bt_app_hf_client_outgoing_cb);
        } else if (param->audio_stat.state == ESP_HF_CLIENT_AUDIO_STATE_DISCONNECTED) {
            if (hfp_stream_user_callback.user_hfp_close_cb != NULL) {
                hfp_stream_user_callback.user_hfp_close_cb();
            }
        }
        break;
    case ESP_HF_CLIENT_BVRA_EVT:
        ESP_LOGI(TAG, "--VR state %s",
                 c_vr_state_str[param->bvra.value]);
        break;
    case ESP_HF_CLIENT_CIND_SERVICE_AVAILABILITY_EVT:
        ESP_LOGI(TAG, "--NETWORK state %s",
                 c_service_availability_status_str[param->service_availability.status]);
        break;
    case ESP_HF_CLIENT_CIND_ROAMING_STATUS_EVT:
        ESP_LOGI(TAG, "--ROAMING: %s",
                 c_roaming_status_str[param->roaming.status]);
        break;
    case ESP_HF_CLIENT_CIND_SIGNAL_STRENGTH_EVT:
        ESP_LOGI(TAG, "--Signal strength: %d",
                 param->signal_strength.value);
        break;
    case ESP_HF_CLIENT_CIND_BATTERY_LEVEL_EVT:
        ESP_LOGI(TAG, "--Battery level %d",
                 param->battery_level.value);
        break;
    case ESP_HF_CLIENT_COPS_CURRENT_OPERATOR_EVT:
        ESP_LOGI(TAG, "--Operator name: %s",
                 param->cops.name);
        break;
    case ESP_HF_CLIENT_CIND_CALL_EVT:
        ESP_LOGI(TAG, "--Call indicator %s",
                 c_call_str[param->call.status]);
        break;
    case ESP_HF_CLIENT_CIND_CALL_SETUP_EVT:
        ESP_LOGI(TAG, "--Call setup indicator %s",
                 c_call_setup_str[param->call_setup.status]);
        break;
    case ESP_HF_CLIENT_CIND_CALL_HELD_EVT:
        ESP_LOGI(TAG, "--Call held indicator %s",
                 c_call_held_str[param->call_held.status]);
        break;
    case ESP_HF_CLIENT_BTRH_EVT:
        ESP_LOGI(TAG, "--Response and hold %s",
                 c_resp_and_hold_str[param->btrh.status]);
        break;
    case ESP_HF_CLIENT_CLIP_EVT:
        ESP_LOGI(TAG, "--Clip number %s",
                 (param->clip.number == NULL) ? "NULL" : (param->clip.number));
        break;
    case ESP_HF_CLIENT_CCWA_EVT:
        ESP_LOGI(TAG, "--Call_waiting %s",
                 (param->ccwa.number == NULL) ? "NULL" : (param->ccwa.number));
        break;
    case ESP_HF_CLIENT_CLCC_EVT:
        ESP_LOGI(TAG, "--Current call: idx %d, dir %s, state %s, mpty %s, number %s",
                 param->clcc.idx,
                 c_call_dir_str[param->clcc.dir],
                 c_call_state_str[param->clcc.status],
                 c_call_mpty_type_str[param->clcc.mpty],
                 (param->clcc.number == NULL) ? "NULL" : (param->clcc.number));
        break;
    case ESP_HF_CLIENT_VOLUME_CONTROL_EVT:
        ESP_LOGI(TAG, "--Volume_target: %s, volume %d",
                 c_volume_control_target_str[param->volume_control.type],
                 param->volume_control.volume);
        break;
    case ESP_HF_CLIENT_AT_RESPONSE_EVT:
        ESP_LOGI(TAG, "--AT response event, code %d, cme %d",
                 param->at_response.code, param->at_response.cme);
        break;
    case ESP_HF_CLIENT_CNUM_EVT:
        ESP_LOGI(TAG, "--Subscriber type %s, number %s",
                 c_subscriber_service_type_str[param->cnum.type],
                 (param->cnum.number == NULL) ? "NULL" : param->cnum.number);
        break;
    case ESP_HF_CLIENT_BSIR_EVT:
        ESP_LOGI(TAG, "--Inband ring state %s",
                 c_inband_ring_state_str[param->bsir.state]);
        break;
    case ESP_HF_CLIENT_BINP_EVT:
        ESP_LOGI(TAG, "--Last voice tag number: %s",
                 (param->binp.number == NULL) ? "NULL" : param->binp.number);
        break;
    default:
        ESP_LOGI(TAG, "HF_CLIENT EVT: %d", event);
        break;
    }
}

// 7500 microseconds(=12 slots) is aligned to 1 msbc frame duration, and is
// multiple of common Tesco for eSCO link with EV3 or 2-EV3 packet type
#define BLOCK_DURATION_US (7500)

#define WBS_SAMPLING_RATE_KHZ (16)
#define SAMPLING_RATE_KHZ (8)

#define BYTES_PER_SAMPLE (2)

// input can refer to Enhanced Setup Synchronous Connection Command in core
// spec4.2 Vol2, Part E
#define WBS_INPUT_DATA_SIZE                                                    \
    (WBS_SAMPLING_RATE_KHZ * BLOCK_DURATION_US / 1000 * BYTES_PER_SAMPLE) // 240
#define INPUT_DATA_SIZE                                                        \
    (SAMPLING_RATE_KHZ * BLOCK_DURATION_US / 1000 * BYTES_PER_SAMPLE) // 120

// increased to get rid of "BT_BTM: SCO xmit Q overflow, pkt dropped"
#define GENERATOR_TICK_US (4000 * 2)

static esp_timer_handle_t s_periodic_timer;
static uint64_t s_last_enter_time, s_now_enter_time;
static uint64_t s_us_duration;
static SemaphoreHandle_t s_send_data_Semaphore = NULL;
static TaskHandle_t s_bt_hf_ag_start_audio_task_handler = NULL;
static esp_hf_audio_state_t s_audio_code;

static void bt_hf_ag_start_audio_timer_cb(void* arg) {
    if (!xSemaphoreGive(s_send_data_Semaphore)) {
        ESP_LOGE(TAG, "%s xSemaphoreGive failed", __func__);
        return;
    }
    return;
}

static void bt_hf_ag_start_audio_task(void* arg) {
    uint64_t frame_data_num;
    size_t item_size = 0;
    ESP_LOGI(TAG, "Entered HFP AG audio task");
    for (;;) {
        if (xSemaphoreTake(s_send_data_Semaphore, (TickType_t)portMAX_DELAY)) {
            s_now_enter_time = esp_timer_get_time();
            s_us_duration = s_now_enter_time - s_last_enter_time;
            if (s_audio_code == ESP_HF_AUDIO_STATE_CONNECTED_MSBC) {
                // time of a frame is 7.5ms, sample is 120, data is 2
                // (byte/sample), so a frame is 240 byte
                // (HF_SBC_ENC_RAW_DATA_SIZE)
                frame_data_num =
                    s_us_duration / BLOCK_DURATION_US * WBS_INPUT_DATA_SIZE;
                s_last_enter_time +=
                    frame_data_num / WBS_INPUT_DATA_SIZE * BLOCK_DURATION_US;
            } else {
                frame_data_num =
                    s_us_duration / BLOCK_DURATION_US * INPUT_DATA_SIZE;
                s_last_enter_time +=
                    frame_data_num / INPUT_DATA_SIZE * BLOCK_DURATION_US;
            }
            if (frame_data_num == 0) {
                continue;
            }
            esp_hf_ag_outgoing_data_ready();
        }
    }
}

void bt_hf_ag_start_audio(void) {
    s_send_data_Semaphore = xSemaphoreCreateBinary();
    xTaskCreate(bt_hf_ag_start_audio_task, "HFP_AG_audio_task", 2048, NULL,
                configMAX_PRIORITIES - 3, &s_bt_hf_ag_start_audio_task_handler);
    const esp_timer_create_args_t c_periodic_timer_args = {
        .callback = &bt_hf_ag_start_audio_timer_cb, .name = "periodic"};
    ESP_ERROR_CHECK(
        esp_timer_create(&c_periodic_timer_args, &s_periodic_timer));
    ESP_ERROR_CHECK(
        esp_timer_start_periodic(s_periodic_timer, GENERATOR_TICK_US));
    ESP_LOGI(TAG, "Started HFP AG audio");
    s_last_enter_time = esp_timer_get_time();
    return;
}

void bt_hf_ag_stop_audio(void) {
    if (s_bt_hf_ag_start_audio_task_handler) {
        vTaskDelete(s_bt_hf_ag_start_audio_task_handler);
        s_bt_hf_ag_start_audio_task_handler = NULL;
    }
    if (s_periodic_timer) {
        ESP_ERROR_CHECK(esp_timer_stop(s_periodic_timer));
        ESP_ERROR_CHECK(esp_timer_delete(s_periodic_timer));
    }
    if (s_send_data_Semaphore) {
        vSemaphoreDelete(s_send_data_Semaphore);
        s_send_data_Semaphore = NULL;
    }
    ESP_LOGI(TAG, "Stoped HFP AG audio task");
    return;
}

// called by hfp ag
// gets const data from mic, sends to stream output rb
static void bt_app_hf_ag_incoming_cb(const uint8_t* buf, uint32_t sz) {
    if (hfp_incoming_stream) {
        if (audio_element_get_state(hfp_incoming_stream) == AEL_STATE_RUNNING) {
            audio_element_output(hfp_incoming_stream, (char*)buf, sz);
        }
#ifdef CONFIG_DEBUG_AUDIO
        if (hfp_ag_audio_packet_count_incoming++ % 100 == 0) {
            ESP_LOGI(TAG, "bt_app_hf_ag_incoming_cb: %lu, sz: %lu", hfp_ag_audio_packet_count_incoming, sz);
        }
#endif
    }
}

// called by hfp ag
// gets data from stream input rb, sends to headphones through p_buf;
// needs esp_hf_ag_outgoing_data_ready
static uint32_t bt_app_hf_ag_outgoing_cb(uint8_t* p_buf, uint32_t sz) {
    int out_len_bytes = audio_element_input(hfp_outgoing_stream, (char*)p_buf, sz);
#ifdef CONFIG_DEBUG_AUDIO
    if (hfp_ag_audio_packet_count_outgoing++ % 100 == 0) {
        ESP_LOGI(TAG, "bt_app_hf_ag_outgoing_cb: %lu, sz: %lu, out_len_bytes: %d", hfp_ag_audio_packet_count_outgoing, sz, out_len_bytes);
    }
#endif
    return out_len_bytes;
}

void bt_hf_ag_cb(esp_hf_cb_event_t event, esp_hf_cb_param_t* param) {
    switch (event) {
    case ESP_HF_AUDIO_STATE_EVT:
        ESP_LOGI(TAG, "Audio state: %d (%s)", param->audio_stat.state,
                 (param->audio_stat.state == ESP_HF_AUDIO_STATE_CONNECTED ||
                  param->audio_stat.state == ESP_HF_AUDIO_STATE_CONNECTED_MSBC)
                     ? "CONNECTED"
                     : "DISCONNECTED");

        if (param->audio_stat.state == ESP_HF_AUDIO_STATE_CONNECTED ||
            param->audio_stat.state == ESP_HF_AUDIO_STATE_CONNECTED_MSBC) {
            s_audio_code = param->audio_stat.state;
            ESP_LOGI(TAG, "HFP audio ready - codec: %s",
                     param->audio_stat.state ==
                             ESP_HF_AUDIO_STATE_CONNECTED_MSBC
                         ? "mSBC"
                         : "CVSD");
            esp_hf_ag_register_data_callback(bt_app_hf_ag_incoming_cb,
                                             bt_app_hf_ag_outgoing_cb);
        } else if (param->audio_stat.state == ESP_HF_AUDIO_STATE_DISCONNECTED) {
        }
        break;

    default:
        ESP_LOGW(TAG, "HFP AG unhandled event: %d", event);
        break;
    }
}

static esp_err_t _hfp_stream_destroy(audio_element_handle_t self)
{
    hfp_stream_config_t *hfp = (hfp_stream_config_t *)audio_element_getdata(self);
    audio_free(hfp);
    return ESP_OK;
}

audio_element_handle_t hfp_stream_init(hfp_stream_config_t *config)
{
    AUDIO_NULL_CHECK(TAG, config, return NULL);
    audio_element_handle_t el = NULL;
    hfp_stream_config_t *hfp = audio_calloc(1, sizeof(hfp_stream_config_t));
    AUDIO_MEM_CHECK(TAG, hfp, return NULL);
    audio_element_cfg_t cfg = DEFAULT_AUDIO_ELEMENT_CONFIG();
    cfg.task_stack = -1; // No need task
    cfg.destroy = _hfp_stream_destroy;
    hfp->type = config->type;
    if (config->type == INCOMING_STREAM) {
        ESP_LOGI(TAG, "incoming stream init");
        cfg.tag = "hfp";
        el = audio_element_init(&cfg);
        hfp_incoming_stream = el;
        AUDIO_MEM_CHECK(TAG, el, {
            audio_free(hfp);
            return NULL;
        });
        audio_element_setdata(el, hfp);
    } else if (config->type == OUTGOING_STREAM) {
        ESP_LOGI(TAG, "outgoing stream init");
        cfg.tag = "hfp_outgoing";
        el = audio_element_init(&cfg);
        hfp_outgoing_stream = el;
        AUDIO_MEM_CHECK(TAG, el, {
            audio_free(hfp);
            return NULL;
        });
        audio_element_setdata(el, hfp);
    } else {
        ESP_LOGE(TAG, "error stream type");
    }
    return el;
}
#endif
