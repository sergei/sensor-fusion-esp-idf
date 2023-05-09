#include <nvs_flash.h>
#include "sensor_fusion.h"
#include "control.h"

#include "sdkconfig.h"
#include "esp_task.h"

#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#include "esp_spp_api.h"

// direct access to sfg here is the only place in the entire library where we cannot simply
// pass a pointer.  This is because it is needed by the UART interrupt handlers.  Since this
// only occurs here, in a subsystem which is defined to be application dependent, that is
// considered acceptable.
extern SensorFusionGlobals sfg;
uint8_t sUARTOutputBuffer[256];     ///< main output buffer
static const char *TAG = "sensor_fusion_control";


static const esp_spp_mode_t esp_spp_mode = ESP_SPP_MODE_CB;
static const esp_spp_sec_t sec_mask = ESP_SPP_SEC_AUTHENTICATE;
static const esp_spp_role_t role_slave = ESP_SPP_ROLE_SLAVE;
#define SPP_SERVER_NAME "SPP_SERVER"
#define EXAMPLE_DEVICE_NAME "BlueRadios123456"
static uint32_t bt_spp_handle= INVALID_HANDLE;

void onReceivedByte(uint8_t data) {
    static char iCommandBuffer_B[5] = "~~~~";	// 5 bytes long to include the unused terminating \0
    sfg.setStatus(&sfg, RECEIVING_WIRELESS);
    DecodeCommandBytes(&sfg, iCommandBuffer_B, &data, 1);
}


static char *bda2str(uint8_t *bda, char *str, size_t size)
{
    if (bda == NULL || str == NULL || size < 18) {
        return NULL;
    }

    uint8_t *p = bda;
    sprintf(str, "%02x:%02x:%02x:%02x:%02x:%02x",
            p[0], p[1], p[2], p[3], p[4], p[5]);
    return str;
}

void esp_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    char bda_str[18] = {0};

    switch (event) {
        case ESP_BT_GAP_AUTH_CMPL_EVT:{
            if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
                ESP_LOGI(TAG, "authentication success: %s bda:[%s]", param->auth_cmpl.device_name,
                         bda2str(param->auth_cmpl.bda, bda_str, sizeof(bda_str)));
            } else {
                ESP_LOGE(TAG, "authentication failed, status:%d", param->auth_cmpl.stat);
            }
            break;
        }
        case ESP_BT_GAP_PIN_REQ_EVT:{
            ESP_LOGI(TAG, "ESP_BT_GAP_PIN_REQ_EVT min_16_digit:%d", param->pin_req.min_16_digit);
            if (param->pin_req.min_16_digit) {
                ESP_LOGI(TAG, "Input pin code: 0000 0000 0000 0000");
                esp_bt_pin_code_t pin_code = {0};
                esp_bt_gap_pin_reply(param->pin_req.bda, true, 16, pin_code);
            } else {
                ESP_LOGI(TAG, "Input pin code: 1234");
                esp_bt_pin_code_t pin_code;
                pin_code[0] = '1';
                pin_code[1] = '2';
                pin_code[2] = '3';
                pin_code[3] = '4';
                esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, pin_code);
            }
            break;
        }

#if (CONFIG_BT_SSP_ENABLED == true)
            case ESP_BT_GAP_CFM_REQ_EVT:
        ESP_LOGI(TAG, "ESP_BT_GAP_CFM_REQ_EVT Please compare the numeric value: %d", param->cfm_req.num_val);
        esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
        break;
    case ESP_BT_GAP_KEY_NOTIF_EVT:
        ESP_LOGI(TAG, "ESP_BT_GAP_KEY_NOTIF_EVT passkey:%d", param->key_notif.passkey);
        break;
    case ESP_BT_GAP_KEY_REQ_EVT:
        ESP_LOGI(TAG, "ESP_BT_GAP_KEY_REQ_EVT Please enter passkey!");
        break;
#endif

        case ESP_BT_GAP_MODE_CHG_EVT:
            ESP_LOGI(TAG, "ESP_BT_GAP_MODE_CHG_EVT mode:%d bda:[%s]", param->mode_chg.mode,
                     bda2str(param->mode_chg.bda, bda_str, sizeof(bda_str)));
            break;

        default: {
            ESP_LOGI(TAG, "event: %d", event);
            break;
        }
    }
}

static void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
    char bda_str[18] = {0};

    switch (event) {
        case ESP_SPP_INIT_EVT:
            if (param->init.status == ESP_SPP_SUCCESS) {
                ESP_LOGI(TAG, "ESP_SPP_INIT_EVT");
                esp_spp_start_srv(sec_mask, role_slave, 0, SPP_SERVER_NAME);
            } else {
                ESP_LOGE(TAG, "ESP_SPP_INIT_EVT status:%d", param->init.status);
            }
            break;
        case ESP_SPP_DISCOVERY_COMP_EVT:
            ESP_LOGI(TAG, "ESP_SPP_DISCOVERY_COMP_EVT");
            break;
        case ESP_SPP_OPEN_EVT:
            ESP_LOGI(TAG, "ESP_SPP_OPEN_EVT");
            break;
        case ESP_SPP_CLOSE_EVT:
            ESP_LOGI(TAG, "ESP_SPP_CLOSE_EVT status:%d handle:%d close_by_remote:%d", param->close.status,
                     param->close.handle, param->close.async);
            bt_spp_handle = INVALID_HANDLE;
            break;
        case ESP_SPP_START_EVT:
            if (param->start.status == ESP_SPP_SUCCESS) {
                ESP_LOGI(TAG, "ESP_SPP_START_EVT handle:%d sec_id:%d scn:%d", param->start.handle, param->start.sec_id,
                         param->start.scn);
                esp_bt_dev_set_device_name(EXAMPLE_DEVICE_NAME);
                esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
            } else {
                ESP_LOGE(TAG, "ESP_SPP_START_EVT status:%d", param->start.status);
            }
            break;
        case ESP_SPP_CL_INIT_EVT:
            ESP_LOGI(TAG, "ESP_SPP_CL_INIT_EVT");
            break;
        case ESP_SPP_DATA_IND_EVT:
            ESP_LOGI(TAG, "ESP_SPP_DATA_IND_EVT len:%d handle:%d",
                     param->data_ind.len, param->data_ind.handle);
            if (param->data_ind.len < 128) {
                esp_log_buffer_hex("", param->data_ind.data, param->data_ind.len);
            }
            for(uint16_t i=0; i < param->data_ind.len; i++){
                onReceivedByte(param->data_ind.data[i]);
            }
            break;
        case ESP_SPP_CONG_EVT:
            ESP_LOGI(TAG, "ESP_SPP_CONG_EVT");
            break;
        case ESP_SPP_WRITE_EVT:
            ESP_LOGI(TAG, "ESP_SPP_WRITE_EVT");
            break;
        case ESP_SPP_SRV_OPEN_EVT:
            ESP_LOGI(TAG, "ESP_SPP_SRV_OPEN_EVT status:%d handle:%d, rem_bda:[%s]", param->srv_open.status,
                 param->srv_open.handle, bda2str(param->srv_open.rem_bda, bda_str, sizeof(bda_str)));
            bt_spp_handle = param->srv_open.handle;
            break;
        case ESP_SPP_SRV_STOP_EVT:
            ESP_LOGI(TAG, "ESP_SPP_SRV_STOP_EVT");
            break;
        case ESP_SPP_UNINIT_EVT:
            ESP_LOGI(TAG, "ESP_SPP_UNINIT_EVT");
            break;
        default:
            break;
    }
}

void initBtSPP() {
    char bda_str[18] = {0};
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    bt_cfg.mode = ESP_BT_MODE_CLASSIC_BT;
    if ((ret = esp_bt_controller_init(&bt_cfg)) != ESP_OK) {
        ESP_LOGE(TAG, "%s initialize controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT)) != ESP_OK) {
        ESP_LOGE(TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bluedroid_init()) != ESP_OK) {
        ESP_LOGE(TAG, "%s initialize bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bluedroid_enable()) != ESP_OK) {
        ESP_LOGE(TAG, "%s enable bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bt_gap_register_callback(esp_bt_gap_cb)) != ESP_OK) {
        ESP_LOGE(TAG, "%s gap register failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_spp_register_callback(esp_spp_cb)) != ESP_OK) {
        ESP_LOGE(TAG, "%s spp register failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_spp_init(esp_spp_mode)) != ESP_OK) {
        ESP_LOGE(TAG, "%s spp init failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

#if (CONFIG_BT_SSP_ENABLED == true)
    /* Set default parameters for Secure Simple Pairing */
    esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
    esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_IO;
    esp_bt_gap_set_security_param(param_type, &iocap, sizeof(uint8_t));
#endif

    /*
     * Set default parameters for Legacy Pairing
     * Use variable pin, input pin code when pairing
     */
    esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_VARIABLE;
    esp_bt_pin_code_t pin_code;
    esp_bt_gap_set_pin(pin_type, 0, pin_code);

    ESP_LOGI(TAG, "Own address:[%s]", bda2str((uint8_t *)esp_bt_dev_get_address(), bda_str, sizeof(bda_str)));

}
// Blocking function pipes specified buffer to both output UARTS
int8_t writeControlPort(ControlSubsystem *pComm, uint8_t buffer[], uint16_t nbytes){
    if( bt_spp_handle != INVALID_HANDLE){
        esp_err_t err = esp_spp_write(bt_spp_handle, nbytes, buffer);
        if ( err == ESP_OK){
            ESP_LOGI(TAG, "Sent %d bytes", nbytes);
        }else{
            ESP_LOGE(TAG, "Error sending %d bytes %s", nbytes, esp_err_to_name(err));
        }
        return 1;
    }
    return 0;
}

int8_t initializeControlPort(ControlSubsystem *pComm) {
    pComm->DefaultQuaternionPacketType = Q3;    // default to simplest algorithm
    pComm->QuaternionPacketType = Q3;           // default to simplest algorithm
    pComm->AngularVelocityPacketOn = true;      // transmit angular velocity packet
    pComm->DebugPacketOn = true;                // transmit debug packet
    pComm->RPCPacketOn = true;                  // transmit roll, pitch, compass packet
    pComm->AltPacketOn = true;                 // Altitude packet
    pComm->AccelCalPacketOn = 0;
    pComm->write = writeControlPort;
    pComm->stream = CreateAndSendPackets;

    // Initialize the Bluetooth UART
    initBtSPP();

    return 1;
}
