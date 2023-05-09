#include <esp_log.h>
#include <nvs_flash.h>
#include <sensor_fusion.h>

static char cal_storage_buffer[512];
static int cal_storage_buffer_valid = 0;

static const char *TAG = "sensor-fusion-esp-idf-storage";

static nvs_handle_t openNvs() {
    // Initialize NVS
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGE(TAG, "Formatting  NVS storage (%d)-%s", err, esp_err_to_name(err));
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK( err );

    // Open
    ESP_LOGI(TAG, "Opening Non-Volatile Storage (NVS) handle... ");
    nvs_handle_t my_handle;
    err = nvs_open("sensor-fusion", NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error (%s) opening NVS handle!\n", esp_err_to_name(err));
        return 0;
    } else {
        return my_handle;
    }

}


void *GetCalibrationDataBuffer(){
    if( !cal_storage_buffer_valid ){
        nvs_handle_t handle = openNvs();

        size_t required_size = 0;  // value will default to 0, if not set yet in NVS
        nvs_get_blob(handle, "sens_cal", NULL, &required_size);
        if (required_size > 0) {
            ESP_ERROR_CHECK(nvs_get_blob(handle, "sens_cal", cal_storage_buffer, &required_size));
            ESP_LOGI(TAG, "Read %d bytes from NVS", required_size);
        }
        nvs_close(handle);
        cal_storage_buffer_valid = 1;
    }
    return cal_storage_buffer;
}

static void StoreBuffer(){
    nvs_handle_t handle = openNvs();
    ESP_ERROR_CHECK(nvs_set_blob(handle, "sens_cal", cal_storage_buffer, sizeof(cal_storage_buffer)));
}

void SaveMagCalibrationToNVM(SensorFusionGlobals *sfg)
{
#if F_USING_MAG
    uint8_t *pSrc, *pDst;					// scratch pointers
    int16_t i;						// loop counter
    uint8_t iNVMBuffer[256];				// NVM write buffer (smallest size writeable to flash)
    uint32_t itmp32;

    // copy existing magnetic, gyro and accelerometer calibrations to buffer
    pSrc = (uint8 *) CALIBRATION_NVM_ADDR;
    pDst = iNVMBuffer;
    for (i = 0; i < 256; i++)
        *(pDst++) = *(pSrc++);
    // default to no magnetic calibration in header
    iNVMBuffer[MAG_NVM_OFFSET] = iNVMBuffer[MAG_NVM_OFFSET + 1] = iNVMBuffer[MAG_NVM_OFFSET + 2] = iNVMBuffer[MAG_NVM_OFFSET + 3] = 0xFF;

    // fill the buffer with the magnetic calibration in bytes 0 to 67 (total 68 bytes)
    // [0-3]: four byte header denoting magnetic calibration present
    itmp32 = 0x12345678;
    pSrc = (uint8 *) &itmp32;
    pDst = iNVMBuffer + MAG_NVM_OFFSET;
    for (i = 0; i < 4; i++)
        *(pDst++) = *(pSrc++);
    // [4-67]: magnetic calibration: 15x float + 1x int32 total 64 bytes
    pSrc = (uint8 *) &(sfg->MagCal);
    for (i = 0; i < 64; i++)
        *(pDst++) = *(pSrc++);

    // write the whole buffer contents to NVM
    StoreBuffer();
#endif // if F_USING_MAG
}

void SaveGyroCalibrationToNVM(SensorFusionGlobals *sfg)
{
#if F_USING_GYRO && (F_9DOF_GBY_KALMAN || F_6DOF_GY_KALMAN)
    uint8_t *pSrc, *pDst;					// scratch pointers
    int16_t i;						// loop counter
    uint8_t iNVMBuffer[256];				// NVM write buffer
    uint32_t itmp32;

    // copy existing magnetic, gyro and accelerometer calibrations to buffer
    pSrc = (uint8 *) CALIBRATION_NVM_ADDR;
    pDst = iNVMBuffer;
    for (i = 0; i < 256; i++)
        *(pDst++) = *(pSrc++);
    // default to no gyroscope calibration in header
    iNVMBuffer[GYRO_NVM_OFFSET] = iNVMBuffer[GYRO_NVM_OFFSET + 1] = iNVMBuffer[GYRO_NVM_OFFSET + 2] = iNVMBuffer[GYRO_NVM_OFFSET + 3] = 0xFF;

    // define the four header bytes
    // [0-3]: four byte header denoting gyro calibration present
    itmp32 = 0x12345678;
    pSrc = (uint8 *) &itmp32;
    pDst = iNVMBuffer + GYRO_NVM_OFFSET;
    for (i = 0; i < 4; i++)
        *(pDst++) = *(pSrc++);

    // [4-15]: 3 gyro offset floats totalling 12 bytes
#if F_9DOF_GBY_KALMAN
    pSrc = (uint8 *) sfg->SV_9DOF_GBY_KALMAN.fbPl;
#elif F_6DOF_GY_KALMAN
    pSrc = (uint8 *) sfg->SV_6DOF_GY_KALMAN.fbPl;
#endif
    for (i = 0; i < 12; i++)
        *(pDst++) = *(pSrc++);

    // write the buffer contents to NVM
    StoreBuffer();
#endif
}

void SaveAccelCalibrationToNVM(SensorFusionGlobals *sfg)
{
#if F_USING_ACCEL
    uint8_t *pSrc, *pDst;					// scratch pointers
    int16_t i;							// loop counter
    uint8_t iNVMBuffer[256];				// NVM write buffer
    uint32_t itmp32;

    // copy existing magnetic, gyro and accelerometer calibrations to buffer
    pSrc = (uint8 *) CALIBRATION_NVM_ADDR;
    pDst = iNVMBuffer;
    for (i = 0; i < 256; i++)
        *(pDst++) = *(pSrc++);
    // default to no accelerometer calibration in header
    iNVMBuffer[ACCEL_NVM_OFFSET] = iNVMBuffer[ACCEL_NVM_OFFSET + 1] = iNVMBuffer[ACCEL_NVM_OFFSET + 2] = iNVMBuffer[ACCEL_NVM_OFFSET + 3] = 0xFF;

    // [0-3]: four byte header denoting accelerometer calibration present
    itmp32 = 0x12345678;
    pSrc = (uint8 *) &itmp32;
    pDst = iNVMBuffer + ACCEL_NVM_OFFSET;
    for (i = 0; i < 4; i++)
        *(pDst++) = *(pSrc++);

    // [4-87]: 21 precision accelerometer calibration floats totalling 84 bytes
    pSrc = (uint8 *) &(sfg->AccelCal);
    for (i = 0; i < 84; i++)
        *(pDst++) = *(pSrc++);

    // write the buffer contents to NVM
    StoreBuffer();
#endif
}

void EraseMagCalibrationFromNVM(void)
{
    uint8_t *pSrc, *pDst;					// scratch pointers
    int16_t i;						// loop counter
    uint8_t iNVMBuffer[256];				// NVM write buffer

    // copy existing magnetic, gyro and accelerometer calibrations to buffer
    pSrc = (uint8 *) CALIBRATION_NVM_ADDR;
    pDst = iNVMBuffer;
    for (i = 0; i < 256; i++)
        *(pDst++) = *(pSrc++);

    // set no magnetic calibration in header
    iNVMBuffer[MAG_NVM_OFFSET] = iNVMBuffer[MAG_NVM_OFFSET + 1] = iNVMBuffer[MAG_NVM_OFFSET + 2] = iNVMBuffer[MAG_NVM_OFFSET + 3] = 0xFF;

    // write the buffer to flash
    StoreBuffer();
}

void EraseGyroCalibrationFromNVM(void)
{
    uint8_t *pSrc, *pDst;					// scratch pointers
    int16_t i;							// loop counter
    uint8_t iNVMBuffer[256];				// NVM write buffer

    // copy existing magnetic, gyro and accelerometer calibrations to buffer
    pSrc = (uint8 *) CALIBRATION_NVM_ADDR;
    pDst = iNVMBuffer;
    for (i = 0; i < 256; i++)
        *(pDst++) = *(pSrc++);

    // set no gyroscope calibration in header
    iNVMBuffer[GYRO_NVM_OFFSET] = iNVMBuffer[GYRO_NVM_OFFSET + 1] = iNVMBuffer[GYRO_NVM_OFFSET + 2] = iNVMBuffer[GYRO_NVM_OFFSET + 3] = 0xFF;

    // write the buffer to flash
    StoreBuffer();
}

void EraseAccelCalibrationFromNVM(void)
{
    uint8_t *pSrc, *pDst;					// scratch pointers
    int16_t i;							// loop counter
    uint8_t iNVMBuffer[256];				// NVM write buffer

    // copy existing magnetic, gyro and accelerometer calibrations to buffer
    pSrc = (uint8 *) CALIBRATION_NVM_ADDR;
    pDst = iNVMBuffer;
    for (i = 0; i < 256; i++)
        *(pDst++) = *(pSrc++);

    // set no gyroscope calibration in header
    iNVMBuffer[ACCEL_NVM_OFFSET] = iNVMBuffer[ACCEL_NVM_OFFSET + 1] = iNVMBuffer[ACCEL_NVM_OFFSET + 2] = iNVMBuffer[ACCEL_NVM_OFFSET + 3] = 0xFF;

    // write the buffer to flash
    StoreBuffer();
}
