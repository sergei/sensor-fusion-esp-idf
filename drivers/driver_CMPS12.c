#include "sensor_fusion.h"
#include "drivers.h"
#include <esp_log.h>
#include "driver/i2c.h"


#define CMPS12_COUNTSPERG 1000
#define CMPS12_COUNTSPERUT 16
#define CMPS12_COUNTSPERDEGPERSEC 16

static const int  I2C_MASTER_NUM = 0;              /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
static const uint32_t I2C_MASTER_FREQ_HZ = 100000; /*!< I2C master clock frequency */
static const int I2C_MASTER_TX_BUF_DISABLE  =  0;  /*!< I2C master doesn't need buffer */
static const int  I2C_MASTER_RX_BUF_DISABLE =  0;  /*!< I2C master doesn't need buffer */
static const int MS_TO_WAIT = 100;

static const char *TAG = "driver_CMPS12";

const int SDA_IO_NUM = 16;  // FIXME mode to board.h or somewhere else
const int SCL_IO_NUM = 17;  // FIXME mode to board.h or somewhere else

static esp_err_t read_uint8(struct PhysicalSensor *sensor, uint8_t regNum, uint8_t *val)  {

    return i2c_master_write_read_device(I2C_MASTER_NUM, sensor->addr,
                                    &regNum, 1, val, 1,
                                    MS_TO_WAIT / portTICK_RATE_MS);
}

static esp_err_t read_uint16(struct PhysicalSensor *sensor, uint8_t regNum, int16_t *val){
    esp_err_t err;

    uint8_t hi;
    err = read_uint8(sensor, regNum, &hi);
    if ( err != ESP_OK)
        return err;

    uint8_t lo;
    err = read_uint8(sensor, regNum + 1, &lo);
    if ( err != ESP_OK)
        return err;

    *val = (((int16_t)hi) << 8) | (int16_t)lo;
    return ESP_OK;
}

static esp_err_t readSensorValue(struct PhysicalSensor *sensor, uint8_t regNum, int16_t *sample) {
    for(int i=0; i < 3; i++){
        esp_err_t err = read_uint16(sensor, regNum, &sample[i]);
        if( err != ESP_OK){
            ESP_LOGE(TAG, "Failed to read reg %02X", regNum);
            return err;
        }
        regNum += 2;
    }
    conditionSample(sample);  // truncate negative values to -32767
    return ESP_OK;
}


int8_t CMPS12_Init(struct PhysicalSensor *sensor, SensorFusionGlobals *sfg){

    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
            .mode = I2C_MODE_MASTER,
            .sda_io_num = SDA_IO_NUM,
            .scl_io_num = SCL_IO_NUM,
            .sda_pullup_en = GPIO_PULLUP_ENABLE,
            .scl_pullup_en = GPIO_PULLUP_ENABLE,
            .master = {
                    .clk_speed = I2C_MASTER_FREQ_HZ,
            },
            .clk_flags = 0
    };

    i2c_param_config(i2c_master_port, &conf);

    esp_err_t  err = i2c_driver_install(i2c_master_port, conf.mode,
                                        I2C_MASTER_RX_BUF_DISABLE,
                                        I2C_MASTER_TX_BUF_DISABLE, 0);

    sfg->Accel.iWhoAmI = 12;
    sfg->Accel.iCountsPerg = CMPS12_COUNTSPERG;
    sfg->Accel.fgPerCount = 1.0F / CMPS12_COUNTSPERG;
    sfg->Accel.isEnabled = true;

    sfg->Mag.iWhoAmI = 12;
    sfg->Mag.iCountsPeruT = CMPS12_COUNTSPERUT;
    sfg->Mag.fCountsPeruT = (float) CMPS12_COUNTSPERUT;
    sfg->Mag.fuTPerCount = 1.0F / CMPS12_COUNTSPERUT;
    sfg->Mag.isEnabled = true;

    sfg->Gyro.iWhoAmI = 12;
    sfg->Gyro.iCountsPerDegPerSec = CMPS12_COUNTSPERDEGPERSEC;
    sfg->Gyro.fDegPerSecPerCount = 1.0F / CMPS12_COUNTSPERDEGPERSEC;
    sfg->Gyro.isEnabled = true;

    sensor->isInitialized = F_USING_ACCEL | F_USING_MAG | F_USING_GYRO;

    if ( err != ESP_OK){
        ESP_LOGE(TAG, "Failed to install driver I2C error %d %s", err, esp_err_to_name(err));
        return SENSOR_ERROR_INVALID_PARAM;
    }else{
        ESP_LOGI(TAG, "CMPS12_Init OK");
        return SENSOR_ERROR_NONE;
    }
}

int8_t CMPS12_Read(struct PhysicalSensor *sensor, SensorFusionGlobals *sfg){
    int16_t   sample[3];
    // Read mag X, reg 0x06 (high byte), 0x07 (low byte)
    // Read mag Y, reg 0x08 (high byte), 0x09 (low byte)
    // Read mag Z, reg 0x0A (high byte), 0x0B (low byte)
    if ( readSensorValue(sensor, 0x06, sample)  != ESP_OK)
        return SENSOR_ERROR_READ;

    ESP_LOGD(TAG, ",mag,t,%lld,x,%d,y,%d,z,%d", esp_timer_get_time(), sample[0], sample[1], sample[2]);
    // place the 6 bytes read into the 16 bit mag structure
    addToFifo((union FifoSensor*) &(sfg->Mag), MAG_FIFO_SIZE, sample);

    // Read acc X, reg 0x0C (high byte), 0x0D (low byte)
    // Read acc Y, reg 0x0E (high byte), 0x0F (low byte)
    // Read acc Z, reg 0x10 (high byte), 0x11 (low byte)
    if ( readSensorValue(sensor, 0x0C, sample)  != ESP_OK)
        return SENSOR_ERROR_READ;

    ESP_LOGD(TAG, ",acc,t,%lld,x,%d,y,%d,z,%d", esp_timer_get_time(), sample[0], sample[1], sample[2]);

    // place the 6 bytes read into the 16 bit mag structure
    addToFifo((union FifoSensor*) &(sfg->Accel), ACCEL_FIFO_SIZE, sample);

    // Read gyro X, reg 0x12 (high byte), 0x13 (low byte)
    // Read gyro Y, reg 0x14 (high byte), 0x15 (low byte)
    // Read gyro Z, reg 0x16 (high byte), 0x17 (low byte)
    if ( readSensorValue(sensor, 0x12, sample)  != ESP_OK)
        return SENSOR_ERROR_READ;

    ESP_LOGD(TAG, ",gyro,t,%lld,x,%d,y,%d,z,%d", esp_timer_get_time(), sample[0], sample[1], sample[2]);
    // place the 6 bytes read into the 16 bit mag structure
    addToFifo((union FifoSensor*) &(sfg->Gyro), GYRO_FIFO_SIZE, sample);

    return SENSOR_ERROR_NONE;
}
