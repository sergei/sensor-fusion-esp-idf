#ifndef ESP_SENSOR_FUSION_H
#define ESP_SENSOR_FUSION_H

#ifdef __cplusplus
extern "C" {
#endif

#include "sensor_fusion.h" // top level magCal and sensor fusion interfaces

typedef void (*on_fusion_data)(SensorFusionGlobals *sfg);

int start_fusing(on_fusion_data on_fusion_data_cb);

#ifdef __cplusplus
}
#endif

#endif // ESP_SENSOR_FUSION_H