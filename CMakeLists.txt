FILE(GLOB_RECURSE sources
        ./source/*.*
        ./drivers/*.*
        ./sensor_fusion/*.*
        ./hal/*.*
        )

idf_component_register(SRCS ${sources}
        INCLUDE_DIRS
        ./source
        ./interfaces
        ./sensor_fusion
        ./drivers
        ./hal

        REQUIRES "nvs_flash"
        REQUIRES "bt"
        )
# Remove pr change this line to adjust the size of log component
target_compile_definitions(${COMPONENT_LIB} PUBLIC "-DLOG_LOCAL_LEVEL=ESP_LOG_VERBOSE")