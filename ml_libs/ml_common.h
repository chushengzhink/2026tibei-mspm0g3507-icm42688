#ifndef ML_COMMON_H
#define ML_COMMON_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    ML_STATUS_OK = 0,
    ML_STATUS_INVALID_ARGUMENT,
    ML_STATUS_UNSUPPORTED,
    ML_STATUS_NOT_INITIALIZED,
    ML_STATUS_TIMEOUT,
    ML_STATUS_BUSY,
    ML_STATUS_NO_ACK,
    ML_STATUS_BUFFER_EMPTY,
    ML_STATUS_BUFFER_FULL,
    ML_STATUS_DEVICE_NOT_FOUND
} ml_status_t;

#endif
