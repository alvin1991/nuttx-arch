/*
 * UAVCAN data structure definition for libcanard.
 *
 * Autogenerated, do not edit.
 *
 */

#ifndef __DSDL_UAVCAN_EQUIPMENT_AHRS_RAWIMU
#define __DSDL_UAVCAN_EQUIPMENT_AHRS_RAWIMU

#include <uavcan_service/libcanard/canard.h>

#ifdef __cplusplus
extern "C"
{
#endif

/******************************* Source text **********************************
#
# Raw IMU data with timestamps.
#
# THIS DEFINITION MAY BE CHANGED IN A NON-BACKWARD-COMPATIBLE WAY IN THE FUTURE.
#

#
# Integration interval, seconds.
# Set to a non-positive value if the integrated samples are not available
# (in this case, only the latest point samples will be valid).
#
float32 integration_interval

#
# Angular velocity samples in radian/second.
# The samples are represented in the body frame, the axes are ordered as follows:
#   1. angular velocity around X (roll rate)
#   2. angular velocity around Y (pitch rate)
#   3. angular velocity around Z (yaw rate)
#
float16[3] rate_gyro_latest                 # Latest sample, radian/second
float32[3] rate_gyro_integral               # Integrated samples, radian/second

#
# Linear acceleration samples in meter/(second^2).
# The samples are represented in the body frame, the axes are ordered as follows:
#   1. linear acceleration along X (forward positive)
#   2. linear acceleration along Y (right positive)
#   3. linear acceleration along Z (down positive)
#
float16[3] accelerometer_latest             # Latest sample, meter/(second^2)
float32[3] accelerometer_integral           # Integrated samples, meter/(second^2)

#
# Covariance matrix. The diagonal entries are ordered as follows:
#   1. roll rate                (radian^2)/(second^2)
#   2. pitch rate               (radian^2)/(second^2)
#   3. yaw rate                 (radian^2)/(second^2)
#   4. forward acceleration     (meter^2)/(second^4)
#   5. rightward acceleration   (meter^2)/(second^4)
#   6. downward acceleration    (meter^2)/(second^4)
#
float16[<=36] covariance
******************************************************************************/

/********************* DSDL signature source definition ***********************
dsdl.uavcan.equipment.ahrs.RawIMU
saturated float32 integration_interval
saturated float16[3] rate_gyro_latest
saturated float32[3] rate_gyro_integral
saturated float16[3] accelerometer_latest
saturated float32[3] accelerometer_integral
saturated float16[<=36] covariance
******************************************************************************/

#define DSDL_UAVCAN_EQUIPMENT_AHRS_RAWIMU_ID               1003
#define DSDL_UAVCAN_EQUIPMENT_AHRS_RAWIMU_NAME             "dsdl.uavcan.equipment.ahrs.RawIMU"
#define DSDL_UAVCAN_EQUIPMENT_AHRS_RAWIMU_SIGNATURE        (0xC7A8C0D8386C2C8DULL)

#define DSDL_UAVCAN_EQUIPMENT_AHRS_RAWIMU_MAX_SIZE         ((902 + 7)/8)

// Constants

#define DSDL_UAVCAN_EQUIPMENT_AHRS_RAWIMU_RATE_GYRO_LATEST_LENGTH                        3
#define DSDL_UAVCAN_EQUIPMENT_AHRS_RAWIMU_RATE_GYRO_INTEGRAL_LENGTH                      3
#define DSDL_UAVCAN_EQUIPMENT_AHRS_RAWIMU_ACCELEROMETER_LATEST_LENGTH                    3
#define DSDL_UAVCAN_EQUIPMENT_AHRS_RAWIMU_ACCELEROMETER_INTEGRAL_LENGTH                  3
#define DSDL_UAVCAN_EQUIPMENT_AHRS_RAWIMU_COVARIANCE_MAX_LENGTH                          36

typedef struct
{
    // FieldTypes
    float      integration_interval;          // float32 Saturate
    float      rate_gyro_latest[3];           // Static Array 16bit[3] max items
    float      rate_gyro_integral[3];         // Static Array 32bit[3] max items
    float      accelerometer_latest[3];       // Static Array 16bit[3] max items
    float      accelerometer_integral[3];     // Static Array 32bit[3] max items
    struct
    {
        uint8_t    len;                       // Dynamic array length
        float*     data;                      // Dynamic Array 16bit[36] max items
    } covariance;

} dsdl_uavcan_equipment_ahrs_RawIMU;

static inline
uint32_t dsdl_uavcan_equipment_ahrs_RawIMU_encode(dsdl_uavcan_equipment_ahrs_RawIMU* source, void* msg_buf);

static inline
int32_t dsdl_uavcan_equipment_ahrs_RawIMU_decode(const CanardRxTransfer* transfer, uint16_t payload_len, dsdl_uavcan_equipment_ahrs_RawIMU* dest, uint8_t** dyn_arr_buf);

static inline
uint32_t dsdl_uavcan_equipment_ahrs_RawIMU_encode_internal(dsdl_uavcan_equipment_ahrs_RawIMU* source, void* msg_buf, uint32_t offset, uint8_t root_item);

static inline
int32_t dsdl_uavcan_equipment_ahrs_RawIMU_decode_internal(const CanardRxTransfer* transfer, uint16_t payload_len, dsdl_uavcan_equipment_ahrs_RawIMU* dest, uint8_t** dyn_arr_buf, int32_t offset, uint8_t tao);

/*
 * UAVCAN data structure definition for libcanard.
 *
 * Autogenerated, do not edit.
 *
 */

#ifndef CANARD_INTERNAL_SATURATE
#define CANARD_INTERNAL_SATURATE(x, max) ( ((x) > max) ? max : ( (-(x) > max) ? (-max) : (x) ) );
#endif

#ifndef CANARD_INTERNAL_SATURATE_UNSIGNED
#define CANARD_INTERNAL_SATURATE_UNSIGNED(x, max) ( ((x) > max) ? max : (x) );
#endif

#define CANARD_INTERNAL_ENABLE_TAO  ((uint8_t) 1)
#define CANARD_INTERNAL_DISABLE_TAO ((uint8_t) 0)

#if defined(__GNUC__)
# define CANARD_MAYBE_UNUSED(x) x __attribute__((unused))
#else
# define CANARD_MAYBE_UNUSED(x) x
#endif

/**
  * @brief dsdl_uavcan_equipment_ahrs_RawIMU_encode_internal
  * @param source : pointer to source data struct
  * @param msg_buf: pointer to msg storage
  * @param offset: bit offset to msg storage
  * @param root_item: for detecting if TAO should be used
  * @retval returns offset
  */
uint32_t dsdl_uavcan_equipment_ahrs_RawIMU_encode_internal(dsdl_uavcan_equipment_ahrs_RawIMU* source,
  void* msg_buf,
  uint32_t offset,
  uint8_t CANARD_MAYBE_UNUSED(root_item))
{
    uint32_t c = 0;

    canardEncodeScalar(msg_buf, offset, 32, (void*)&source->integration_interval); // 2147483647
    offset += 32;

    // Static array (rate_gyro_latest)
    for (c = 0; c < 3; c++)
    {
        canardEncodeScalar(msg_buf, offset, 16, (void*)(source->rate_gyro_latest + c)); // 32767
        offset += 16;
    }

    // Static array (rate_gyro_integral)
    for (c = 0; c < 3; c++)
    {
        canardEncodeScalar(msg_buf, offset, 32, (void*)(source->rate_gyro_integral + c)); // 2147483647
        offset += 32;
    }

    // Static array (accelerometer_latest)
    for (c = 0; c < 3; c++)
    {
        canardEncodeScalar(msg_buf, offset, 16, (void*)(source->accelerometer_latest + c)); // 32767
        offset += 16;
    }

    // Static array (accelerometer_integral)
    for (c = 0; c < 3; c++)
    {
        canardEncodeScalar(msg_buf, offset, 32, (void*)(source->accelerometer_integral + c)); // 2147483647
        offset += 32;
    }

    // Dynamic Array (covariance)
    if (! root_item)
    {
        // - Add array length
        canardEncodeScalar(msg_buf, offset, 6, (void*)&source->covariance.len);
        offset += 6;
    }

    // - Add array items
    for (c = 0; c < source->covariance.len; c++)
    {
        canardEncodeScalar(msg_buf,
                           offset,
                           16,
                           (void*)(source->covariance.data + c));// 32767
        offset += 16;
    }

    return offset;
}

/**
  * @brief dsdl_uavcan_equipment_ahrs_RawIMU_encode
  * @param source : Pointer to source data struct
  * @param msg_buf: Pointer to msg storage
  * @retval returns message length as bytes
  */
uint32_t dsdl_uavcan_equipment_ahrs_RawIMU_encode(dsdl_uavcan_equipment_ahrs_RawIMU* source, void* msg_buf)
{
    uint32_t offset = 0;

    offset = dsdl_uavcan_equipment_ahrs_RawIMU_encode_internal(source, msg_buf, offset, 1);

    return (offset + 7 ) / 8;
}

/**
  * @brief dsdl_uavcan_equipment_ahrs_RawIMU_decode_internal
  * @param transfer: Pointer to CanardRxTransfer transfer
  * @param payload_len: Payload message length
  * @param dest: Pointer to destination struct
  * @param dyn_arr_buf: NULL or Pointer to memory storage to be used for dynamic arrays
  *                     dsdl_uavcan_equipment_ahrs_RawIMU dyn memory will point to dyn_arr_buf memory.
  *                     NULL will ignore dynamic arrays decoding.
  * @param offset: Call with 0, bit offset to msg storage
  * @param tao: is tail array optimization used
  * @retval offset or ERROR value if < 0
  */
int32_t dsdl_uavcan_equipment_ahrs_RawIMU_decode_internal(
  const CanardRxTransfer* transfer,
  uint16_t CANARD_MAYBE_UNUSED(payload_len),
  dsdl_uavcan_equipment_ahrs_RawIMU* dest,
  uint8_t** CANARD_MAYBE_UNUSED(dyn_arr_buf),
  int32_t offset,
  uint8_t CANARD_MAYBE_UNUSED(tao))
{
    int32_t ret = 0;
    uint32_t c = 0;

    ret = canardDecodeScalar(transfer, offset, 32, false, (void*)&dest->integration_interval);
    if (ret != 32)
    {
        goto dsdl_uavcan_equipment_ahrs_RawIMU_error_exit;
    }
    offset += 32;

    // Static array (rate_gyro_latest)
    for (c = 0; c < 3; c++)
    {
        ret = canardDecodeScalar(transfer, offset, 16, false, (void*)(dest->rate_gyro_latest + c));
        if (ret != 16)
        {
            goto dsdl_uavcan_equipment_ahrs_RawIMU_error_exit;
        }
        offset += 16;
    }

    // Static array (rate_gyro_integral)
    for (c = 0; c < 3; c++)
    {
        ret = canardDecodeScalar(transfer, offset, 32, false, (void*)(dest->rate_gyro_integral + c));
        if (ret != 32)
        {
            goto dsdl_uavcan_equipment_ahrs_RawIMU_error_exit;
        }
        offset += 32;
    }

    // Static array (accelerometer_latest)
    for (c = 0; c < 3; c++)
    {
        ret = canardDecodeScalar(transfer, offset, 16, false, (void*)(dest->accelerometer_latest + c));
        if (ret != 16)
        {
            goto dsdl_uavcan_equipment_ahrs_RawIMU_error_exit;
        }
        offset += 16;
    }

    // Static array (accelerometer_integral)
    for (c = 0; c < 3; c++)
    {
        ret = canardDecodeScalar(transfer, offset, 32, false, (void*)(dest->accelerometer_integral + c));
        if (ret != 32)
        {
            goto dsdl_uavcan_equipment_ahrs_RawIMU_error_exit;
        }
        offset += 32;
    }

    // Dynamic Array (covariance)
    //  - Last item in struct & Root item & (Array Size > 8 bit), tail array optimization
    if (payload_len && tao == CANARD_INTERNAL_ENABLE_TAO)
    {
        //  - Calculate Array length from MSG length
        dest->covariance.len = ((payload_len * 8) - offset ) / 16; // 16 bit array item size
    }
    else
    {
        // - Array length 6 bits
        ret = canardDecodeScalar(transfer,
                                 offset,
                                 6,
                                 false,
                                 (void*)&dest->covariance.len); // 32767
        if (ret != 6)
        {
            goto dsdl_uavcan_equipment_ahrs_RawIMU_error_exit;
        }
        offset += 6;
    }

    //  - Get Array
    if (dyn_arr_buf)
    {
        dest->covariance.data = (float*)*dyn_arr_buf;
    }

    for (c = 0; c < dest->covariance.len; c++)
    {
        if (dyn_arr_buf)
        {
            ret = canardDecodeScalar(transfer,
                                     offset,
                                     16,
                                     false,
                                     (void*)*dyn_arr_buf); // 32767
            if (ret != 16)
            {
                goto dsdl_uavcan_equipment_ahrs_RawIMU_error_exit;
            }
            *dyn_arr_buf = (uint8_t*)(((float*)*dyn_arr_buf) + 1);
        }
        offset += 16;
    }
    return offset;

dsdl_uavcan_equipment_ahrs_RawIMU_error_exit:
    if (ret < 0)
    {
        return ret;
    }
    else
    {
        return -CANARD_ERROR_INTERNAL;
    }
}

/**
  * @brief dsdl_uavcan_equipment_ahrs_RawIMU_decode
  * @param transfer: Pointer to CanardRxTransfer transfer
  * @param payload_len: Payload message length
  * @param dest: Pointer to destination struct
  * @param dyn_arr_buf: NULL or Pointer to memory storage to be used for dynamic arrays
  *                     dsdl_uavcan_equipment_ahrs_RawIMU dyn memory will point to dyn_arr_buf memory.
  *                     NULL will ignore dynamic arrays decoding.
  * @retval offset or ERROR value if < 0
  */
int32_t dsdl_uavcan_equipment_ahrs_RawIMU_decode(const CanardRxTransfer* transfer,
  uint16_t payload_len,
  dsdl_uavcan_equipment_ahrs_RawIMU* dest,
  uint8_t** dyn_arr_buf)
{
    const int32_t offset = 0;
    int32_t ret = 0;

    /* Backward compatibility support for removing TAO
     *  - first try to decode with TAO DISABLED
     *  - if it fails fall back to TAO ENABLED
     */
    uint8_t tao = CANARD_INTERNAL_DISABLE_TAO;

    while (1)
    {
        // Clear the destination struct
        for (uint32_t c = 0; c < sizeof(dsdl_uavcan_equipment_ahrs_RawIMU); c++)
        {
            ((uint8_t*)dest)[c] = 0x00;
        }

        ret = dsdl_uavcan_equipment_ahrs_RawIMU_decode_internal(transfer, payload_len, dest, dyn_arr_buf, offset, tao);

        if (ret >= 0)
        {
            break;
        }

        if (tao == CANARD_INTERNAL_ENABLE_TAO)
        {
            break;
        }
        tao = CANARD_INTERNAL_ENABLE_TAO;
    }

    return ret;
}

#ifdef __cplusplus
} // extern "C"
#endif
#endif // __DSDL_UAVCAN_EQUIPMENT_AHRS_RAWIMU