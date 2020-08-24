/*
 * UAVCAN data structure definition for libcanard.
 *
 * Autogenerated, do not edit.
 *
 */

#ifndef __DSDL_UAVCAN_EQUIPMENT_AHRS_SOLUTION
#define __DSDL_UAVCAN_EQUIPMENT_AHRS_SOLUTION

#include <uavcan_service/libcanard/canard.h>

#ifdef __cplusplus
extern "C"
{
#endif

/******************************* Source text **********************************
#
# Inertial data and orientation in body frame.
#

#uavcan.Timestamp timestamp

#
# Normalized quaternion
#
float16[4] orientation_xyzw
void4
float16[<=9] orientation_covariance

#
# rad/sec
#
float16[3] angular_velocity
void4
float16[<=9] angular_velocity_covariance

#
# m/s^2
#
float16[3] linear_acceleration
float16[<=9] linear_acceleration_covariance
******************************************************************************/

/********************* DSDL signature source definition ***********************
dsdl.uavcan.equipment.ahrs.Solution
saturated float16[4] orientation_xyzw
void4
saturated float16[<=9] orientation_covariance
saturated float16[3] angular_velocity
void4
saturated float16[<=9] angular_velocity_covariance
saturated float16[3] linear_acceleration
saturated float16[<=9] linear_acceleration_covariance
******************************************************************************/

#define DSDL_UAVCAN_EQUIPMENT_AHRS_SOLUTION_ID             1000
#define DSDL_UAVCAN_EQUIPMENT_AHRS_SOLUTION_NAME           "dsdl.uavcan.equipment.ahrs.Solution"
#define DSDL_UAVCAN_EQUIPMENT_AHRS_SOLUTION_SIGNATURE      (0x256B5CB662380BFDULL)

#define DSDL_UAVCAN_EQUIPMENT_AHRS_SOLUTION_MAX_SIZE       ((612 + 7)/8)

// Constants

#define DSDL_UAVCAN_EQUIPMENT_AHRS_SOLUTION_ORIENTATION_XYZW_LENGTH                      4
#define DSDL_UAVCAN_EQUIPMENT_AHRS_SOLUTION_ORIENTATION_COVARIANCE_MAX_LENGTH            9
#define DSDL_UAVCAN_EQUIPMENT_AHRS_SOLUTION_ANGULAR_VELOCITY_LENGTH                      3
#define DSDL_UAVCAN_EQUIPMENT_AHRS_SOLUTION_ANGULAR_VELOCITY_COVARIANCE_MAX_LENGTH       9
#define DSDL_UAVCAN_EQUIPMENT_AHRS_SOLUTION_LINEAR_ACCELERATION_LENGTH                   3
#define DSDL_UAVCAN_EQUIPMENT_AHRS_SOLUTION_LINEAR_ACCELERATION_COVARIANCE_MAX_LENGTH    9

typedef struct
{
    // FieldTypes
    float      orientation_xyzw[4];           // Static Array 16bit[4] max items
    // void4
    struct
    {
        uint8_t    len;                       // Dynamic array length
        float*     data;                      // Dynamic Array 16bit[9] max items
    } orientation_covariance;
    float      angular_velocity[3];           // Static Array 16bit[3] max items
    // void4
    struct
    {
        uint8_t    len;                       // Dynamic array length
        float*     data;                      // Dynamic Array 16bit[9] max items
    } angular_velocity_covariance;
    float      linear_acceleration[3];        // Static Array 16bit[3] max items
    struct
    {
        uint8_t    len;                       // Dynamic array length
        float*     data;                      // Dynamic Array 16bit[9] max items
    } linear_acceleration_covariance;

} dsdl_uavcan_equipment_ahrs_Solution;

static inline
uint32_t dsdl_uavcan_equipment_ahrs_Solution_encode(dsdl_uavcan_equipment_ahrs_Solution* source, void* msg_buf);

static inline
int32_t dsdl_uavcan_equipment_ahrs_Solution_decode(const CanardRxTransfer* transfer, uint16_t payload_len, dsdl_uavcan_equipment_ahrs_Solution* dest, uint8_t** dyn_arr_buf);

static inline
uint32_t dsdl_uavcan_equipment_ahrs_Solution_encode_internal(dsdl_uavcan_equipment_ahrs_Solution* source, void* msg_buf, uint32_t offset, uint8_t root_item);

static inline
int32_t dsdl_uavcan_equipment_ahrs_Solution_decode_internal(const CanardRxTransfer* transfer, uint16_t payload_len, dsdl_uavcan_equipment_ahrs_Solution* dest, uint8_t** dyn_arr_buf, int32_t offset, uint8_t tao);

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
  * @brief dsdl_uavcan_equipment_ahrs_Solution_encode_internal
  * @param source : pointer to source data struct
  * @param msg_buf: pointer to msg storage
  * @param offset: bit offset to msg storage
  * @param root_item: for detecting if TAO should be used
  * @retval returns offset
  */
uint32_t dsdl_uavcan_equipment_ahrs_Solution_encode_internal(dsdl_uavcan_equipment_ahrs_Solution* source,
  void* msg_buf,
  uint32_t offset,
  uint8_t CANARD_MAYBE_UNUSED(root_item))
{
    uint32_t c = 0;

    // Static array (orientation_xyzw)
    for (c = 0; c < 4; c++)
    {
        canardEncodeScalar(msg_buf, offset, 16, (void*)(source->orientation_xyzw + c)); // 32767
        offset += 16;
    }

    // Void4
    offset += 4;

    // Dynamic Array (orientation_covariance)
    // - Add array length
    canardEncodeScalar(msg_buf, offset, 4, (void*)&source->orientation_covariance.len);
    offset += 4;

    // - Add array items
    for (c = 0; c < source->orientation_covariance.len; c++)
    {
        canardEncodeScalar(msg_buf,
                           offset,
                           16,
                           (void*)(source->orientation_covariance.data + c));// 32767
        offset += 16;
    }

    // Static array (angular_velocity)
    for (c = 0; c < 3; c++)
    {
        canardEncodeScalar(msg_buf, offset, 16, (void*)(source->angular_velocity + c)); // 32767
        offset += 16;
    }

    // Void4
    offset += 4;

    // Dynamic Array (angular_velocity_covariance)
    // - Add array length
    canardEncodeScalar(msg_buf, offset, 4, (void*)&source->angular_velocity_covariance.len);
    offset += 4;

    // - Add array items
    for (c = 0; c < source->angular_velocity_covariance.len; c++)
    {
        canardEncodeScalar(msg_buf,
                           offset,
                           16,
                           (void*)(source->angular_velocity_covariance.data + c));// 32767
        offset += 16;
    }

    // Static array (linear_acceleration)
    for (c = 0; c < 3; c++)
    {
        canardEncodeScalar(msg_buf, offset, 16, (void*)(source->linear_acceleration + c)); // 32767
        offset += 16;
    }

    // Dynamic Array (linear_acceleration_covariance)
    if (! root_item)
    {
        // - Add array length
        canardEncodeScalar(msg_buf, offset, 4, (void*)&source->linear_acceleration_covariance.len);
        offset += 4;
    }

    // - Add array items
    for (c = 0; c < source->linear_acceleration_covariance.len; c++)
    {
        canardEncodeScalar(msg_buf,
                           offset,
                           16,
                           (void*)(source->linear_acceleration_covariance.data + c));// 32767
        offset += 16;
    }

    return offset;
}

/**
  * @brief dsdl_uavcan_equipment_ahrs_Solution_encode
  * @param source : Pointer to source data struct
  * @param msg_buf: Pointer to msg storage
  * @retval returns message length as bytes
  */
uint32_t dsdl_uavcan_equipment_ahrs_Solution_encode(dsdl_uavcan_equipment_ahrs_Solution* source, void* msg_buf)
{
    uint32_t offset = 0;

    offset = dsdl_uavcan_equipment_ahrs_Solution_encode_internal(source, msg_buf, offset, 1);

    return (offset + 7 ) / 8;
}

/**
  * @brief dsdl_uavcan_equipment_ahrs_Solution_decode_internal
  * @param transfer: Pointer to CanardRxTransfer transfer
  * @param payload_len: Payload message length
  * @param dest: Pointer to destination struct
  * @param dyn_arr_buf: NULL or Pointer to memory storage to be used for dynamic arrays
  *                     dsdl_uavcan_equipment_ahrs_Solution dyn memory will point to dyn_arr_buf memory.
  *                     NULL will ignore dynamic arrays decoding.
  * @param offset: Call with 0, bit offset to msg storage
  * @param tao: is tail array optimization used
  * @retval offset or ERROR value if < 0
  */
int32_t dsdl_uavcan_equipment_ahrs_Solution_decode_internal(
  const CanardRxTransfer* transfer,
  uint16_t CANARD_MAYBE_UNUSED(payload_len),
  dsdl_uavcan_equipment_ahrs_Solution* dest,
  uint8_t** CANARD_MAYBE_UNUSED(dyn_arr_buf),
  int32_t offset,
  uint8_t CANARD_MAYBE_UNUSED(tao))
{
    int32_t ret = 0;
    uint32_t c = 0;

    // Static array (orientation_xyzw)
    for (c = 0; c < 4; c++)
    {
        ret = canardDecodeScalar(transfer, offset, 16, false, (void*)(dest->orientation_xyzw + c));
        if (ret != 16)
        {
            goto dsdl_uavcan_equipment_ahrs_Solution_error_exit;
        }
        offset += 16;
    }

    // Void4
    offset += 4;

    // Dynamic Array (orientation_covariance)
    //  - Array length, not last item 4 bits
    ret = canardDecodeScalar(transfer,
                             offset,
                             4,
                             false,
                             (void*)&dest->orientation_covariance.len); // 32767
    if (ret != 4)
    {
        goto dsdl_uavcan_equipment_ahrs_Solution_error_exit;
    }
    offset += 4;

    //  - Get Array
    if (dyn_arr_buf)
    {
        dest->orientation_covariance.data = (float*)*dyn_arr_buf;
    }

    for (c = 0; c < dest->orientation_covariance.len; c++)
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
                goto dsdl_uavcan_equipment_ahrs_Solution_error_exit;
            }
            *dyn_arr_buf = (uint8_t*)(((float*)*dyn_arr_buf) + 1);
        }
        offset += 16;
    }

    // Static array (angular_velocity)
    for (c = 0; c < 3; c++)
    {
        ret = canardDecodeScalar(transfer, offset, 16, false, (void*)(dest->angular_velocity + c));
        if (ret != 16)
        {
            goto dsdl_uavcan_equipment_ahrs_Solution_error_exit;
        }
        offset += 16;
    }

    // Void4
    offset += 4;

    // Dynamic Array (angular_velocity_covariance)
    //  - Array length, not last item 4 bits
    ret = canardDecodeScalar(transfer,
                             offset,
                             4,
                             false,
                             (void*)&dest->angular_velocity_covariance.len); // 32767
    if (ret != 4)
    {
        goto dsdl_uavcan_equipment_ahrs_Solution_error_exit;
    }
    offset += 4;

    //  - Get Array
    if (dyn_arr_buf)
    {
        dest->angular_velocity_covariance.data = (float*)*dyn_arr_buf;
    }

    for (c = 0; c < dest->angular_velocity_covariance.len; c++)
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
                goto dsdl_uavcan_equipment_ahrs_Solution_error_exit;
            }
            *dyn_arr_buf = (uint8_t*)(((float*)*dyn_arr_buf) + 1);
        }
        offset += 16;
    }

    // Static array (linear_acceleration)
    for (c = 0; c < 3; c++)
    {
        ret = canardDecodeScalar(transfer, offset, 16, false, (void*)(dest->linear_acceleration + c));
        if (ret != 16)
        {
            goto dsdl_uavcan_equipment_ahrs_Solution_error_exit;
        }
        offset += 16;
    }

    // Dynamic Array (linear_acceleration_covariance)
    //  - Last item in struct & Root item & (Array Size > 8 bit), tail array optimization
    if (payload_len && tao == CANARD_INTERNAL_ENABLE_TAO)
    {
        //  - Calculate Array length from MSG length
        dest->linear_acceleration_covariance.len = ((payload_len * 8) - offset ) / 16; // 16 bit array item size
    }
    else
    {
        // - Array length 4 bits
        ret = canardDecodeScalar(transfer,
                                 offset,
                                 4,
                                 false,
                                 (void*)&dest->linear_acceleration_covariance.len); // 32767
        if (ret != 4)
        {
            goto dsdl_uavcan_equipment_ahrs_Solution_error_exit;
        }
        offset += 4;
    }

    //  - Get Array
    if (dyn_arr_buf)
    {
        dest->linear_acceleration_covariance.data = (float*)*dyn_arr_buf;
    }

    for (c = 0; c < dest->linear_acceleration_covariance.len; c++)
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
                goto dsdl_uavcan_equipment_ahrs_Solution_error_exit;
            }
            *dyn_arr_buf = (uint8_t*)(((float*)*dyn_arr_buf) + 1);
        }
        offset += 16;
    }
    return offset;

dsdl_uavcan_equipment_ahrs_Solution_error_exit:
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
  * @brief dsdl_uavcan_equipment_ahrs_Solution_decode
  * @param transfer: Pointer to CanardRxTransfer transfer
  * @param payload_len: Payload message length
  * @param dest: Pointer to destination struct
  * @param dyn_arr_buf: NULL or Pointer to memory storage to be used for dynamic arrays
  *                     dsdl_uavcan_equipment_ahrs_Solution dyn memory will point to dyn_arr_buf memory.
  *                     NULL will ignore dynamic arrays decoding.
  * @retval offset or ERROR value if < 0
  */
int32_t dsdl_uavcan_equipment_ahrs_Solution_decode(const CanardRxTransfer* transfer,
  uint16_t payload_len,
  dsdl_uavcan_equipment_ahrs_Solution* dest,
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
        for (uint32_t c = 0; c < sizeof(dsdl_uavcan_equipment_ahrs_Solution); c++)
        {
            ((uint8_t*)dest)[c] = 0x00;
        }

        ret = dsdl_uavcan_equipment_ahrs_Solution_decode_internal(transfer, payload_len, dest, dyn_arr_buf, offset, tao);

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
#endif // __DSDL_UAVCAN_EQUIPMENT_AHRS_SOLUTION