/*
 * UAVCAN data structure definition for libcanard.
 *
 * Autogenerated, do not edit.
 *
 */

#ifndef __DSDL_UAVCAN_EQUIPMENT_AHRS_FUSIONIMU
#define __DSDL_UAVCAN_EQUIPMENT_AHRS_FUSIONIMU

#include <uavcan_service/libcanard/canard.h>

#ifdef __cplusplus
extern "C"
{
#endif

/******************************* Source text **********************************
#
# Fusion IMU data.
#
# THIS DEFINITION MAY BE CHANGED IN A NON-BACKWARD-COMPATIBLE WAY IN THE FUTURE.
#

#
# State of fusion.
# The state of fusion in the body frame, the status are ordered as follows:
#   0x00,OK
#   0x01,ERROR
#
int8 state                 # Status of fusion
#
# Angle fusion in degree.
# The samples are represented in the body frame, the axes are ordered as follows:
#   1. angle around X (roll)
#   2. angle around Y (pitch)
#   3. angle around Z (yaw)
#
float32 fusion_roll		    # Integrated fusion, degree
float32 fusion_pitch		# Integrated fusion, degree
float32 fusion_yaw	        # Integrated fusion, degree

#
# Angular speed fusion in degree/second.
# The samples are represented in the body frame, the axes are ordered as follows:
#   1. angular velocity around X (roll rate)
#   2. angular velocity around Y (pitch rate)
#   3. angular velocity around Z (yaw rate)
#
float32 fusion_roll_velocity		# Integrated fusion, degree/second
float32 fusion_pitch_velocity		# Integrated fusion, degree/second
float32 fusion_yaw_velocity	        # Integrated fusion, degree/second

#
# quaternion.
# The quaternion are represented in the body frame.
#
float32[4] q           # Integrated quaternion
******************************************************************************/

/********************* DSDL signature source definition ***********************
dsdl.uavcan.equipment.ahrs.FusionIMU
saturated int8 state
saturated float32 fusion_roll
saturated float32 fusion_pitch
saturated float32 fusion_yaw
saturated float32 fusion_roll_velocity
saturated float32 fusion_pitch_velocity
saturated float32 fusion_yaw_velocity
saturated float32[4] q
******************************************************************************/

#define DSDL_UAVCAN_EQUIPMENT_AHRS_FUSIONIMU_ID            1004
#define DSDL_UAVCAN_EQUIPMENT_AHRS_FUSIONIMU_NAME          "dsdl.uavcan.equipment.ahrs.FusionIMU"
#define DSDL_UAVCAN_EQUIPMENT_AHRS_FUSIONIMU_SIGNATURE     (0x302806DA73333DE4ULL)

#define DSDL_UAVCAN_EQUIPMENT_AHRS_FUSIONIMU_MAX_SIZE      ((328 + 7)/8)

// Constants

#define DSDL_UAVCAN_EQUIPMENT_AHRS_FUSIONIMU_Q_LENGTH                                    4

typedef struct
{
    // FieldTypes
    int8_t     state;                         // bit len 8
    float      fusion_roll;                   // float32 Saturate
    float      fusion_pitch;                  // float32 Saturate
    float      fusion_yaw;                    // float32 Saturate
    float      fusion_roll_velocity;          // float32 Saturate
    float      fusion_pitch_velocity;         // float32 Saturate
    float      fusion_yaw_velocity;           // float32 Saturate
    float      q[4];                          // Static Array 32bit[4] max items

} dsdl_uavcan_equipment_ahrs_FusionIMU;

static inline
uint32_t dsdl_uavcan_equipment_ahrs_FusionIMU_encode(dsdl_uavcan_equipment_ahrs_FusionIMU* source, void* msg_buf);

static inline
int32_t dsdl_uavcan_equipment_ahrs_FusionIMU_decode(const CanardRxTransfer* transfer, uint16_t payload_len, dsdl_uavcan_equipment_ahrs_FusionIMU* dest, uint8_t** dyn_arr_buf);

static inline
uint32_t dsdl_uavcan_equipment_ahrs_FusionIMU_encode_internal(dsdl_uavcan_equipment_ahrs_FusionIMU* source, void* msg_buf, uint32_t offset, uint8_t root_item);

static inline
int32_t dsdl_uavcan_equipment_ahrs_FusionIMU_decode_internal(const CanardRxTransfer* transfer, uint16_t payload_len, dsdl_uavcan_equipment_ahrs_FusionIMU* dest, uint8_t** dyn_arr_buf, int32_t offset, uint8_t tao);

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
  * @brief dsdl_uavcan_equipment_ahrs_FusionIMU_encode_internal
  * @param source : pointer to source data struct
  * @param msg_buf: pointer to msg storage
  * @param offset: bit offset to msg storage
  * @param root_item: for detecting if TAO should be used
  * @retval returns offset
  */
uint32_t dsdl_uavcan_equipment_ahrs_FusionIMU_encode_internal(dsdl_uavcan_equipment_ahrs_FusionIMU* source,
  void* msg_buf,
  uint32_t offset,
  uint8_t CANARD_MAYBE_UNUSED(root_item))
{
    uint32_t c = 0;

    canardEncodeScalar(msg_buf, offset, 8, (void*)&source->state); // 127
    offset += 8;

    canardEncodeScalar(msg_buf, offset, 32, (void*)&source->fusion_roll); // 2147483647
    offset += 32;

    canardEncodeScalar(msg_buf, offset, 32, (void*)&source->fusion_pitch); // 2147483647
    offset += 32;

    canardEncodeScalar(msg_buf, offset, 32, (void*)&source->fusion_yaw); // 2147483647
    offset += 32;

    canardEncodeScalar(msg_buf, offset, 32, (void*)&source->fusion_roll_velocity); // 2147483647
    offset += 32;

    canardEncodeScalar(msg_buf, offset, 32, (void*)&source->fusion_pitch_velocity); // 2147483647
    offset += 32;

    canardEncodeScalar(msg_buf, offset, 32, (void*)&source->fusion_yaw_velocity); // 2147483647
    offset += 32;

    // Static array (q)
    for (c = 0; c < 4; c++)
    {
        canardEncodeScalar(msg_buf, offset, 32, (void*)(source->q + c)); // 2147483647
        offset += 32;
    }

    return offset;
}

/**
  * @brief dsdl_uavcan_equipment_ahrs_FusionIMU_encode
  * @param source : Pointer to source data struct
  * @param msg_buf: Pointer to msg storage
  * @retval returns message length as bytes
  */
uint32_t dsdl_uavcan_equipment_ahrs_FusionIMU_encode(dsdl_uavcan_equipment_ahrs_FusionIMU* source, void* msg_buf)
{
    uint32_t offset = 0;

    offset = dsdl_uavcan_equipment_ahrs_FusionIMU_encode_internal(source, msg_buf, offset, 1);

    return (offset + 7 ) / 8;
}

/**
  * @brief dsdl_uavcan_equipment_ahrs_FusionIMU_decode_internal
  * @param transfer: Pointer to CanardRxTransfer transfer
  * @param payload_len: Payload message length
  * @param dest: Pointer to destination struct
  * @param dyn_arr_buf: NULL or Pointer to memory storage to be used for dynamic arrays
  *                     dsdl_uavcan_equipment_ahrs_FusionIMU dyn memory will point to dyn_arr_buf memory.
  *                     NULL will ignore dynamic arrays decoding.
  * @param offset: Call with 0, bit offset to msg storage
  * @param tao: is tail array optimization used
  * @retval offset or ERROR value if < 0
  */
int32_t dsdl_uavcan_equipment_ahrs_FusionIMU_decode_internal(
  const CanardRxTransfer* transfer,
  uint16_t CANARD_MAYBE_UNUSED(payload_len),
  dsdl_uavcan_equipment_ahrs_FusionIMU* dest,
  uint8_t** CANARD_MAYBE_UNUSED(dyn_arr_buf),
  int32_t offset,
  uint8_t CANARD_MAYBE_UNUSED(tao))
{
    int32_t ret = 0;
    uint32_t c = 0;

    ret = canardDecodeScalar(transfer, offset, 8, true, (void*)&dest->state);
    if (ret != 8)
    {
        goto dsdl_uavcan_equipment_ahrs_FusionIMU_error_exit;
    }
    offset += 8;

    ret = canardDecodeScalar(transfer, offset, 32, false, (void*)&dest->fusion_roll);
    if (ret != 32)
    {
        goto dsdl_uavcan_equipment_ahrs_FusionIMU_error_exit;
    }
    offset += 32;

    ret = canardDecodeScalar(transfer, offset, 32, false, (void*)&dest->fusion_pitch);
    if (ret != 32)
    {
        goto dsdl_uavcan_equipment_ahrs_FusionIMU_error_exit;
    }
    offset += 32;

    ret = canardDecodeScalar(transfer, offset, 32, false, (void*)&dest->fusion_yaw);
    if (ret != 32)
    {
        goto dsdl_uavcan_equipment_ahrs_FusionIMU_error_exit;
    }
    offset += 32;

    ret = canardDecodeScalar(transfer, offset, 32, false, (void*)&dest->fusion_roll_velocity);
    if (ret != 32)
    {
        goto dsdl_uavcan_equipment_ahrs_FusionIMU_error_exit;
    }
    offset += 32;

    ret = canardDecodeScalar(transfer, offset, 32, false, (void*)&dest->fusion_pitch_velocity);
    if (ret != 32)
    {
        goto dsdl_uavcan_equipment_ahrs_FusionIMU_error_exit;
    }
    offset += 32;

    ret = canardDecodeScalar(transfer, offset, 32, false, (void*)&dest->fusion_yaw_velocity);
    if (ret != 32)
    {
        goto dsdl_uavcan_equipment_ahrs_FusionIMU_error_exit;
    }
    offset += 32;

    // Static array (q)
    for (c = 0; c < 4; c++)
    {
        ret = canardDecodeScalar(transfer, offset, 32, false, (void*)(dest->q + c));
        if (ret != 32)
        {
            goto dsdl_uavcan_equipment_ahrs_FusionIMU_error_exit;
        }
        offset += 32;
    }
    return offset;

dsdl_uavcan_equipment_ahrs_FusionIMU_error_exit:
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
  * @brief dsdl_uavcan_equipment_ahrs_FusionIMU_decode
  * @param transfer: Pointer to CanardRxTransfer transfer
  * @param payload_len: Payload message length
  * @param dest: Pointer to destination struct
  * @param dyn_arr_buf: NULL or Pointer to memory storage to be used for dynamic arrays
  *                     dsdl_uavcan_equipment_ahrs_FusionIMU dyn memory will point to dyn_arr_buf memory.
  *                     NULL will ignore dynamic arrays decoding.
  * @retval offset or ERROR value if < 0
  */
int32_t dsdl_uavcan_equipment_ahrs_FusionIMU_decode(const CanardRxTransfer* transfer,
  uint16_t payload_len,
  dsdl_uavcan_equipment_ahrs_FusionIMU* dest,
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
        for (uint32_t c = 0; c < sizeof(dsdl_uavcan_equipment_ahrs_FusionIMU); c++)
        {
            ((uint8_t*)dest)[c] = 0x00;
        }

        ret = dsdl_uavcan_equipment_ahrs_FusionIMU_decode_internal(transfer, payload_len, dest, dyn_arr_buf, offset, tao);

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
#endif // __DSDL_UAVCAN_EQUIPMENT_AHRS_FUSIONIMU