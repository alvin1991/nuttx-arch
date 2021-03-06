/*
 * UAVCAN data structure definition for libcanard.
 *
 * Autogenerated, do not edit.
 *
 */

#ifndef __DSDL_UAVCAN_EQUIPMENT_AIR_DATA_INDICATEDAIRSPEED
#define __DSDL_UAVCAN_EQUIPMENT_AIR_DATA_INDICATEDAIRSPEED

#include <uavcan_service/libcanard/canard.h>

#ifdef __cplusplus
extern "C"
{
#endif

/******************************* Source text **********************************
#
# IAS.
#

float16 indicated_airspeed              # m/s
float16 indicated_airspeed_variance     # (m/s)^2
******************************************************************************/

/********************* DSDL signature source definition ***********************
dsdl.uavcan.equipment.air_data.IndicatedAirspeed
saturated float16 indicated_airspeed
saturated float16 indicated_airspeed_variance
******************************************************************************/

#define DSDL_UAVCAN_EQUIPMENT_AIR_DATA_INDICATEDAIRSPEED_ID 1021
#define DSDL_UAVCAN_EQUIPMENT_AIR_DATA_INDICATEDAIRSPEED_NAME "dsdl.uavcan.equipment.air_data.IndicatedAirspeed"
#define DSDL_UAVCAN_EQUIPMENT_AIR_DATA_INDICATEDAIRSPEED_SIGNATURE (0xD1A5C7C4AAA3713ULL)

#define DSDL_UAVCAN_EQUIPMENT_AIR_DATA_INDICATEDAIRSPEED_MAX_SIZE ((32 + 7)/8)

// Constants

typedef struct
{
    // FieldTypes
    float      indicated_airspeed;            // float16 Saturate
    float      indicated_airspeed_variance;   // float16 Saturate

} dsdl_uavcan_equipment_air_data_IndicatedAirspeed;

static inline
uint32_t dsdl_uavcan_equipment_air_data_IndicatedAirspeed_encode(dsdl_uavcan_equipment_air_data_IndicatedAirspeed* source, void* msg_buf);

static inline
int32_t dsdl_uavcan_equipment_air_data_IndicatedAirspeed_decode(const CanardRxTransfer* transfer, uint16_t payload_len, dsdl_uavcan_equipment_air_data_IndicatedAirspeed* dest, uint8_t** dyn_arr_buf);

static inline
uint32_t dsdl_uavcan_equipment_air_data_IndicatedAirspeed_encode_internal(dsdl_uavcan_equipment_air_data_IndicatedAirspeed* source, void* msg_buf, uint32_t offset, uint8_t root_item);

static inline
int32_t dsdl_uavcan_equipment_air_data_IndicatedAirspeed_decode_internal(const CanardRxTransfer* transfer, uint16_t payload_len, dsdl_uavcan_equipment_air_data_IndicatedAirspeed* dest, uint8_t** dyn_arr_buf, int32_t offset, uint8_t tao);

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
  * @brief dsdl_uavcan_equipment_air_data_IndicatedAirspeed_encode_internal
  * @param source : pointer to source data struct
  * @param msg_buf: pointer to msg storage
  * @param offset: bit offset to msg storage
  * @param root_item: for detecting if TAO should be used
  * @retval returns offset
  */
uint32_t dsdl_uavcan_equipment_air_data_IndicatedAirspeed_encode_internal(dsdl_uavcan_equipment_air_data_IndicatedAirspeed* source,
  void* msg_buf,
  uint32_t offset,
  uint8_t CANARD_MAYBE_UNUSED(root_item))
{
#ifndef CANARD_USE_FLOAT16_CAST
    uint16_t tmp_float = 0;
#else
    CANARD_USE_FLOAT16_CAST tmp_float = 0;
#endif

    // float16 special handling
#ifndef CANARD_USE_FLOAT16_CAST
    tmp_float = canardConvertNativeFloatToFloat16(source->indicated_airspeed);
#else
    tmp_float = (CANARD_USE_FLOAT16_CAST)source->indicated_airspeed;
#endif
    canardEncodeScalar(msg_buf, offset, 16, (void*)&tmp_float); // 32767
    offset += 16;

    // float16 special handling
#ifndef CANARD_USE_FLOAT16_CAST
    tmp_float = canardConvertNativeFloatToFloat16(source->indicated_airspeed_variance);
#else
    tmp_float = (CANARD_USE_FLOAT16_CAST)source->indicated_airspeed_variance;
#endif
    canardEncodeScalar(msg_buf, offset, 16, (void*)&tmp_float); // 32767
    offset += 16;

    return offset;
}

/**
  * @brief dsdl_uavcan_equipment_air_data_IndicatedAirspeed_encode
  * @param source : Pointer to source data struct
  * @param msg_buf: Pointer to msg storage
  * @retval returns message length as bytes
  */
uint32_t dsdl_uavcan_equipment_air_data_IndicatedAirspeed_encode(dsdl_uavcan_equipment_air_data_IndicatedAirspeed* source, void* msg_buf)
{
    uint32_t offset = 0;

    offset = dsdl_uavcan_equipment_air_data_IndicatedAirspeed_encode_internal(source, msg_buf, offset, 1);

    return (offset + 7 ) / 8;
}

/**
  * @brief dsdl_uavcan_equipment_air_data_IndicatedAirspeed_decode_internal
  * @param transfer: Pointer to CanardRxTransfer transfer
  * @param payload_len: Payload message length
  * @param dest: Pointer to destination struct
  * @param dyn_arr_buf: NULL or Pointer to memory storage to be used for dynamic arrays
  *                     dsdl_uavcan_equipment_air_data_IndicatedAirspeed dyn memory will point to dyn_arr_buf memory.
  *                     NULL will ignore dynamic arrays decoding.
  * @param offset: Call with 0, bit offset to msg storage
  * @param tao: is tail array optimization used
  * @retval offset or ERROR value if < 0
  */
int32_t dsdl_uavcan_equipment_air_data_IndicatedAirspeed_decode_internal(
  const CanardRxTransfer* transfer,
  uint16_t CANARD_MAYBE_UNUSED(payload_len),
  dsdl_uavcan_equipment_air_data_IndicatedAirspeed* dest,
  uint8_t** CANARD_MAYBE_UNUSED(dyn_arr_buf),
  int32_t offset,
  uint8_t CANARD_MAYBE_UNUSED(tao))
{
    int32_t ret = 0;
#ifndef CANARD_USE_FLOAT16_CAST
    uint16_t tmp_float = 0;
#else
    CANARD_USE_FLOAT16_CAST tmp_float = 0;
#endif

    // float16 special handling
    ret = canardDecodeScalar(transfer, offset, 16, false, (void*)&tmp_float);

    if (ret != 16)
    {
        goto dsdl_uavcan_equipment_air_data_IndicatedAirspeed_error_exit;
    }
#ifndef CANARD_USE_FLOAT16_CAST
    dest->indicated_airspeed = canardConvertFloat16ToNativeFloat(tmp_float);
#else
    dest->indicated_airspeed = (float)tmp_float;
#endif
    offset += 16;

    // float16 special handling
    ret = canardDecodeScalar(transfer, offset, 16, false, (void*)&tmp_float);

    if (ret != 16)
    {
        goto dsdl_uavcan_equipment_air_data_IndicatedAirspeed_error_exit;
    }
#ifndef CANARD_USE_FLOAT16_CAST
    dest->indicated_airspeed_variance = canardConvertFloat16ToNativeFloat(tmp_float);
#else
    dest->indicated_airspeed_variance = (float)tmp_float;
#endif
    offset += 16;
    return offset;

dsdl_uavcan_equipment_air_data_IndicatedAirspeed_error_exit:
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
  * @brief dsdl_uavcan_equipment_air_data_IndicatedAirspeed_decode
  * @param transfer: Pointer to CanardRxTransfer transfer
  * @param payload_len: Payload message length
  * @param dest: Pointer to destination struct
  * @param dyn_arr_buf: NULL or Pointer to memory storage to be used for dynamic arrays
  *                     dsdl_uavcan_equipment_air_data_IndicatedAirspeed dyn memory will point to dyn_arr_buf memory.
  *                     NULL will ignore dynamic arrays decoding.
  * @retval offset or ERROR value if < 0
  */
int32_t dsdl_uavcan_equipment_air_data_IndicatedAirspeed_decode(const CanardRxTransfer* transfer,
  uint16_t payload_len,
  dsdl_uavcan_equipment_air_data_IndicatedAirspeed* dest,
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
        for (uint32_t c = 0; c < sizeof(dsdl_uavcan_equipment_air_data_IndicatedAirspeed); c++)
        {
            ((uint8_t*)dest)[c] = 0x00;
        }

        ret = dsdl_uavcan_equipment_air_data_IndicatedAirspeed_decode_internal(transfer, payload_len, dest, dyn_arr_buf, offset, tao);

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
#endif // __DSDL_UAVCAN_EQUIPMENT_AIR_DATA_INDICATEDAIRSPEED