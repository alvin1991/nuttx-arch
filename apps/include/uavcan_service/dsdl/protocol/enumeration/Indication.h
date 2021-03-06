/*
 * UAVCAN data structure definition for libcanard.
 *
 * Autogenerated, do not edit.
 *
 */

#ifndef __DSDL_UAVCAN_PROTOCOL_ENUMERATION_INDICATION
#define __DSDL_UAVCAN_PROTOCOL_ENUMERATION_INDICATION

#include <uavcan_service/libcanard/canard.h>

#ifdef __cplusplus
extern "C"
{
#endif

/******************************* Source text **********************************
#
# This message will be broadcasted when the node receives user input in the process of enumeration.
#

void6

#
# This field is unused; keep it empty
#
#uavcan.protocol.param.NumericValue value

#
# Name of the enumerated parameter.
# This field must always be populated by the enumeratee.
# If multiple parameters were enumerated at once (e.g. ESC index and the direction of rotation),
# the field should contain the name of the most important parameter.
#
uint8[<=92] parameter_name
******************************************************************************/

/********************* DSDL signature source definition ***********************
dsdl.uavcan.protocol.enumeration.Indication
void6
saturated uint8[<=92] parameter_name
******************************************************************************/

#define DSDL_UAVCAN_PROTOCOL_ENUMERATION_INDICATION_ID     380
#define DSDL_UAVCAN_PROTOCOL_ENUMERATION_INDICATION_NAME   "dsdl.uavcan.protocol.enumeration.Indication"
#define DSDL_UAVCAN_PROTOCOL_ENUMERATION_INDICATION_SIGNATURE (0xFCDB0B45BE37CE5CULL)

#define DSDL_UAVCAN_PROTOCOL_ENUMERATION_INDICATION_MAX_SIZE ((749 + 7)/8)

// Constants

#define DSDL_UAVCAN_PROTOCOL_ENUMERATION_INDICATION_PARAMETER_NAME_MAX_LENGTH            92

typedef struct
{
    // FieldTypes
    // void6
    struct
    {
        uint8_t    len;                       // Dynamic array length
        uint8_t*   data;                      // Dynamic Array 8bit[92] max items
    } parameter_name;

} dsdl_uavcan_protocol_enumeration_Indication;

static inline
uint32_t dsdl_uavcan_protocol_enumeration_Indication_encode(dsdl_uavcan_protocol_enumeration_Indication* source, void* msg_buf);

static inline
int32_t dsdl_uavcan_protocol_enumeration_Indication_decode(const CanardRxTransfer* transfer, uint16_t payload_len, dsdl_uavcan_protocol_enumeration_Indication* dest, uint8_t** dyn_arr_buf);

static inline
uint32_t dsdl_uavcan_protocol_enumeration_Indication_encode_internal(dsdl_uavcan_protocol_enumeration_Indication* source, void* msg_buf, uint32_t offset, uint8_t root_item);

static inline
int32_t dsdl_uavcan_protocol_enumeration_Indication_decode_internal(const CanardRxTransfer* transfer, uint16_t payload_len, dsdl_uavcan_protocol_enumeration_Indication* dest, uint8_t** dyn_arr_buf, int32_t offset, uint8_t tao);

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
  * @brief dsdl_uavcan_protocol_enumeration_Indication_encode_internal
  * @param source : pointer to source data struct
  * @param msg_buf: pointer to msg storage
  * @param offset: bit offset to msg storage
  * @param root_item: for detecting if TAO should be used
  * @retval returns offset
  */
uint32_t dsdl_uavcan_protocol_enumeration_Indication_encode_internal(dsdl_uavcan_protocol_enumeration_Indication* source,
  void* msg_buf,
  uint32_t offset,
  uint8_t CANARD_MAYBE_UNUSED(root_item))
{
    uint32_t c = 0;

    // Void6
    offset += 6;

    // Dynamic Array (parameter_name)
    if (! root_item)
    {
        // - Add array length
        canardEncodeScalar(msg_buf, offset, 7, (void*)&source->parameter_name.len);
        offset += 7;
    }

    // - Add array items
    for (c = 0; c < source->parameter_name.len; c++)
    {
        canardEncodeScalar(msg_buf,
                           offset,
                           8,
                           (void*)(source->parameter_name.data + c));// 255
        offset += 8;
    }

    return offset;
}

/**
  * @brief dsdl_uavcan_protocol_enumeration_Indication_encode
  * @param source : Pointer to source data struct
  * @param msg_buf: Pointer to msg storage
  * @retval returns message length as bytes
  */
uint32_t dsdl_uavcan_protocol_enumeration_Indication_encode(dsdl_uavcan_protocol_enumeration_Indication* source, void* msg_buf)
{
    uint32_t offset = 0;

    offset = dsdl_uavcan_protocol_enumeration_Indication_encode_internal(source, msg_buf, offset, 1);

    return (offset + 7 ) / 8;
}

/**
  * @brief dsdl_uavcan_protocol_enumeration_Indication_decode_internal
  * @param transfer: Pointer to CanardRxTransfer transfer
  * @param payload_len: Payload message length
  * @param dest: Pointer to destination struct
  * @param dyn_arr_buf: NULL or Pointer to memory storage to be used for dynamic arrays
  *                     dsdl_uavcan_protocol_enumeration_Indication dyn memory will point to dyn_arr_buf memory.
  *                     NULL will ignore dynamic arrays decoding.
  * @param offset: Call with 0, bit offset to msg storage
  * @param tao: is tail array optimization used
  * @retval offset or ERROR value if < 0
  */
int32_t dsdl_uavcan_protocol_enumeration_Indication_decode_internal(
  const CanardRxTransfer* transfer,
  uint16_t CANARD_MAYBE_UNUSED(payload_len),
  dsdl_uavcan_protocol_enumeration_Indication* dest,
  uint8_t** CANARD_MAYBE_UNUSED(dyn_arr_buf),
  int32_t offset,
  uint8_t CANARD_MAYBE_UNUSED(tao))
{
    int32_t ret = 0;
    uint32_t c = 0;

    // Void6
    offset += 6;

    // Dynamic Array (parameter_name)
    //  - Last item in struct & Root item & (Array Size > 8 bit), tail array optimization
    if (payload_len && tao == CANARD_INTERNAL_ENABLE_TAO)
    {
        //  - Calculate Array length from MSG length
        dest->parameter_name.len = ((payload_len * 8) - offset ) / 8; // 8 bit array item size
    }
    else
    {
        // - Array length 7 bits
        ret = canardDecodeScalar(transfer,
                                 offset,
                                 7,
                                 false,
                                 (void*)&dest->parameter_name.len); // 255
        if (ret != 7)
        {
            goto dsdl_uavcan_protocol_enumeration_Indication_error_exit;
        }
        offset += 7;
    }

    //  - Get Array
    if (dyn_arr_buf)
    {
        dest->parameter_name.data = (uint8_t*)*dyn_arr_buf;
    }

    for (c = 0; c < dest->parameter_name.len; c++)
    {
        if (dyn_arr_buf)
        {
            ret = canardDecodeScalar(transfer,
                                     offset,
                                     8,
                                     false,
                                     (void*)*dyn_arr_buf); // 255
            if (ret != 8)
            {
                goto dsdl_uavcan_protocol_enumeration_Indication_error_exit;
            }
            *dyn_arr_buf = (uint8_t*)(((uint8_t*)*dyn_arr_buf) + 1);
        }
        offset += 8;
    }
    return offset;

dsdl_uavcan_protocol_enumeration_Indication_error_exit:
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
  * @brief dsdl_uavcan_protocol_enumeration_Indication_decode
  * @param transfer: Pointer to CanardRxTransfer transfer
  * @param payload_len: Payload message length
  * @param dest: Pointer to destination struct
  * @param dyn_arr_buf: NULL or Pointer to memory storage to be used for dynamic arrays
  *                     dsdl_uavcan_protocol_enumeration_Indication dyn memory will point to dyn_arr_buf memory.
  *                     NULL will ignore dynamic arrays decoding.
  * @retval offset or ERROR value if < 0
  */
int32_t dsdl_uavcan_protocol_enumeration_Indication_decode(const CanardRxTransfer* transfer,
  uint16_t payload_len,
  dsdl_uavcan_protocol_enumeration_Indication* dest,
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
        for (uint32_t c = 0; c < sizeof(dsdl_uavcan_protocol_enumeration_Indication); c++)
        {
            ((uint8_t*)dest)[c] = 0x00;
        }

        ret = dsdl_uavcan_protocol_enumeration_Indication_decode_internal(transfer, payload_len, dest, dyn_arr_buf, offset, tao);

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
#endif // __DSDL_UAVCAN_PROTOCOL_ENUMERATION_INDICATION