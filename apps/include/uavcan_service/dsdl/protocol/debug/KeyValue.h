/*
 * UAVCAN data structure definition for libcanard.
 *
 * Autogenerated, do not edit.
 *
 */

#ifndef __DSDL_UAVCAN_PROTOCOL_DEBUG_KEYVALUE
#define __DSDL_UAVCAN_PROTOCOL_DEBUG_KEYVALUE

#include <uavcan_service/libcanard/canard.h>

#ifdef __cplusplus
extern "C"
{
#endif

/******************************* Source text **********************************
#
# Generic named parameter (key/value pair).
#

#
# Integers are exactly representable in the range (-2^24, 2^24) which is (-16'777'216, 16'777'216).
#
float32 value

#
# Tail array optimization is enabled, so if key length does not exceed 3 characters, the whole
# message can fit into one CAN frame. The message always fits into one CAN FD frame.
#
uint8[<=58] key
******************************************************************************/

/********************* DSDL signature source definition ***********************
dsdl.uavcan.protocol.debug.KeyValue
saturated float32 value
saturated uint8[<=58] key
******************************************************************************/

#define DSDL_UAVCAN_PROTOCOL_DEBUG_KEYVALUE_ID             16370
#define DSDL_UAVCAN_PROTOCOL_DEBUG_KEYVALUE_NAME           "dsdl.uavcan.protocol.debug.KeyValue"
#define DSDL_UAVCAN_PROTOCOL_DEBUG_KEYVALUE_SIGNATURE      (0x65A495C3B9F1DBB5ULL)

#define DSDL_UAVCAN_PROTOCOL_DEBUG_KEYVALUE_MAX_SIZE       ((502 + 7)/8)

// Constants

#define DSDL_UAVCAN_PROTOCOL_DEBUG_KEYVALUE_KEY_MAX_LENGTH                               58

typedef struct
{
    // FieldTypes
    float      value;                         // float32 Saturate
    struct
    {
        uint8_t    len;                       // Dynamic array length
        uint8_t*   data;                      // Dynamic Array 8bit[58] max items
    } key;

} dsdl_uavcan_protocol_debug_KeyValue;

static inline
uint32_t dsdl_uavcan_protocol_debug_KeyValue_encode(dsdl_uavcan_protocol_debug_KeyValue* source, void* msg_buf);

static inline
int32_t dsdl_uavcan_protocol_debug_KeyValue_decode(const CanardRxTransfer* transfer, uint16_t payload_len, dsdl_uavcan_protocol_debug_KeyValue* dest, uint8_t** dyn_arr_buf);

static inline
uint32_t dsdl_uavcan_protocol_debug_KeyValue_encode_internal(dsdl_uavcan_protocol_debug_KeyValue* source, void* msg_buf, uint32_t offset, uint8_t root_item);

static inline
int32_t dsdl_uavcan_protocol_debug_KeyValue_decode_internal(const CanardRxTransfer* transfer, uint16_t payload_len, dsdl_uavcan_protocol_debug_KeyValue* dest, uint8_t** dyn_arr_buf, int32_t offset, uint8_t tao);

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
  * @brief dsdl_uavcan_protocol_debug_KeyValue_encode_internal
  * @param source : pointer to source data struct
  * @param msg_buf: pointer to msg storage
  * @param offset: bit offset to msg storage
  * @param root_item: for detecting if TAO should be used
  * @retval returns offset
  */
uint32_t dsdl_uavcan_protocol_debug_KeyValue_encode_internal(dsdl_uavcan_protocol_debug_KeyValue* source,
  void* msg_buf,
  uint32_t offset,
  uint8_t CANARD_MAYBE_UNUSED(root_item))
{
    uint32_t c = 0;

    canardEncodeScalar(msg_buf, offset, 32, (void*)&source->value); // 2147483647
    offset += 32;

    // Dynamic Array (key)
    if (! root_item)
    {
        // - Add array length
        canardEncodeScalar(msg_buf, offset, 6, (void*)&source->key.len);
        offset += 6;
    }

    // - Add array items
    for (c = 0; c < source->key.len; c++)
    {
        canardEncodeScalar(msg_buf,
                           offset,
                           8,
                           (void*)(source->key.data + c));// 255
        offset += 8;
    }

    return offset;
}

/**
  * @brief dsdl_uavcan_protocol_debug_KeyValue_encode
  * @param source : Pointer to source data struct
  * @param msg_buf: Pointer to msg storage
  * @retval returns message length as bytes
  */
uint32_t dsdl_uavcan_protocol_debug_KeyValue_encode(dsdl_uavcan_protocol_debug_KeyValue* source, void* msg_buf)
{
    uint32_t offset = 0;

    offset = dsdl_uavcan_protocol_debug_KeyValue_encode_internal(source, msg_buf, offset, 1);

    return (offset + 7 ) / 8;
}

/**
  * @brief dsdl_uavcan_protocol_debug_KeyValue_decode_internal
  * @param transfer: Pointer to CanardRxTransfer transfer
  * @param payload_len: Payload message length
  * @param dest: Pointer to destination struct
  * @param dyn_arr_buf: NULL or Pointer to memory storage to be used for dynamic arrays
  *                     dsdl_uavcan_protocol_debug_KeyValue dyn memory will point to dyn_arr_buf memory.
  *                     NULL will ignore dynamic arrays decoding.
  * @param offset: Call with 0, bit offset to msg storage
  * @param tao: is tail array optimization used
  * @retval offset or ERROR value if < 0
  */
int32_t dsdl_uavcan_protocol_debug_KeyValue_decode_internal(
  const CanardRxTransfer* transfer,
  uint16_t CANARD_MAYBE_UNUSED(payload_len),
  dsdl_uavcan_protocol_debug_KeyValue* dest,
  uint8_t** CANARD_MAYBE_UNUSED(dyn_arr_buf),
  int32_t offset,
  uint8_t CANARD_MAYBE_UNUSED(tao))
{
    int32_t ret = 0;
    uint32_t c = 0;

    ret = canardDecodeScalar(transfer, offset, 32, false, (void*)&dest->value);
    if (ret != 32)
    {
        goto dsdl_uavcan_protocol_debug_KeyValue_error_exit;
    }
    offset += 32;

    // Dynamic Array (key)
    //  - Last item in struct & Root item & (Array Size > 8 bit), tail array optimization
    if (payload_len && tao == CANARD_INTERNAL_ENABLE_TAO)
    {
        //  - Calculate Array length from MSG length
        dest->key.len = ((payload_len * 8) - offset ) / 8; // 8 bit array item size
    }
    else
    {
        // - Array length 6 bits
        ret = canardDecodeScalar(transfer,
                                 offset,
                                 6,
                                 false,
                                 (void*)&dest->key.len); // 255
        if (ret != 6)
        {
            goto dsdl_uavcan_protocol_debug_KeyValue_error_exit;
        }
        offset += 6;
    }

    //  - Get Array
    if (dyn_arr_buf)
    {
        dest->key.data = (uint8_t*)*dyn_arr_buf;
    }

    for (c = 0; c < dest->key.len; c++)
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
                goto dsdl_uavcan_protocol_debug_KeyValue_error_exit;
            }
            *dyn_arr_buf = (uint8_t*)(((uint8_t*)*dyn_arr_buf) + 1);
        }
        offset += 8;
    }
    return offset;

dsdl_uavcan_protocol_debug_KeyValue_error_exit:
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
  * @brief dsdl_uavcan_protocol_debug_KeyValue_decode
  * @param transfer: Pointer to CanardRxTransfer transfer
  * @param payload_len: Payload message length
  * @param dest: Pointer to destination struct
  * @param dyn_arr_buf: NULL or Pointer to memory storage to be used for dynamic arrays
  *                     dsdl_uavcan_protocol_debug_KeyValue dyn memory will point to dyn_arr_buf memory.
  *                     NULL will ignore dynamic arrays decoding.
  * @retval offset or ERROR value if < 0
  */
int32_t dsdl_uavcan_protocol_debug_KeyValue_decode(const CanardRxTransfer* transfer,
  uint16_t payload_len,
  dsdl_uavcan_protocol_debug_KeyValue* dest,
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
        for (uint32_t c = 0; c < sizeof(dsdl_uavcan_protocol_debug_KeyValue); c++)
        {
            ((uint8_t*)dest)[c] = 0x00;
        }

        ret = dsdl_uavcan_protocol_debug_KeyValue_decode_internal(transfer, payload_len, dest, dyn_arr_buf, offset, tao);

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
#endif // __DSDL_UAVCAN_PROTOCOL_DEBUG_KEYVALUE