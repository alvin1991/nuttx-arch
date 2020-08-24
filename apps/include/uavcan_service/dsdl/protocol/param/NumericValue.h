/*
 * UAVCAN data structure definition for libcanard.
 *
 * Autogenerated, do not edit.
 *
 */

#ifndef __DSDL_UAVCAN_PROTOCOL_PARAM_NUMERICVALUE
#define __DSDL_UAVCAN_PROTOCOL_PARAM_NUMERICVALUE

#include <uavcan_service/libcanard/canard.h>

#ifdef __cplusplus
extern "C"
{
#endif

#include <dsdl/uavcan/protocol/param/Empty.h>

/******************************* Source text **********************************
#
# Numeric-only value.
#
# This is a union, which means that this structure can contain either one of the fields below.
# The structure is prefixed with tag - a selector value that indicates which particular field is encoded.
#

@union                          # Tag is 2 bits long.

Empty empty                     # Empty field, used to represent an undefined value.

int64   integer_value
float32 real_value
******************************************************************************/

/********************* DSDL signature source definition ***********************
dsdl.uavcan.protocol.param.NumericValue
@union
dsdl.uavcan.protocol.param.Empty empty
saturated int64 integer_value
saturated float32 real_value
******************************************************************************/

#define DSDL_UAVCAN_PROTOCOL_PARAM_NUMERICVALUE_NAME       "dsdl.uavcan.protocol.param.NumericValue"
#define DSDL_UAVCAN_PROTOCOL_PARAM_NUMERICVALUE_SIGNATURE  (0xF824FFBAF28A224FULL)

#define DSDL_UAVCAN_PROTOCOL_PARAM_NUMERICVALUE_MAX_SIZE   ((66 + 7)/8)

// Constants

typedef enum
{
    DSDL_UAVCAN_PROTOCOL_PARAM_NUMERICVALUE_EMPTY,
    DSDL_UAVCAN_PROTOCOL_PARAM_NUMERICVALUE_INTEGER_VALUE,
    DSDL_UAVCAN_PROTOCOL_PARAM_NUMERICVALUE_REAL_VALUE
} dsdl_uavcan_protocol_param_NumericValue_ENUM;

typedef struct
{
    dsdl_uavcan_protocol_param_NumericValue_ENUM union_tag;        // union_tag indicates what field the data structure is holding

    union
    {
    // FieldTypes
    dsdl_uavcan_protocol_param_Empty empty;                         //
    int64_t    integer_value;                 // bit len 64
    float      real_value;                    // float32 Saturate

    };
} dsdl_uavcan_protocol_param_NumericValue;

static inline
uint32_t dsdl_uavcan_protocol_param_NumericValue_encode(dsdl_uavcan_protocol_param_NumericValue* source, void* msg_buf);

static inline
int32_t dsdl_uavcan_protocol_param_NumericValue_decode(const CanardRxTransfer* transfer, uint16_t payload_len, dsdl_uavcan_protocol_param_NumericValue* dest, uint8_t** dyn_arr_buf);

static inline
uint32_t dsdl_uavcan_protocol_param_NumericValue_encode_internal(dsdl_uavcan_protocol_param_NumericValue* source, void* msg_buf, uint32_t offset, uint8_t root_item);

static inline
int32_t dsdl_uavcan_protocol_param_NumericValue_decode_internal(const CanardRxTransfer* transfer, uint16_t payload_len, dsdl_uavcan_protocol_param_NumericValue* dest, uint8_t** dyn_arr_buf, int32_t offset, uint8_t tao);

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
  * @brief dsdl_uavcan_protocol_param_NumericValue_encode_internal
  * @param source : pointer to source data struct
  * @param msg_buf: pointer to msg storage
  * @param offset: bit offset to msg storage
  * @param root_item: for detecting if TAO should be used
  * @retval returns offset
  */
uint32_t dsdl_uavcan_protocol_param_NumericValue_encode_internal(dsdl_uavcan_protocol_param_NumericValue* source,
  void* msg_buf,
  uint32_t offset,
  uint8_t CANARD_MAYBE_UNUSED(root_item))
{
    // Max Union Tag Value
    CANARD_ASSERT(source->union_tag <= 2);

    // Union Tag 2 bits
    canardEncodeScalar(msg_buf, offset, 2, (void*)&source->union_tag); // 2 bits
    offset += 2;

    if (source->union_tag == 0) {
    // Compound
    offset = dsdl_uavcan_protocol_param_Empty_encode_internal((void*)&source->empty, msg_buf, offset, 0);
    }
    else if (source->union_tag == 1) {
    canardEncodeScalar(msg_buf, offset, 64, (void*)&source->integer_value); // 9223372036854775807
    offset += 64;

    }
    else if (source->union_tag == 2) {
    canardEncodeScalar(msg_buf, offset, 32, (void*)&source->real_value); // 2147483647
    offset += 32;

    }

    return offset;
}

/**
  * @brief dsdl_uavcan_protocol_param_NumericValue_encode
  * @param source : Pointer to source data struct
  * @param msg_buf: Pointer to msg storage
  * @retval returns message length as bytes
  */
uint32_t dsdl_uavcan_protocol_param_NumericValue_encode(dsdl_uavcan_protocol_param_NumericValue* source, void* msg_buf)
{
    uint32_t offset = 0;

    offset = dsdl_uavcan_protocol_param_NumericValue_encode_internal(source, msg_buf, offset, 1);

    return (offset + 7 ) / 8;
}

/**
  * @brief dsdl_uavcan_protocol_param_NumericValue_decode_internal
  * @param transfer: Pointer to CanardRxTransfer transfer
  * @param payload_len: Payload message length
  * @param dest: Pointer to destination struct
  * @param dyn_arr_buf: NULL or Pointer to memory storage to be used for dynamic arrays
  *                     dsdl_uavcan_protocol_param_NumericValue dyn memory will point to dyn_arr_buf memory.
  *                     NULL will ignore dynamic arrays decoding.
  * @param offset: Call with 0, bit offset to msg storage
  * @param tao: is tail array optimization used
  * @retval offset or ERROR value if < 0
  */
int32_t dsdl_uavcan_protocol_param_NumericValue_decode_internal(
  const CanardRxTransfer* transfer,
  uint16_t CANARD_MAYBE_UNUSED(payload_len),
  dsdl_uavcan_protocol_param_NumericValue* dest,
  uint8_t** CANARD_MAYBE_UNUSED(dyn_arr_buf),
  int32_t offset,
  uint8_t CANARD_MAYBE_UNUSED(tao))
{
    int32_t ret = 0;

    // Get Union Tag
    ret = canardDecodeScalar(transfer, offset, 2, false, (void*)&dest->union_tag); // 2
    if (ret != 2)
    {
        goto dsdl_uavcan_protocol_param_NumericValue_error_exit;
    }
    offset += 2;

    if (dest->union_tag == 0)
    {
    // Compound
    offset = dsdl_uavcan_protocol_param_Empty_decode_internal(transfer, 0, (void*)&dest->empty, dyn_arr_buf, offset, tao);
    if (offset < 0)
    {
        ret = offset;
        goto dsdl_uavcan_protocol_param_NumericValue_error_exit;
    }
    }
    else if (dest->union_tag == 1)
    {
    ret = canardDecodeScalar(transfer, offset, 64, true, (void*)&dest->integer_value);
    if (ret != 64)
    {
        goto dsdl_uavcan_protocol_param_NumericValue_error_exit;
    }
    offset += 64;
    }
    else if (dest->union_tag == 2)
    {
    ret = canardDecodeScalar(transfer, offset, 32, false, (void*)&dest->real_value);
    if (ret != 32)
    {
        goto dsdl_uavcan_protocol_param_NumericValue_error_exit;
    }
    offset += 32;
    }
    return offset;

dsdl_uavcan_protocol_param_NumericValue_error_exit:
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
  * @brief dsdl_uavcan_protocol_param_NumericValue_decode
  * @param transfer: Pointer to CanardRxTransfer transfer
  * @param payload_len: Payload message length
  * @param dest: Pointer to destination struct
  * @param dyn_arr_buf: NULL or Pointer to memory storage to be used for dynamic arrays
  *                     dsdl_uavcan_protocol_param_NumericValue dyn memory will point to dyn_arr_buf memory.
  *                     NULL will ignore dynamic arrays decoding.
  * @retval offset or ERROR value if < 0
  */
int32_t dsdl_uavcan_protocol_param_NumericValue_decode(const CanardRxTransfer* transfer,
  uint16_t payload_len,
  dsdl_uavcan_protocol_param_NumericValue* dest,
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
        for (uint32_t c = 0; c < sizeof(dsdl_uavcan_protocol_param_NumericValue); c++)
        {
            ((uint8_t*)dest)[c] = 0x00;
        }

        ret = dsdl_uavcan_protocol_param_NumericValue_decode_internal(transfer, payload_len, dest, dyn_arr_buf, offset, tao);

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
#endif // __DSDL_UAVCAN_PROTOCOL_PARAM_NUMERICVALUE