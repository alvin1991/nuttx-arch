/*
 * UAVCAN data structure definition for libcanard.
 *
 * Autogenerated, do not edit.
 *
 */

#ifndef __DSDL_UAVCAN_PROTOCOL_GETTRANSPORTSTATS
#define __DSDL_UAVCAN_PROTOCOL_GETTRANSPORTSTATS

#include <uavcan_service/libcanard/canard.h>

#ifdef __cplusplus
extern "C"
{
#endif

#include <dsdl/uavcan/protocol/CANIfaceStats.h>

/******************************* Source text **********************************
#
# Get transport statistics.
#

---

#
# UAVCAN transport layer statistics.
#
uint48 transfers_tx             # Number of transmitted transfers.
uint48 transfers_rx             # Number of received transfers.
uint48 transfer_errors          # Number of errors detected in the UAVCAN transport layer.

#
# CAN bus statistics, for each interface independently.
#
CANIfaceStats[<=3] can_iface_stats
******************************************************************************/

/********************* DSDL signature source definition ***********************
dsdl.uavcan.protocol.GetTransportStats
---
saturated uint48 transfers_tx
saturated uint48 transfers_rx
saturated uint48 transfer_errors
dsdl.uavcan.protocol.CANIfaceStats[<=3] can_iface_stats
******************************************************************************/

#define DSDL_UAVCAN_PROTOCOL_GETTRANSPORTSTATS_ID          4
#define DSDL_UAVCAN_PROTOCOL_GETTRANSPORTSTATS_NAME        "dsdl.uavcan.protocol.GetTransportStats"
#define DSDL_UAVCAN_PROTOCOL_GETTRANSPORTSTATS_SIGNATURE   (0x5FB269C4A2045519ULL)

#define DSDL_UAVCAN_PROTOCOL_GETTRANSPORTSTATS_REQUEST_MAX_SIZE ((0 + 7)/8)

typedef struct
{
    uint8_t empty;
} dsdl_uavcan_protocol_GetTransportStatsRequest;

static inline
uint32_t dsdl_uavcan_protocol_GetTransportStatsRequest_encode(dsdl_uavcan_protocol_GetTransportStatsRequest* source, void* msg_buf);

static inline
int32_t dsdl_uavcan_protocol_GetTransportStatsRequest_decode(const CanardRxTransfer* transfer, uint16_t payload_len, dsdl_uavcan_protocol_GetTransportStatsRequest* dest, uint8_t** dyn_arr_buf);

static inline
uint32_t dsdl_uavcan_protocol_GetTransportStatsRequest_encode_internal(dsdl_uavcan_protocol_GetTransportStatsRequest* source, void* msg_buf, uint32_t offset, uint8_t root_item);

static inline
int32_t dsdl_uavcan_protocol_GetTransportStatsRequest_decode_internal(const CanardRxTransfer* transfer, uint16_t payload_len, dsdl_uavcan_protocol_GetTransportStatsRequest* dest, uint8_t** dyn_arr_buf, int32_t offset, uint8_t tao);

#define DSDL_UAVCAN_PROTOCOL_GETTRANSPORTSTATS_RESPONSE_MAX_SIZE ((578 + 7)/8)

// Constants

#define DSDL_UAVCAN_PROTOCOL_GETTRANSPORTSTATS_RESPONSE_CAN_IFACE_STATS_MAX_LENGTH       3

typedef struct
{
    // FieldTypes
    uint64_t   transfers_tx;                  // bit len 48
    uint64_t   transfers_rx;                  // bit len 48
    uint64_t   transfer_errors;               // bit len 48
    struct
    {
        uint8_t    len;                       // Dynamic array length
        dsdl_uavcan_protocol_CANIfaceStats* data;                      // Dynamic Array 144bit[3] max items
    } can_iface_stats;

} dsdl_uavcan_protocol_GetTransportStatsResponse;

static inline
uint32_t dsdl_uavcan_protocol_GetTransportStatsResponse_encode(dsdl_uavcan_protocol_GetTransportStatsResponse* source, void* msg_buf);

static inline
int32_t dsdl_uavcan_protocol_GetTransportStatsResponse_decode(const CanardRxTransfer* transfer, uint16_t payload_len, dsdl_uavcan_protocol_GetTransportStatsResponse* dest, uint8_t** dyn_arr_buf);

static inline
uint32_t dsdl_uavcan_protocol_GetTransportStatsResponse_encode_internal(dsdl_uavcan_protocol_GetTransportStatsResponse* source, void* msg_buf, uint32_t offset, uint8_t root_item);

static inline
int32_t dsdl_uavcan_protocol_GetTransportStatsResponse_decode_internal(const CanardRxTransfer* transfer, uint16_t payload_len, dsdl_uavcan_protocol_GetTransportStatsResponse* dest, uint8_t** dyn_arr_buf, int32_t offset, uint8_t tao);

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

uint32_t dsdl_uavcan_protocol_GetTransportStatsRequest_encode_internal(dsdl_uavcan_protocol_GetTransportStatsRequest* CANARD_MAYBE_UNUSED(source),
  void* CANARD_MAYBE_UNUSED(msg_buf),
  uint32_t offset,
  uint8_t CANARD_MAYBE_UNUSED(root_item))
{
    return offset;
}

uint32_t dsdl_uavcan_protocol_GetTransportStatsRequest_encode(dsdl_uavcan_protocol_GetTransportStatsRequest* CANARD_MAYBE_UNUSED(source), void* CANARD_MAYBE_UNUSED(msg_buf))
{
    return 0;
}

int32_t dsdl_uavcan_protocol_GetTransportStatsRequest_decode_internal(const CanardRxTransfer* CANARD_MAYBE_UNUSED(transfer),
  uint16_t CANARD_MAYBE_UNUSED(payload_len),
  dsdl_uavcan_protocol_GetTransportStatsRequest* CANARD_MAYBE_UNUSED(dest),
  uint8_t** CANARD_MAYBE_UNUSED(dyn_arr_buf),
  int32_t offset,
  uint8_t CANARD_MAYBE_UNUSED(tao))
{
    return offset;
}

int32_t dsdl_uavcan_protocol_GetTransportStatsRequest_decode(const CanardRxTransfer* CANARD_MAYBE_UNUSED(transfer),
  uint16_t CANARD_MAYBE_UNUSED(payload_len),
  dsdl_uavcan_protocol_GetTransportStatsRequest* CANARD_MAYBE_UNUSED(dest),
  uint8_t** CANARD_MAYBE_UNUSED(dyn_arr_buf))
{
    return 0;
}

/**
  * @brief dsdl_uavcan_protocol_GetTransportStatsResponse_encode_internal
  * @param source : pointer to source data struct
  * @param msg_buf: pointer to msg storage
  * @param offset: bit offset to msg storage
  * @param root_item: for detecting if TAO should be used
  * @retval returns offset
  */
uint32_t dsdl_uavcan_protocol_GetTransportStatsResponse_encode_internal(dsdl_uavcan_protocol_GetTransportStatsResponse* source,
  void* msg_buf,
  uint32_t offset,
  uint8_t CANARD_MAYBE_UNUSED(root_item))
{
    uint32_t c = 0;

    source->transfers_tx = CANARD_INTERNAL_SATURATE_UNSIGNED(source->transfers_tx, 281474976710655)
    canardEncodeScalar(msg_buf, offset, 48, (void*)&source->transfers_tx); // 281474976710655
    offset += 48;

    source->transfers_rx = CANARD_INTERNAL_SATURATE_UNSIGNED(source->transfers_rx, 281474976710655)
    canardEncodeScalar(msg_buf, offset, 48, (void*)&source->transfers_rx); // 281474976710655
    offset += 48;

    source->transfer_errors = CANARD_INTERNAL_SATURATE_UNSIGNED(source->transfer_errors, 281474976710655)
    canardEncodeScalar(msg_buf, offset, 48, (void*)&source->transfer_errors); // 281474976710655
    offset += 48;

    // Dynamic Array (can_iface_stats)
    if (! root_item)
    {
        // - Add array length
        canardEncodeScalar(msg_buf, offset, 2, (void*)&source->can_iface_stats.len);
        offset += 2;
    }

    // - Add array items
    for (c = 0; c < source->can_iface_stats.len; c++)
    {
        offset += dsdl_uavcan_protocol_CANIfaceStats_encode_internal((void*)&source->can_iface_stats.data[c], msg_buf, offset, 0);
    }

    return offset;
}

/**
  * @brief dsdl_uavcan_protocol_GetTransportStatsResponse_encode
  * @param source : Pointer to source data struct
  * @param msg_buf: Pointer to msg storage
  * @retval returns message length as bytes
  */
uint32_t dsdl_uavcan_protocol_GetTransportStatsResponse_encode(dsdl_uavcan_protocol_GetTransportStatsResponse* source, void* msg_buf)
{
    uint32_t offset = 0;

    offset = dsdl_uavcan_protocol_GetTransportStatsResponse_encode_internal(source, msg_buf, offset, 1);

    return (offset + 7 ) / 8;
}

/**
  * @brief dsdl_uavcan_protocol_GetTransportStatsResponse_decode_internal
  * @param transfer: Pointer to CanardRxTransfer transfer
  * @param payload_len: Payload message length
  * @param dest: Pointer to destination struct
  * @param dyn_arr_buf: NULL or Pointer to memory storage to be used for dynamic arrays
  *                     dsdl_uavcan_protocol_GetTransportStatsResponse dyn memory will point to dyn_arr_buf memory.
  *                     NULL will ignore dynamic arrays decoding.
  * @param offset: Call with 0, bit offset to msg storage
  * @param tao: is tail array optimization used
  * @retval offset or ERROR value if < 0
  */
int32_t dsdl_uavcan_protocol_GetTransportStatsResponse_decode_internal(
  const CanardRxTransfer* transfer,
  uint16_t CANARD_MAYBE_UNUSED(payload_len),
  dsdl_uavcan_protocol_GetTransportStatsResponse* dest,
  uint8_t** CANARD_MAYBE_UNUSED(dyn_arr_buf),
  int32_t offset,
  uint8_t CANARD_MAYBE_UNUSED(tao))
{
    int32_t ret = 0;
    uint32_t c = 0;

    ret = canardDecodeScalar(transfer, offset, 48, false, (void*)&dest->transfers_tx);
    if (ret != 48)
    {
        goto dsdl_uavcan_protocol_GetTransportStatsResponse_error_exit;
    }
    offset += 48;

    ret = canardDecodeScalar(transfer, offset, 48, false, (void*)&dest->transfers_rx);
    if (ret != 48)
    {
        goto dsdl_uavcan_protocol_GetTransportStatsResponse_error_exit;
    }
    offset += 48;

    ret = canardDecodeScalar(transfer, offset, 48, false, (void*)&dest->transfer_errors);
    if (ret != 48)
    {
        goto dsdl_uavcan_protocol_GetTransportStatsResponse_error_exit;
    }
    offset += 48;

    // Dynamic Array (can_iface_stats)
    //  - Last item in struct & Root item & (Array Size > 8 bit), tail array optimization
    if (payload_len && tao == CANARD_INTERNAL_ENABLE_TAO)
    {
        //  - Calculate Array length from MSG length
        dest->can_iface_stats.len = ((payload_len * 8) - offset ) / 144; // 144 bit array item size
    }
    else
    {
        // - Array length 2 bits
        ret = canardDecodeScalar(transfer,
                                 offset,
                                 2,
                                 false,
                                 (void*)&dest->can_iface_stats.len); // 0
        if (ret != 2)
        {
            goto dsdl_uavcan_protocol_GetTransportStatsResponse_error_exit;
        }
        offset += 2;
    }

    //  - Get Array
    if (dyn_arr_buf)
    {
        dest->can_iface_stats.data = (dsdl_uavcan_protocol_CANIfaceStats*)*dyn_arr_buf;
    }

    for (c = 0; c < dest->can_iface_stats.len; c++)
    {
        offset += dsdl_uavcan_protocol_CANIfaceStats_decode_internal(transfer,
                                                0,
                                                (void*)&dest->can_iface_stats.data[c],
                                                dyn_arr_buf,
                                                offset,
                                                tao);
    }
    return offset;

dsdl_uavcan_protocol_GetTransportStatsResponse_error_exit:
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
  * @brief dsdl_uavcan_protocol_GetTransportStatsResponse_decode
  * @param transfer: Pointer to CanardRxTransfer transfer
  * @param payload_len: Payload message length
  * @param dest: Pointer to destination struct
  * @param dyn_arr_buf: NULL or Pointer to memory storage to be used for dynamic arrays
  *                     dsdl_uavcan_protocol_GetTransportStatsResponse dyn memory will point to dyn_arr_buf memory.
  *                     NULL will ignore dynamic arrays decoding.
  * @retval offset or ERROR value if < 0
  */
int32_t dsdl_uavcan_protocol_GetTransportStatsResponse_decode(const CanardRxTransfer* transfer,
  uint16_t payload_len,
  dsdl_uavcan_protocol_GetTransportStatsResponse* dest,
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
        for (uint32_t c = 0; c < sizeof(dsdl_uavcan_protocol_GetTransportStatsResponse); c++)
        {
            ((uint8_t*)dest)[c] = 0x00;
        }

        ret = dsdl_uavcan_protocol_GetTransportStatsResponse_decode_internal(transfer, payload_len, dest, dyn_arr_buf, offset, tao);

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
#endif // __DSDL_UAVCAN_PROTOCOL_GETTRANSPORTSTATS