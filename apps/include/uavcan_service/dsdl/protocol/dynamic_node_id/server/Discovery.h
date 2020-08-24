/*
 * UAVCAN data structure definition for libcanard.
 *
 * Autogenerated, do not edit.
 *
 */

#ifndef __DSDL_UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_SERVER_DISCOVERY
#define __DSDL_UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_SERVER_DISCOVERY

#include <uavcan_service/libcanard/canard.h>

#ifdef __cplusplus
extern "C"
{
#endif

/******************************* Source text **********************************
#
# THIS DEFINITION IS SUBJECT TO CHANGE.
#
# This message is used by allocation servers to find each other's node ID.
# Please refer to the specification for details.
#
# A server should stop publishing this message as soon as it has discovered all other nodes in the cluster.
#
# An exception applies: when a server receives a Discovery message from another server where the list
# of known nodes is incomplete (i.e. len(known_nodes) < configured_cluster_size), the server must
# publish a discovery message once. This condition allows other servers to quickly re-discover the cluster
# after restart.
#

#
# This message should be broadcasted by the server at this interval until all other servers are discovered.
#
uint16 BROADCASTING_PERIOD_MS = 1000

#
# Number of servers in the cluster as configured on the sender.
#
uint8 configured_cluster_size

#
# Node ID of servers that are known to the publishing server, including the publishing server itself.
# Capacity of this array defines maximum size of the server cluster.
#
uint8[<=5] known_nodes
******************************************************************************/

/********************* DSDL signature source definition ***********************
dsdl.uavcan.protocol.dynamic_node_id.server.Discovery
saturated uint8 configured_cluster_size
saturated uint8[<=5] known_nodes
******************************************************************************/

#define DSDL_UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_SERVER_DISCOVERY_ID 390
#define DSDL_UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_SERVER_DISCOVERY_NAME "dsdl.uavcan.protocol.dynamic_node_id.server.Discovery"
#define DSDL_UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_SERVER_DISCOVERY_SIGNATURE (0xE5821304D2D92DFDULL)

#define DSDL_UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_SERVER_DISCOVERY_MAX_SIZE ((51 + 7)/8)

// Constants
#define DSDL_UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_SERVER_DISCOVERY_BROADCASTING_PERIOD_MS       1000 // 1000

#define DSDL_UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_SERVER_DISCOVERY_KNOWN_NODES_MAX_LENGTH     5

typedef struct
{
    // FieldTypes
    uint8_t    configured_cluster_size;       // bit len 8
    struct
    {
        uint8_t    len;                       // Dynamic array length
        uint8_t*   data;                      // Dynamic Array 8bit[5] max items
    } known_nodes;

} dsdl_uavcan_protocol_dynamic_node_id_server_Discovery;

static inline
uint32_t dsdl_uavcan_protocol_dynamic_node_id_server_Discovery_encode(dsdl_uavcan_protocol_dynamic_node_id_server_Discovery* source, void* msg_buf);

static inline
int32_t dsdl_uavcan_protocol_dynamic_node_id_server_Discovery_decode(const CanardRxTransfer* transfer, uint16_t payload_len, dsdl_uavcan_protocol_dynamic_node_id_server_Discovery* dest, uint8_t** dyn_arr_buf);

static inline
uint32_t dsdl_uavcan_protocol_dynamic_node_id_server_Discovery_encode_internal(dsdl_uavcan_protocol_dynamic_node_id_server_Discovery* source, void* msg_buf, uint32_t offset, uint8_t root_item);

static inline
int32_t dsdl_uavcan_protocol_dynamic_node_id_server_Discovery_decode_internal(const CanardRxTransfer* transfer, uint16_t payload_len, dsdl_uavcan_protocol_dynamic_node_id_server_Discovery* dest, uint8_t** dyn_arr_buf, int32_t offset, uint8_t tao);

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
  * @brief dsdl_uavcan_protocol_dynamic_node_id_server_Discovery_encode_internal
  * @param source : pointer to source data struct
  * @param msg_buf: pointer to msg storage
  * @param offset: bit offset to msg storage
  * @param root_item: for detecting if TAO should be used
  * @retval returns offset
  */
uint32_t dsdl_uavcan_protocol_dynamic_node_id_server_Discovery_encode_internal(dsdl_uavcan_protocol_dynamic_node_id_server_Discovery* source,
  void* msg_buf,
  uint32_t offset,
  uint8_t CANARD_MAYBE_UNUSED(root_item))
{
    uint32_t c = 0;

    canardEncodeScalar(msg_buf, offset, 8, (void*)&source->configured_cluster_size); // 255
    offset += 8;

    // Dynamic Array (known_nodes)
    if (! root_item)
    {
        // - Add array length
        canardEncodeScalar(msg_buf, offset, 3, (void*)&source->known_nodes.len);
        offset += 3;
    }

    // - Add array items
    for (c = 0; c < source->known_nodes.len; c++)
    {
        canardEncodeScalar(msg_buf,
                           offset,
                           8,
                           (void*)(source->known_nodes.data + c));// 255
        offset += 8;
    }

    return offset;
}

/**
  * @brief dsdl_uavcan_protocol_dynamic_node_id_server_Discovery_encode
  * @param source : Pointer to source data struct
  * @param msg_buf: Pointer to msg storage
  * @retval returns message length as bytes
  */
uint32_t dsdl_uavcan_protocol_dynamic_node_id_server_Discovery_encode(dsdl_uavcan_protocol_dynamic_node_id_server_Discovery* source, void* msg_buf)
{
    uint32_t offset = 0;

    offset = dsdl_uavcan_protocol_dynamic_node_id_server_Discovery_encode_internal(source, msg_buf, offset, 1);

    return (offset + 7 ) / 8;
}

/**
  * @brief dsdl_uavcan_protocol_dynamic_node_id_server_Discovery_decode_internal
  * @param transfer: Pointer to CanardRxTransfer transfer
  * @param payload_len: Payload message length
  * @param dest: Pointer to destination struct
  * @param dyn_arr_buf: NULL or Pointer to memory storage to be used for dynamic arrays
  *                     dsdl_uavcan_protocol_dynamic_node_id_server_Discovery dyn memory will point to dyn_arr_buf memory.
  *                     NULL will ignore dynamic arrays decoding.
  * @param offset: Call with 0, bit offset to msg storage
  * @param tao: is tail array optimization used
  * @retval offset or ERROR value if < 0
  */
int32_t dsdl_uavcan_protocol_dynamic_node_id_server_Discovery_decode_internal(
  const CanardRxTransfer* transfer,
  uint16_t CANARD_MAYBE_UNUSED(payload_len),
  dsdl_uavcan_protocol_dynamic_node_id_server_Discovery* dest,
  uint8_t** CANARD_MAYBE_UNUSED(dyn_arr_buf),
  int32_t offset,
  uint8_t CANARD_MAYBE_UNUSED(tao))
{
    int32_t ret = 0;
    uint32_t c = 0;

    ret = canardDecodeScalar(transfer, offset, 8, false, (void*)&dest->configured_cluster_size);
    if (ret != 8)
    {
        goto dsdl_uavcan_protocol_dynamic_node_id_server_Discovery_error_exit;
    }
    offset += 8;

    // Dynamic Array (known_nodes)
    //  - Last item in struct & Root item & (Array Size > 8 bit), tail array optimization
    if (payload_len && tao == CANARD_INTERNAL_ENABLE_TAO)
    {
        //  - Calculate Array length from MSG length
        dest->known_nodes.len = ((payload_len * 8) - offset ) / 8; // 8 bit array item size
    }
    else
    {
        // - Array length 3 bits
        ret = canardDecodeScalar(transfer,
                                 offset,
                                 3,
                                 false,
                                 (void*)&dest->known_nodes.len); // 255
        if (ret != 3)
        {
            goto dsdl_uavcan_protocol_dynamic_node_id_server_Discovery_error_exit;
        }
        offset += 3;
    }

    //  - Get Array
    if (dyn_arr_buf)
    {
        dest->known_nodes.data = (uint8_t*)*dyn_arr_buf;
    }

    for (c = 0; c < dest->known_nodes.len; c++)
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
                goto dsdl_uavcan_protocol_dynamic_node_id_server_Discovery_error_exit;
            }
            *dyn_arr_buf = (uint8_t*)(((uint8_t*)*dyn_arr_buf) + 1);
        }
        offset += 8;
    }
    return offset;

dsdl_uavcan_protocol_dynamic_node_id_server_Discovery_error_exit:
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
  * @brief dsdl_uavcan_protocol_dynamic_node_id_server_Discovery_decode
  * @param transfer: Pointer to CanardRxTransfer transfer
  * @param payload_len: Payload message length
  * @param dest: Pointer to destination struct
  * @param dyn_arr_buf: NULL or Pointer to memory storage to be used for dynamic arrays
  *                     dsdl_uavcan_protocol_dynamic_node_id_server_Discovery dyn memory will point to dyn_arr_buf memory.
  *                     NULL will ignore dynamic arrays decoding.
  * @retval offset or ERROR value if < 0
  */
int32_t dsdl_uavcan_protocol_dynamic_node_id_server_Discovery_decode(const CanardRxTransfer* transfer,
  uint16_t payload_len,
  dsdl_uavcan_protocol_dynamic_node_id_server_Discovery* dest,
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
        for (uint32_t c = 0; c < sizeof(dsdl_uavcan_protocol_dynamic_node_id_server_Discovery); c++)
        {
            ((uint8_t*)dest)[c] = 0x00;
        }

        ret = dsdl_uavcan_protocol_dynamic_node_id_server_Discovery_decode_internal(transfer, payload_len, dest, dyn_arr_buf, offset, tao);

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
#endif // __DSDL_UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_SERVER_DISCOVERY