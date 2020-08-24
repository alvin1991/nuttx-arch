/*
 * UAVCAN data structure definition for libcanard.
 *
 * Autogenerated, do not edit.
 *
 */

#ifndef __DSDL_UAVCAN_PROTOCOL_FILE_GETINFO
#define __DSDL_UAVCAN_PROTOCOL_FILE_GETINFO

#include <uavcan_service/libcanard/canard.h>

#ifdef __cplusplus
extern "C"
{
#endif

#include <dsdl/uavcan/protocol/file/EntryType.h>
#include <dsdl/uavcan/protocol/file/Error.h>
#include <dsdl/uavcan/protocol/file/Path.h>

/******************************* Source text **********************************
#
# Request info about a remote file system entry (file, directory, etc).
#

Path path

---

#
# File size in bytes.
# Should be set to zero for directories.
#
uint40 size

Error error

EntryType entry_type
******************************************************************************/

/********************* DSDL signature source definition ***********************
dsdl.uavcan.protocol.file.GetInfo
dsdl.uavcan.protocol.file.Path path
---
saturated uint40 size
dsdl.uavcan.protocol.file.Error error
dsdl.uavcan.protocol.file.EntryType entry_type
******************************************************************************/

#define DSDL_UAVCAN_PROTOCOL_FILE_GETINFO_ID               45
#define DSDL_UAVCAN_PROTOCOL_FILE_GETINFO_NAME             "dsdl.uavcan.protocol.file.GetInfo"
#define DSDL_UAVCAN_PROTOCOL_FILE_GETINFO_SIGNATURE        (0x7FC8B446E04D925BULL)

#define DSDL_UAVCAN_PROTOCOL_FILE_GETINFO_REQUEST_MAX_SIZE ((1608 + 7)/8)

// Constants

typedef struct
{
    // FieldTypes
    dsdl_uavcan_protocol_file_Path path;                          //

} dsdl_uavcan_protocol_file_GetInfoRequest;

static inline
uint32_t dsdl_uavcan_protocol_file_GetInfoRequest_encode(dsdl_uavcan_protocol_file_GetInfoRequest* source, void* msg_buf);

static inline
int32_t dsdl_uavcan_protocol_file_GetInfoRequest_decode(const CanardRxTransfer* transfer, uint16_t payload_len, dsdl_uavcan_protocol_file_GetInfoRequest* dest, uint8_t** dyn_arr_buf);

static inline
uint32_t dsdl_uavcan_protocol_file_GetInfoRequest_encode_internal(dsdl_uavcan_protocol_file_GetInfoRequest* source, void* msg_buf, uint32_t offset, uint8_t root_item);

static inline
int32_t dsdl_uavcan_protocol_file_GetInfoRequest_decode_internal(const CanardRxTransfer* transfer, uint16_t payload_len, dsdl_uavcan_protocol_file_GetInfoRequest* dest, uint8_t** dyn_arr_buf, int32_t offset, uint8_t tao);

#define DSDL_UAVCAN_PROTOCOL_FILE_GETINFO_RESPONSE_MAX_SIZE ((64 + 7)/8)

// Constants

typedef struct
{
    // FieldTypes
    uint64_t   size;                          // bit len 40
    dsdl_uavcan_protocol_file_Error error;                         //
    dsdl_uavcan_protocol_file_EntryType entry_type;                    //

} dsdl_uavcan_protocol_file_GetInfoResponse;

static inline
uint32_t dsdl_uavcan_protocol_file_GetInfoResponse_encode(dsdl_uavcan_protocol_file_GetInfoResponse* source, void* msg_buf);

static inline
int32_t dsdl_uavcan_protocol_file_GetInfoResponse_decode(const CanardRxTransfer* transfer, uint16_t payload_len, dsdl_uavcan_protocol_file_GetInfoResponse* dest, uint8_t** dyn_arr_buf);

static inline
uint32_t dsdl_uavcan_protocol_file_GetInfoResponse_encode_internal(dsdl_uavcan_protocol_file_GetInfoResponse* source, void* msg_buf, uint32_t offset, uint8_t root_item);

static inline
int32_t dsdl_uavcan_protocol_file_GetInfoResponse_decode_internal(const CanardRxTransfer* transfer, uint16_t payload_len, dsdl_uavcan_protocol_file_GetInfoResponse* dest, uint8_t** dyn_arr_buf, int32_t offset, uint8_t tao);

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
  * @brief dsdl_uavcan_protocol_file_GetInfoRequest_encode_internal
  * @param source : pointer to source data struct
  * @param msg_buf: pointer to msg storage
  * @param offset: bit offset to msg storage
  * @param root_item: for detecting if TAO should be used
  * @retval returns offset
  */
uint32_t dsdl_uavcan_protocol_file_GetInfoRequest_encode_internal(dsdl_uavcan_protocol_file_GetInfoRequest* source,
  void* msg_buf,
  uint32_t offset,
  uint8_t CANARD_MAYBE_UNUSED(root_item))
{
    // Compound
    offset = dsdl_uavcan_protocol_file_Path_encode_internal((void*)&source->path, msg_buf, offset, 0);

    return offset;
}

/**
  * @brief dsdl_uavcan_protocol_file_GetInfoRequest_encode
  * @param source : Pointer to source data struct
  * @param msg_buf: Pointer to msg storage
  * @retval returns message length as bytes
  */
uint32_t dsdl_uavcan_protocol_file_GetInfoRequest_encode(dsdl_uavcan_protocol_file_GetInfoRequest* source, void* msg_buf)
{
    uint32_t offset = 0;

    offset = dsdl_uavcan_protocol_file_GetInfoRequest_encode_internal(source, msg_buf, offset, 1);

    return (offset + 7 ) / 8;
}

/**
  * @brief dsdl_uavcan_protocol_file_GetInfoRequest_decode_internal
  * @param transfer: Pointer to CanardRxTransfer transfer
  * @param payload_len: Payload message length
  * @param dest: Pointer to destination struct
  * @param dyn_arr_buf: NULL or Pointer to memory storage to be used for dynamic arrays
  *                     dsdl_uavcan_protocol_file_GetInfoRequest dyn memory will point to dyn_arr_buf memory.
  *                     NULL will ignore dynamic arrays decoding.
  * @param offset: Call with 0, bit offset to msg storage
  * @param tao: is tail array optimization used
  * @retval offset or ERROR value if < 0
  */
int32_t dsdl_uavcan_protocol_file_GetInfoRequest_decode_internal(
  const CanardRxTransfer* transfer,
  uint16_t CANARD_MAYBE_UNUSED(payload_len),
  dsdl_uavcan_protocol_file_GetInfoRequest* dest,
  uint8_t** CANARD_MAYBE_UNUSED(dyn_arr_buf),
  int32_t offset,
  uint8_t CANARD_MAYBE_UNUSED(tao))
{
    int32_t ret = 0;

    // Compound
    offset = dsdl_uavcan_protocol_file_Path_decode_internal(transfer, 0, (void*)&dest->path, dyn_arr_buf, offset, tao);
    if (offset < 0)
    {
        ret = offset;
        goto dsdl_uavcan_protocol_file_GetInfoRequest_error_exit;
    }
    return offset;

dsdl_uavcan_protocol_file_GetInfoRequest_error_exit:
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
  * @brief dsdl_uavcan_protocol_file_GetInfoRequest_decode
  * @param transfer: Pointer to CanardRxTransfer transfer
  * @param payload_len: Payload message length
  * @param dest: Pointer to destination struct
  * @param dyn_arr_buf: NULL or Pointer to memory storage to be used for dynamic arrays
  *                     dsdl_uavcan_protocol_file_GetInfoRequest dyn memory will point to dyn_arr_buf memory.
  *                     NULL will ignore dynamic arrays decoding.
  * @retval offset or ERROR value if < 0
  */
int32_t dsdl_uavcan_protocol_file_GetInfoRequest_decode(const CanardRxTransfer* transfer,
  uint16_t payload_len,
  dsdl_uavcan_protocol_file_GetInfoRequest* dest,
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
        for (uint32_t c = 0; c < sizeof(dsdl_uavcan_protocol_file_GetInfoRequest); c++)
        {
            ((uint8_t*)dest)[c] = 0x00;
        }

        ret = dsdl_uavcan_protocol_file_GetInfoRequest_decode_internal(transfer, payload_len, dest, dyn_arr_buf, offset, tao);

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

/**
  * @brief dsdl_uavcan_protocol_file_GetInfoResponse_encode_internal
  * @param source : pointer to source data struct
  * @param msg_buf: pointer to msg storage
  * @param offset: bit offset to msg storage
  * @param root_item: for detecting if TAO should be used
  * @retval returns offset
  */
uint32_t dsdl_uavcan_protocol_file_GetInfoResponse_encode_internal(dsdl_uavcan_protocol_file_GetInfoResponse* source,
  void* msg_buf,
  uint32_t offset,
  uint8_t CANARD_MAYBE_UNUSED(root_item))
{
    source->size = CANARD_INTERNAL_SATURATE_UNSIGNED(source->size, 1099511627775)
    canardEncodeScalar(msg_buf, offset, 40, (void*)&source->size); // 1099511627775
    offset += 40;

    // Compound
    offset = dsdl_uavcan_protocol_file_Error_encode_internal((void*)&source->error, msg_buf, offset, 0);

    // Compound
    offset = dsdl_uavcan_protocol_file_EntryType_encode_internal((void*)&source->entry_type, msg_buf, offset, 0);

    return offset;
}

/**
  * @brief dsdl_uavcan_protocol_file_GetInfoResponse_encode
  * @param source : Pointer to source data struct
  * @param msg_buf: Pointer to msg storage
  * @retval returns message length as bytes
  */
uint32_t dsdl_uavcan_protocol_file_GetInfoResponse_encode(dsdl_uavcan_protocol_file_GetInfoResponse* source, void* msg_buf)
{
    uint32_t offset = 0;

    offset = dsdl_uavcan_protocol_file_GetInfoResponse_encode_internal(source, msg_buf, offset, 1);

    return (offset + 7 ) / 8;
}

/**
  * @brief dsdl_uavcan_protocol_file_GetInfoResponse_decode_internal
  * @param transfer: Pointer to CanardRxTransfer transfer
  * @param payload_len: Payload message length
  * @param dest: Pointer to destination struct
  * @param dyn_arr_buf: NULL or Pointer to memory storage to be used for dynamic arrays
  *                     dsdl_uavcan_protocol_file_GetInfoResponse dyn memory will point to dyn_arr_buf memory.
  *                     NULL will ignore dynamic arrays decoding.
  * @param offset: Call with 0, bit offset to msg storage
  * @param tao: is tail array optimization used
  * @retval offset or ERROR value if < 0
  */
int32_t dsdl_uavcan_protocol_file_GetInfoResponse_decode_internal(
  const CanardRxTransfer* transfer,
  uint16_t CANARD_MAYBE_UNUSED(payload_len),
  dsdl_uavcan_protocol_file_GetInfoResponse* dest,
  uint8_t** CANARD_MAYBE_UNUSED(dyn_arr_buf),
  int32_t offset,
  uint8_t CANARD_MAYBE_UNUSED(tao))
{
    int32_t ret = 0;

    ret = canardDecodeScalar(transfer, offset, 40, false, (void*)&dest->size);
    if (ret != 40)
    {
        goto dsdl_uavcan_protocol_file_GetInfoResponse_error_exit;
    }
    offset += 40;

    // Compound
    offset = dsdl_uavcan_protocol_file_Error_decode_internal(transfer, 0, (void*)&dest->error, dyn_arr_buf, offset, tao);
    if (offset < 0)
    {
        ret = offset;
        goto dsdl_uavcan_protocol_file_GetInfoResponse_error_exit;
    }

    // Compound
    offset = dsdl_uavcan_protocol_file_EntryType_decode_internal(transfer, 0, (void*)&dest->entry_type, dyn_arr_buf, offset, tao);
    if (offset < 0)
    {
        ret = offset;
        goto dsdl_uavcan_protocol_file_GetInfoResponse_error_exit;
    }
    return offset;

dsdl_uavcan_protocol_file_GetInfoResponse_error_exit:
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
  * @brief dsdl_uavcan_protocol_file_GetInfoResponse_decode
  * @param transfer: Pointer to CanardRxTransfer transfer
  * @param payload_len: Payload message length
  * @param dest: Pointer to destination struct
  * @param dyn_arr_buf: NULL or Pointer to memory storage to be used for dynamic arrays
  *                     dsdl_uavcan_protocol_file_GetInfoResponse dyn memory will point to dyn_arr_buf memory.
  *                     NULL will ignore dynamic arrays decoding.
  * @retval offset or ERROR value if < 0
  */
int32_t dsdl_uavcan_protocol_file_GetInfoResponse_decode(const CanardRxTransfer* transfer,
  uint16_t payload_len,
  dsdl_uavcan_protocol_file_GetInfoResponse* dest,
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
        for (uint32_t c = 0; c < sizeof(dsdl_uavcan_protocol_file_GetInfoResponse); c++)
        {
            ((uint8_t*)dest)[c] = 0x00;
        }

        ret = dsdl_uavcan_protocol_file_GetInfoResponse_decode_internal(transfer, payload_len, dest, dyn_arr_buf, offset, tao);

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
#endif // __DSDL_UAVCAN_PROTOCOL_FILE_GETINFO