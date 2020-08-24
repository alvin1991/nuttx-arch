/*
 * UAVCAN data structure definition for libcanard.
 *
 * Autogenerated, do not edit.
 *
 */

#ifndef __DSDL_UAVCAN_PROTOCOL_FILE_WRITE
#define __DSDL_UAVCAN_PROTOCOL_FILE_WRITE

#include <uavcan_service/libcanard/canard.h>

#ifdef __cplusplus
extern "C"
{
#endif

#include <dsdl/uavcan/protocol/file/Error.h>
#include <dsdl/uavcan/protocol/file/Path.h>

/******************************* Source text **********************************
#
# Write into a remote file.
# The server shall place the contents of the field 'data' into the file pointed by 'path' at the offset specified by
# the field 'offset'.
#
# When writing a file, the client should repeatedly call this service with data while advancing offset until the file
# is written completely. When write is complete, the client shall call the service one last time, with the offset
# set to the size of the file and with the data field empty, which will signal the server that the write operation is
# complete.
#
# When the write operation is complete, the server shall truncate the resulting file past the specified offset.
#
# Server implementation advice:
# It is recommended to implement proper handling of concurrent writes to the same file from different clients, for
# example by means of creating a staging area for uncompleted writes (like FTP servers do).
#

uint40 offset

Path path

uint8[<=192] data

---

Error error
******************************************************************************/

/********************* DSDL signature source definition ***********************
dsdl.uavcan.protocol.file.Write
saturated uint40 offset
dsdl.uavcan.protocol.file.Path path
saturated uint8[<=192] data
---
dsdl.uavcan.protocol.file.Error error
******************************************************************************/

#define DSDL_UAVCAN_PROTOCOL_FILE_WRITE_ID                 49
#define DSDL_UAVCAN_PROTOCOL_FILE_WRITE_NAME               "dsdl.uavcan.protocol.file.Write"
#define DSDL_UAVCAN_PROTOCOL_FILE_WRITE_SIGNATURE          (0xCFE18CF5FF9621ABULL)

#define DSDL_UAVCAN_PROTOCOL_FILE_WRITE_REQUEST_MAX_SIZE   ((3192 + 7)/8)

// Constants

#define DSDL_UAVCAN_PROTOCOL_FILE_WRITE_REQUEST_DATA_MAX_LENGTH                          192

typedef struct
{
    // FieldTypes
    uint64_t   offset;                        // bit len 40
    dsdl_uavcan_protocol_file_Path path;                          //
    struct
    {
        uint8_t    len;                       // Dynamic array length
        uint8_t*   data;                      // Dynamic Array 8bit[192] max items
    } data;

} dsdl_uavcan_protocol_file_WriteRequest;

static inline
uint32_t dsdl_uavcan_protocol_file_WriteRequest_encode(dsdl_uavcan_protocol_file_WriteRequest* source, void* msg_buf);

static inline
int32_t dsdl_uavcan_protocol_file_WriteRequest_decode(const CanardRxTransfer* transfer, uint16_t payload_len, dsdl_uavcan_protocol_file_WriteRequest* dest, uint8_t** dyn_arr_buf);

static inline
uint32_t dsdl_uavcan_protocol_file_WriteRequest_encode_internal(dsdl_uavcan_protocol_file_WriteRequest* source, void* msg_buf, uint32_t offset, uint8_t root_item);

static inline
int32_t dsdl_uavcan_protocol_file_WriteRequest_decode_internal(const CanardRxTransfer* transfer, uint16_t payload_len, dsdl_uavcan_protocol_file_WriteRequest* dest, uint8_t** dyn_arr_buf, int32_t offset, uint8_t tao);

#define DSDL_UAVCAN_PROTOCOL_FILE_WRITE_RESPONSE_MAX_SIZE  ((16 + 7)/8)

// Constants

typedef struct
{
    // FieldTypes
    dsdl_uavcan_protocol_file_Error error;                         //

} dsdl_uavcan_protocol_file_WriteResponse;

static inline
uint32_t dsdl_uavcan_protocol_file_WriteResponse_encode(dsdl_uavcan_protocol_file_WriteResponse* source, void* msg_buf);

static inline
int32_t dsdl_uavcan_protocol_file_WriteResponse_decode(const CanardRxTransfer* transfer, uint16_t payload_len, dsdl_uavcan_protocol_file_WriteResponse* dest, uint8_t** dyn_arr_buf);

static inline
uint32_t dsdl_uavcan_protocol_file_WriteResponse_encode_internal(dsdl_uavcan_protocol_file_WriteResponse* source, void* msg_buf, uint32_t offset, uint8_t root_item);

static inline
int32_t dsdl_uavcan_protocol_file_WriteResponse_decode_internal(const CanardRxTransfer* transfer, uint16_t payload_len, dsdl_uavcan_protocol_file_WriteResponse* dest, uint8_t** dyn_arr_buf, int32_t offset, uint8_t tao);

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
  * @brief dsdl_uavcan_protocol_file_WriteRequest_encode_internal
  * @param source : pointer to source data struct
  * @param msg_buf: pointer to msg storage
  * @param offset: bit offset to msg storage
  * @param root_item: for detecting if TAO should be used
  * @retval returns offset
  */
uint32_t dsdl_uavcan_protocol_file_WriteRequest_encode_internal(dsdl_uavcan_protocol_file_WriteRequest* source,
  void* msg_buf,
  uint32_t offset,
  uint8_t CANARD_MAYBE_UNUSED(root_item))
{
    uint32_t c = 0;

    source->offset = CANARD_INTERNAL_SATURATE_UNSIGNED(source->offset, 1099511627775)
    canardEncodeScalar(msg_buf, offset, 40, (void*)&source->offset); // 1099511627775
    offset += 40;

    // Compound
    offset = dsdl_uavcan_protocol_file_Path_encode_internal((void*)&source->path, msg_buf, offset, 0);

    // Dynamic Array (data)
    if (! root_item)
    {
        // - Add array length
        canardEncodeScalar(msg_buf, offset, 8, (void*)&source->data.len);
        offset += 8;
    }

    // - Add array items
    for (c = 0; c < source->data.len; c++)
    {
        canardEncodeScalar(msg_buf,
                           offset,
                           8,
                           (void*)(source->data.data + c));// 255
        offset += 8;
    }

    return offset;
}

/**
  * @brief dsdl_uavcan_protocol_file_WriteRequest_encode
  * @param source : Pointer to source data struct
  * @param msg_buf: Pointer to msg storage
  * @retval returns message length as bytes
  */
uint32_t dsdl_uavcan_protocol_file_WriteRequest_encode(dsdl_uavcan_protocol_file_WriteRequest* source, void* msg_buf)
{
    uint32_t offset = 0;

    offset = dsdl_uavcan_protocol_file_WriteRequest_encode_internal(source, msg_buf, offset, 1);

    return (offset + 7 ) / 8;
}

/**
  * @brief dsdl_uavcan_protocol_file_WriteRequest_decode_internal
  * @param transfer: Pointer to CanardRxTransfer transfer
  * @param payload_len: Payload message length
  * @param dest: Pointer to destination struct
  * @param dyn_arr_buf: NULL or Pointer to memory storage to be used for dynamic arrays
  *                     dsdl_uavcan_protocol_file_WriteRequest dyn memory will point to dyn_arr_buf memory.
  *                     NULL will ignore dynamic arrays decoding.
  * @param offset: Call with 0, bit offset to msg storage
  * @param tao: is tail array optimization used
  * @retval offset or ERROR value if < 0
  */
int32_t dsdl_uavcan_protocol_file_WriteRequest_decode_internal(
  const CanardRxTransfer* transfer,
  uint16_t CANARD_MAYBE_UNUSED(payload_len),
  dsdl_uavcan_protocol_file_WriteRequest* dest,
  uint8_t** CANARD_MAYBE_UNUSED(dyn_arr_buf),
  int32_t offset,
  uint8_t CANARD_MAYBE_UNUSED(tao))
{
    int32_t ret = 0;
    uint32_t c = 0;

    ret = canardDecodeScalar(transfer, offset, 40, false, (void*)&dest->offset);
    if (ret != 40)
    {
        goto dsdl_uavcan_protocol_file_WriteRequest_error_exit;
    }
    offset += 40;

    // Compound
    offset = dsdl_uavcan_protocol_file_Path_decode_internal(transfer, 0, (void*)&dest->path, dyn_arr_buf, offset, tao);
    if (offset < 0)
    {
        ret = offset;
        goto dsdl_uavcan_protocol_file_WriteRequest_error_exit;
    }

    // Dynamic Array (data)
    //  - Last item in struct & Root item & (Array Size > 8 bit), tail array optimization
    if (payload_len && tao == CANARD_INTERNAL_ENABLE_TAO)
    {
        //  - Calculate Array length from MSG length
        dest->data.len = ((payload_len * 8) - offset ) / 8; // 8 bit array item size
    }
    else
    {
        // - Array length 8 bits
        ret = canardDecodeScalar(transfer,
                                 offset,
                                 8,
                                 false,
                                 (void*)&dest->data.len); // 255
        if (ret != 8)
        {
            goto dsdl_uavcan_protocol_file_WriteRequest_error_exit;
        }
        offset += 8;
    }

    //  - Get Array
    if (dyn_arr_buf)
    {
        dest->data.data = (uint8_t*)*dyn_arr_buf;
    }

    for (c = 0; c < dest->data.len; c++)
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
                goto dsdl_uavcan_protocol_file_WriteRequest_error_exit;
            }
            *dyn_arr_buf = (uint8_t*)(((uint8_t*)*dyn_arr_buf) + 1);
        }
        offset += 8;
    }
    return offset;

dsdl_uavcan_protocol_file_WriteRequest_error_exit:
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
  * @brief dsdl_uavcan_protocol_file_WriteRequest_decode
  * @param transfer: Pointer to CanardRxTransfer transfer
  * @param payload_len: Payload message length
  * @param dest: Pointer to destination struct
  * @param dyn_arr_buf: NULL or Pointer to memory storage to be used for dynamic arrays
  *                     dsdl_uavcan_protocol_file_WriteRequest dyn memory will point to dyn_arr_buf memory.
  *                     NULL will ignore dynamic arrays decoding.
  * @retval offset or ERROR value if < 0
  */
int32_t dsdl_uavcan_protocol_file_WriteRequest_decode(const CanardRxTransfer* transfer,
  uint16_t payload_len,
  dsdl_uavcan_protocol_file_WriteRequest* dest,
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
        for (uint32_t c = 0; c < sizeof(dsdl_uavcan_protocol_file_WriteRequest); c++)
        {
            ((uint8_t*)dest)[c] = 0x00;
        }

        ret = dsdl_uavcan_protocol_file_WriteRequest_decode_internal(transfer, payload_len, dest, dyn_arr_buf, offset, tao);

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
  * @brief dsdl_uavcan_protocol_file_WriteResponse_encode_internal
  * @param source : pointer to source data struct
  * @param msg_buf: pointer to msg storage
  * @param offset: bit offset to msg storage
  * @param root_item: for detecting if TAO should be used
  * @retval returns offset
  */
uint32_t dsdl_uavcan_protocol_file_WriteResponse_encode_internal(dsdl_uavcan_protocol_file_WriteResponse* source,
  void* msg_buf,
  uint32_t offset,
  uint8_t CANARD_MAYBE_UNUSED(root_item))
{
    // Compound
    offset = dsdl_uavcan_protocol_file_Error_encode_internal((void*)&source->error, msg_buf, offset, 0);

    return offset;
}

/**
  * @brief dsdl_uavcan_protocol_file_WriteResponse_encode
  * @param source : Pointer to source data struct
  * @param msg_buf: Pointer to msg storage
  * @retval returns message length as bytes
  */
uint32_t dsdl_uavcan_protocol_file_WriteResponse_encode(dsdl_uavcan_protocol_file_WriteResponse* source, void* msg_buf)
{
    uint32_t offset = 0;

    offset = dsdl_uavcan_protocol_file_WriteResponse_encode_internal(source, msg_buf, offset, 1);

    return (offset + 7 ) / 8;
}

/**
  * @brief dsdl_uavcan_protocol_file_WriteResponse_decode_internal
  * @param transfer: Pointer to CanardRxTransfer transfer
  * @param payload_len: Payload message length
  * @param dest: Pointer to destination struct
  * @param dyn_arr_buf: NULL or Pointer to memory storage to be used for dynamic arrays
  *                     dsdl_uavcan_protocol_file_WriteResponse dyn memory will point to dyn_arr_buf memory.
  *                     NULL will ignore dynamic arrays decoding.
  * @param offset: Call with 0, bit offset to msg storage
  * @param tao: is tail array optimization used
  * @retval offset or ERROR value if < 0
  */
int32_t dsdl_uavcan_protocol_file_WriteResponse_decode_internal(
  const CanardRxTransfer* transfer,
  uint16_t CANARD_MAYBE_UNUSED(payload_len),
  dsdl_uavcan_protocol_file_WriteResponse* dest,
  uint8_t** CANARD_MAYBE_UNUSED(dyn_arr_buf),
  int32_t offset,
  uint8_t CANARD_MAYBE_UNUSED(tao))
{
    int32_t ret = 0;

    // Compound
    offset = dsdl_uavcan_protocol_file_Error_decode_internal(transfer, 0, (void*)&dest->error, dyn_arr_buf, offset, tao);
    if (offset < 0)
    {
        ret = offset;
        goto dsdl_uavcan_protocol_file_WriteResponse_error_exit;
    }
    return offset;

dsdl_uavcan_protocol_file_WriteResponse_error_exit:
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
  * @brief dsdl_uavcan_protocol_file_WriteResponse_decode
  * @param transfer: Pointer to CanardRxTransfer transfer
  * @param payload_len: Payload message length
  * @param dest: Pointer to destination struct
  * @param dyn_arr_buf: NULL or Pointer to memory storage to be used for dynamic arrays
  *                     dsdl_uavcan_protocol_file_WriteResponse dyn memory will point to dyn_arr_buf memory.
  *                     NULL will ignore dynamic arrays decoding.
  * @retval offset or ERROR value if < 0
  */
int32_t dsdl_uavcan_protocol_file_WriteResponse_decode(const CanardRxTransfer* transfer,
  uint16_t payload_len,
  dsdl_uavcan_protocol_file_WriteResponse* dest,
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
        for (uint32_t c = 0; c < sizeof(dsdl_uavcan_protocol_file_WriteResponse); c++)
        {
            ((uint8_t*)dest)[c] = 0x00;
        }

        ret = dsdl_uavcan_protocol_file_WriteResponse_decode_internal(transfer, payload_len, dest, dyn_arr_buf, offset, tao);

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
#endif // __DSDL_UAVCAN_PROTOCOL_FILE_WRITE