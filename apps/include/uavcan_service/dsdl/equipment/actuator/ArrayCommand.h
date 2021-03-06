/*
 * UAVCAN data structure definition for libcanard.
 *
 * Autogenerated, do not edit.
 *
 */

#ifndef __DSDL_UAVCAN_EQUIPMENT_ACTUATOR_ARRAYCOMMAND
#define __DSDL_UAVCAN_EQUIPMENT_ACTUATOR_ARRAYCOMMAND

#include <uavcan_service/libcanard/canard.h>

#ifdef __cplusplus
extern "C"
{
#endif

#include <dsdl/uavcan/equipment/actuator/Command.h>

/******************************* Source text **********************************
#
# Actuator commands.
# The system supports up to 256 actuators; up to 15 of them can be commanded with one message.
#

Command[<=15] commands
******************************************************************************/

/********************* DSDL signature source definition ***********************
dsdl.uavcan.equipment.actuator.ArrayCommand
dsdl.uavcan.equipment.actuator.Command[<=15] commands
******************************************************************************/

#define DSDL_UAVCAN_EQUIPMENT_ACTUATOR_ARRAYCOMMAND_ID     1010
#define DSDL_UAVCAN_EQUIPMENT_ACTUATOR_ARRAYCOMMAND_NAME   "dsdl.uavcan.equipment.actuator.ArrayCommand"
#define DSDL_UAVCAN_EQUIPMENT_ACTUATOR_ARRAYCOMMAND_SIGNATURE (0xD0BEC073C75877F8ULL)

#define DSDL_UAVCAN_EQUIPMENT_ACTUATOR_ARRAYCOMMAND_MAX_SIZE ((484 + 7)/8)

// Constants

#define DSDL_UAVCAN_EQUIPMENT_ACTUATOR_ARRAYCOMMAND_COMMANDS_MAX_LENGTH                  15

typedef struct
{
    // FieldTypes
    struct
    {
        uint8_t    len;                       // Dynamic array length
        dsdl_uavcan_equipment_actuator_Command* data;                      // Dynamic Array 32bit[15] max items
    } commands;

} dsdl_uavcan_equipment_actuator_ArrayCommand;

static inline
uint32_t dsdl_uavcan_equipment_actuator_ArrayCommand_encode(dsdl_uavcan_equipment_actuator_ArrayCommand* source, void* msg_buf);

static inline
int32_t dsdl_uavcan_equipment_actuator_ArrayCommand_decode(const CanardRxTransfer* transfer, uint16_t payload_len, dsdl_uavcan_equipment_actuator_ArrayCommand* dest, uint8_t** dyn_arr_buf);

static inline
uint32_t dsdl_uavcan_equipment_actuator_ArrayCommand_encode_internal(dsdl_uavcan_equipment_actuator_ArrayCommand* source, void* msg_buf, uint32_t offset, uint8_t root_item);

static inline
int32_t dsdl_uavcan_equipment_actuator_ArrayCommand_decode_internal(const CanardRxTransfer* transfer, uint16_t payload_len, dsdl_uavcan_equipment_actuator_ArrayCommand* dest, uint8_t** dyn_arr_buf, int32_t offset, uint8_t tao);

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
  * @brief dsdl_uavcan_equipment_actuator_ArrayCommand_encode_internal
  * @param source : pointer to source data struct
  * @param msg_buf: pointer to msg storage
  * @param offset: bit offset to msg storage
  * @param root_item: for detecting if TAO should be used
  * @retval returns offset
  */
uint32_t dsdl_uavcan_equipment_actuator_ArrayCommand_encode_internal(dsdl_uavcan_equipment_actuator_ArrayCommand* source,
  void* msg_buf,
  uint32_t offset,
  uint8_t CANARD_MAYBE_UNUSED(root_item))
{
    uint32_t c = 0;

    // Dynamic Array (commands)
    if (! root_item)
    {
        // - Add array length
        canardEncodeScalar(msg_buf, offset, 4, (void*)&source->commands.len);
        offset += 4;
    }

    // - Add array items
    for (c = 0; c < source->commands.len; c++)
    {
        offset += dsdl_uavcan_equipment_actuator_Command_encode_internal((void*)&source->commands.data[c], msg_buf, offset, 0);
    }

    return offset;
}

/**
  * @brief dsdl_uavcan_equipment_actuator_ArrayCommand_encode
  * @param source : Pointer to source data struct
  * @param msg_buf: Pointer to msg storage
  * @retval returns message length as bytes
  */
uint32_t dsdl_uavcan_equipment_actuator_ArrayCommand_encode(dsdl_uavcan_equipment_actuator_ArrayCommand* source, void* msg_buf)
{
    uint32_t offset = 0;

    offset = dsdl_uavcan_equipment_actuator_ArrayCommand_encode_internal(source, msg_buf, offset, 1);

    return (offset + 7 ) / 8;
}

/**
  * @brief dsdl_uavcan_equipment_actuator_ArrayCommand_decode_internal
  * @param transfer: Pointer to CanardRxTransfer transfer
  * @param payload_len: Payload message length
  * @param dest: Pointer to destination struct
  * @param dyn_arr_buf: NULL or Pointer to memory storage to be used for dynamic arrays
  *                     dsdl_uavcan_equipment_actuator_ArrayCommand dyn memory will point to dyn_arr_buf memory.
  *                     NULL will ignore dynamic arrays decoding.
  * @param offset: Call with 0, bit offset to msg storage
  * @param tao: is tail array optimization used
  * @retval offset or ERROR value if < 0
  */
int32_t dsdl_uavcan_equipment_actuator_ArrayCommand_decode_internal(
  const CanardRxTransfer* transfer,
  uint16_t CANARD_MAYBE_UNUSED(payload_len),
  dsdl_uavcan_equipment_actuator_ArrayCommand* dest,
  uint8_t** CANARD_MAYBE_UNUSED(dyn_arr_buf),
  int32_t offset,
  uint8_t CANARD_MAYBE_UNUSED(tao))
{
    int32_t ret = 0;
    uint32_t c = 0;

    // Dynamic Array (commands)
    //  - Last item in struct & Root item & (Array Size > 8 bit), tail array optimization
    if (payload_len && tao == CANARD_INTERNAL_ENABLE_TAO)
    {
        //  - Calculate Array length from MSG length
        dest->commands.len = ((payload_len * 8) - offset ) / 32; // 32 bit array item size
    }
    else
    {
        // - Array length 4 bits
        ret = canardDecodeScalar(transfer,
                                 offset,
                                 4,
                                 false,
                                 (void*)&dest->commands.len); // 0
        if (ret != 4)
        {
            goto dsdl_uavcan_equipment_actuator_ArrayCommand_error_exit;
        }
        offset += 4;
    }

    //  - Get Array
    if (dyn_arr_buf)
    {
        dest->commands.data = (dsdl_uavcan_equipment_actuator_Command*)*dyn_arr_buf;
    }

    for (c = 0; c < dest->commands.len; c++)
    {
        offset += dsdl_uavcan_equipment_actuator_Command_decode_internal(transfer,
                                                0,
                                                (void*)&dest->commands.data[c],
                                                dyn_arr_buf,
                                                offset,
                                                tao);
    }
    return offset;

dsdl_uavcan_equipment_actuator_ArrayCommand_error_exit:
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
  * @brief dsdl_uavcan_equipment_actuator_ArrayCommand_decode
  * @param transfer: Pointer to CanardRxTransfer transfer
  * @param payload_len: Payload message length
  * @param dest: Pointer to destination struct
  * @param dyn_arr_buf: NULL or Pointer to memory storage to be used for dynamic arrays
  *                     dsdl_uavcan_equipment_actuator_ArrayCommand dyn memory will point to dyn_arr_buf memory.
  *                     NULL will ignore dynamic arrays decoding.
  * @retval offset or ERROR value if < 0
  */
int32_t dsdl_uavcan_equipment_actuator_ArrayCommand_decode(const CanardRxTransfer* transfer,
  uint16_t payload_len,
  dsdl_uavcan_equipment_actuator_ArrayCommand* dest,
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
        for (uint32_t c = 0; c < sizeof(dsdl_uavcan_equipment_actuator_ArrayCommand); c++)
        {
            ((uint8_t*)dest)[c] = 0x00;
        }

        ret = dsdl_uavcan_equipment_actuator_ArrayCommand_decode_internal(transfer, payload_len, dest, dyn_arr_buf, offset, tao);

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
#endif // __DSDL_UAVCAN_EQUIPMENT_ACTUATOR_ARRAYCOMMAND