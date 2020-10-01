/**
 * Copyright (c) 2020 Jay Logue
 * All rights reserved.
 *
 *    Licensed under the Apache License, Version 2.0 (the "License");
 *    you may not use this file except in compliance with the License.
 *    You may obtain a copy of the License at
 *
 *        http://www.apache.org/licenses/LICENSE-2.0
 *
 *    Unless required by applicable law or agreed to in writing, software
 *    distributed under the License is distributed on an "AS IS" BASIS,
 *    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *    See the License for the specific language governing permissions and
 *    limitations under the License.
 *
 */

#include <sdk_common.h>

#if defined(SOFTDEVICE_PRESENT) && SOFTDEVICE_PRESENT

#include <inttypes.h>
#include <stdio.h>

#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"
#include "ble.h"
#include "ble_gap.h"
#include "ble_gattc.h"
#include "ble_advdata.h"
#include "ble_srv_common.h"
#include "nrf_ble_gatt.h"
#include "ble_conn_state.h"

#if NRF_LOG_ENABLED
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#endif // NRF_LOG_ENABLED

#include <SampleBLEService.h>
#include <FunctExitUtils.h>

namespace {

const ble_uuid128_t sServiceUUID128    = { { 0x23, 0xD1, 0xBC, 0xEA, 0x5F, 0x78, 0x23, 0x15, 0xDE, 0xEF, 0x12, 0x12, 0x23, 0x15, 0x00, 0x00 } };
const ble_uuid128_t sButtonCharUUID128 = { { 0x23, 0xD1, 0xBC, 0xEA, 0x5F, 0x78, 0x23, 0x15, 0xDE, 0xEF, 0x12, 0x12, 0x24, 0x15, 0x00, 0x00 } };
const ble_uuid128_t sLEDCharUUID128    = { { 0x23, 0xD1, 0xBC, 0xEA, 0x5F, 0x78, 0x23, 0x15, 0xDE, 0xEF, 0x12, 0x12, 0x25, 0x15, 0x00, 0x00 } };

NRF_BLE_GATT_DEF(sGATTModule);
uint8_t sAdvHandle = BLE_GAP_ADV_SET_HANDLE_NOT_SET;
uint8_t sEncodedAdvDataBuf[BLE_GAP_ADV_SET_DATA_SIZE_MAX];
uint16_t sEncodedAdvDataLen;
char sDeviceName[16];
ble_uuid_t sServiceUUID;
ble_uuid_t sLEDCharUUID;
ble_uuid_t sButtonCharUUID;
uint16_t sServiceHandle;
ble_gatts_char_handles_t sLEDCharHandles;
ble_gatts_char_handles_t sButtonCharHandles;

} // unnamed namespace

ret_code_t SampleBLEService::Init(void)
{
    ret_code_t res;

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, BLE_OBSERVER_PRIO, SampleBLEService::HandleBLEEvent, NULL);

    ble_conn_state_init();

    res = SetDeviceName();
    SuccessOrExit(res);

    res = ConfigureGATTService();
    SuccessOrExit(res);

    res = ConfigureAdvertising();
    SuccessOrExit(res);

    res = StartAdvertising();
    SuccessOrExit(res);

exit:
    return res;
}

void SampleBLEService::Shutdown(void)
{

}

void SampleBLEService::UpdateButtonState(bool isPressed)
{
    // For each active connection, generate a notification for the Button characteristic.
    ble_conn_state_for_each_connected(
        [](uint16_t conHandle, void *context)
        {
            ret_code_t res;
            ble_gatts_hvx_params_t hvxParams;
            uint8_t charValue = *((bool *)context) ? 0x01 : 0x00;
            uint16_t charValueLen = 1;

            memset(&hvxParams, 0, sizeof(hvxParams));
            hvxParams.type = BLE_GATT_HVX_NOTIFICATION;
            hvxParams.handle = sButtonCharHandles.value_handle;
            hvxParams.p_data = &charValue;
            hvxParams.p_len = &charValueLen;

            res = sd_ble_gatts_hvx(conHandle, &hvxParams);
            if (res != NRF_SUCCESS)
            {
                NRF_LOG_INFO("Error sending BLE notification (con %" PRIu16 "): 0x%08" PRIX32, conHandle, res);
            }
        },
        &isPressed);
}

ret_code_t SampleBLEService::SetDeviceName(void)
{
    ret_code_t res;
    ble_gap_conn_sec_mode_t secMode;
    ble_gap_addr_t devAddr;

    // Get the device's BLE MAC address
    res = sd_ble_gap_addr_get(&devAddr);
    SuccessOrExit(res);

    // Form a unique device name based on the last digits of the MAC address.
    snprintf(sDeviceName, sizeof(sDeviceName), "%.*s%02" PRIX8 "%02" PRIX8,
             sizeof(sDeviceName) - 5, APP_DEVICE_NAME_PREFIX, devAddr.addr[1], devAddr.addr[0]);
    sDeviceName[sizeof(sDeviceName)-1] = 0;

    // Do not allow device name characteristic to be changed
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&secMode);

    // Configure the device name within the BLE soft device.
    res = sd_ble_gap_device_name_set(&secMode, (const uint8_t *)sDeviceName, strlen(sDeviceName));
    SuccessOrExit(res);

exit:
    return res;
}

ret_code_t SampleBLEService::ConfigureAdvertising(void)
{
    ret_code_t res;
    ble_advdata_t advData;
    ble_gap_adv_data_t gapAdvData;
    ble_gap_adv_params_t gapAdvParams;

    // Form the contents of the advertising packet.
    memset(&advData, 0, sizeof(advData));
    advData.name_type = BLE_ADVDATA_FULL_NAME;
    advData.include_appearance = false;
    advData.flags = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    sEncodedAdvDataLen = sizeof(sEncodedAdvDataBuf);
    res = ble_advdata_encode(&advData, sEncodedAdvDataBuf, &sEncodedAdvDataLen);
    SuccessOrExit(res);

    // Setup parameters controlling how advertising will happen.
    memset(&gapAdvParams, 0, sizeof(gapAdvParams));
    gapAdvParams.properties.type = BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED;
    gapAdvParams.primary_phy     = BLE_GAP_PHY_1MBPS;
    gapAdvParams.duration        = BLE_GAP_ADV_TIMEOUT_GENERAL_UNLIMITED;
    gapAdvParams.filter_policy   = BLE_GAP_ADV_FP_ANY;
    gapAdvParams.interval        = MSEC_TO_UNITS(APP_ADVERTISING_RATE_MS, UNIT_0_625_MS);
    gapAdvParams.duration        = 0; // No timeout

    // Configure an "advertising set" in the BLE soft device with the given data and parameters.
    // If the advertising set doesn't exist, this call will create it and return its handle.
    memset(&gapAdvData, 0, sizeof(gapAdvData));
    gapAdvData.adv_data.p_data = sEncodedAdvDataBuf;
    gapAdvData.adv_data.len = sEncodedAdvDataLen;
    res = sd_ble_gap_adv_set_configure(&sAdvHandle, &gapAdvData, &gapAdvParams);
    SuccessOrExit(res);

exit:
    return res;
}

ret_code_t SampleBLEService::StartAdvertising(void)
{
    ret_code_t res;

    VerifyOrExit(sAdvHandle != BLE_GAP_ADV_SET_HANDLE_NOT_SET, res = NRF_ERROR_INVALID_STATE);

#if NRF_LOG_ENABLED && NRF_LOG_LEVEL >= NRF_LOG_SEVERITY_INFO

    {
        ble_gap_addr_t devAddr;
        char devAddrStr[6*3+1];
        sd_ble_gap_addr_get(&devAddr);
        snprintf(devAddrStr, sizeof(devAddrStr), "%02" PRIX8 ":%02" PRIX8 ":%02" PRIX8 ":%02" PRIX8 ":%02" PRIX8 ":%02" PRIX8,
                 devAddr.addr[5], devAddr.addr[4], devAddr.addr[3], devAddr.addr[2], devAddr.addr[1], devAddr.addr[0]);
        NRF_LOG_INFO("Starting BLE advertising (device name: %s, MAC addr: %s)", sDeviceName, devAddrStr);
    }

#endif

    // Instruct the soft device to start advertising using the configured advertising set.
    res = sd_ble_gap_adv_start(sAdvHandle, BLE_CONN_CONFIG_TAG);
    SuccessOrExit(res);

    // Let other portions of the application know that advertising has started.
    OnAdvertisingStarted();

exit:
    return res;
}

ret_code_t SampleBLEService::ConfigureGATTService(void)
{
    ret_code_t res;
    ble_add_char_params_t addCharParams;
    ble_gatts_value_t value;
    uint8_t zero = 0;

    // Initialize the nRF5 GATT module and set the allowable GATT MTU and GAP packet sizes
    // based on compile-time config values.
    res = nrf_ble_gatt_init(&sGATTModule, NULL);
    SuccessOrExit(res);
    res = nrf_ble_gatt_att_mtu_periph_set(&sGATTModule, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
    SuccessOrExit(res);
    res = nrf_ble_gatt_data_length_set(&sGATTModule, BLE_CONN_HANDLE_INVALID, NRF_SDH_BLE_GAP_DATA_LENGTH);
    SuccessOrExit(res);

    // Register vendor-specific UUIDs
    //     NOTE: An NRF_ERROR_NO_MEM here means the soft device hasn't been configured
    //     with space for enough custom UUIDs.  Typically, this limit is set by overriding
    //     the NRF_SDH_BLE_VS_UUID_COUNT config option.
    res = RegisterVendorUUID(sServiceUUID, sServiceUUID128);
    SuccessOrExit(res);
    res = RegisterVendorUUID(sButtonCharUUID, sButtonCharUUID128);
    SuccessOrExit(res);
    res = RegisterVendorUUID(sLEDCharUUID, sLEDCharUUID128);
    SuccessOrExit(res);

    // Add service
    res = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &sServiceUUID, &sServiceHandle);
    SuccessOrExit(res);

    // Add button characteristic
    memset(&addCharParams, 0, sizeof(addCharParams));
    addCharParams.uuid = sButtonCharUUID.uuid;
    addCharParams.uuid_type = sButtonCharUUID.type;
    addCharParams.init_len = 1;
    addCharParams.max_len = 1;
    addCharParams.char_props.read = 1;
    addCharParams.char_props.notify = 1;
    addCharParams.read_access = SEC_OPEN;
    addCharParams.cccd_write_access = SEC_OPEN;
    res = characteristic_add(sServiceHandle, &addCharParams, &sButtonCharHandles);
    SuccessOrExit(res);

    // Add LED characteristic
    memset(&addCharParams, 0, sizeof(addCharParams));
    addCharParams.uuid = sLEDCharUUID.uuid;
    addCharParams.uuid_type = sLEDCharUUID.type;
    addCharParams.init_len = 1;
    addCharParams.max_len = 1;
    addCharParams.char_props.read = 1;
    addCharParams.char_props.write = 1;
    addCharParams.read_access = SEC_OPEN;
    addCharParams.write_access = SEC_OPEN;
    res = characteristic_add(sServiceHandle, &addCharParams, &sLEDCharHandles);
    SuccessOrExit(res);

    // Set the initial values of the characteristics
    memset(&value, 0, sizeof(value));
    value.len = 1;
    value.p_value = &zero;
    res = sd_ble_gatts_value_set(BLE_CONN_HANDLE_INVALID, sButtonCharHandles.value_handle, &value);
    SuccessOrExit(res);
    res = sd_ble_gatts_value_set(BLE_CONN_HANDLE_INVALID, sLEDCharHandles.value_handle, &value);
    SuccessOrExit(res);

exit:
    return res;
}

void SampleBLEService::HandleBLEEvent(ble_evt_t const * bleEvent, void * context)
{
    ret_code_t res;

    switch (bleEvent->header.evt_id)
    {
    case BLE_GAP_EVT_CONNECTED:

        NRF_LOG_INFO("BLE connection established (con %" PRIu16 ")", bleEvent->evt.gap_evt.conn_handle);

        OnConnectionEstablished();

        // Re-enable advertising if more than one peripheral connection allowed and not at the maximum
        // number of peripheral connections.
#if NRF_SDH_BLE_PERIPHERAL_LINK_COUNT > 1
        if (ble_conn_state_peripheral_conn_count() < NRF_SDH_BLE_PERIPHERAL_LINK_COUNT)
        {
            StartAdvertising();
        }
#endif

        break;

    case BLE_GAP_EVT_DISCONNECTED:

        NRF_LOG_INFO("BLE connection terminated (con %" PRIu16 ", reason 0x%02" PRIx8 ")", bleEvent->evt.gap_evt.conn_handle, bleEvent->evt.gap_evt.params.disconnected.reason);

        OnConnectionTerminated();

        // Re-enable advertising if not at the maximum number of peripheral connections.
        if (ble_conn_state_peripheral_conn_count() < NRF_SDH_BLE_PERIPHERAL_LINK_COUNT)
        {
            StartAdvertising();
        }

        break;

    case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
        // BLE Pairing not supported
        res = sd_ble_gap_sec_params_reply(bleEvent->evt.gap_evt.conn_handle,
                                          BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP,
                                          NULL,
                                          NULL);
        SuccessOrExit(res);
        break;

    case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
    {
        NRF_LOG_INFO("BLE GAP PHY update request (con %" PRIu16 ")", bleEvent->evt.gap_evt.conn_handle);
        const ble_gap_phys_t phys = { BLE_GAP_PHY_AUTO, BLE_GAP_PHY_AUTO };
        res = sd_ble_gap_phy_update(bleEvent->evt.gap_evt.conn_handle, &phys);
        SuccessOrExit(res);
        break;
    }

    case BLE_GATTS_EVT_SYS_ATTR_MISSING:
        res = sd_ble_gatts_sys_attr_set(bleEvent->evt.gatts_evt.conn_handle, NULL, 0, 0);
        SuccessOrExit(res);
        break;

    case BLE_GATTS_EVT_TIMEOUT:
        NRF_LOG_INFO("BLE GATT Server timeout (con %" PRIu16 ")", bleEvent->evt.gatts_evt.conn_handle);
        res = sd_ble_gap_disconnect(bleEvent->evt.gatts_evt.conn_handle,
                                    BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        SuccessOrExit(res);
        break;

    case BLE_GATTS_EVT_WRITE:

        if (bleEvent->evt.gatts_evt.params.write.handle == sLEDCharHandles.value_handle &&
            bleEvent->evt.gatts_evt.params.write.len == 1)
        {
            bool setOn = (bleEvent->evt.gatts_evt.params.write.data[0] != 0);
            NRF_LOG_INFO("LED characteristic write: %s", setOn ? "ON" : "OFF");
            OnLEDWrite(setOn);
        }

        break;

    case BLE_GATTS_EVT_HVC:
//        err = HandleTXComplete(event);
//        SuccessOrExit(err);
        break;

    default:
        break;
    }

exit:
    return;
}

ret_code_t SampleBLEService::RegisterVendorUUID(ble_uuid_t & uuid, const ble_uuid128_t & vendorUUID)
{
    ret_code_t res;
    ble_uuid128_t vendorBaseUUID;
    uint8_t shortUUIDType;

    // Construct a "base" UUID from the given vendor UUID by zeroing out bytes 2 and 3.
    //     NOTE: the SoftDevice API expects UUIDs in little-endian form, so UUID bytes 2 and 3
    //     correspond to offsets 13 and 12 in the array.
    vendorBaseUUID = vendorUUID;
    vendorBaseUUID.uuid128[13] = vendorBaseUUID.uuid128[12] = 0;

    // Register the base UUID with the SoftDevice and get the corresponding "short" UUID type.
    // By registering the base UUID value, instead of the full vendor UUID, we save space in
    // the SoftDevice UUID table by consuming only a single entry when there are multiple vendor
    // UUIDs that vary only in bytes 2 and 3.
    res = sd_ble_uuid_vs_add(&vendorBaseUUID, &shortUUIDType);
    SuccessOrExit(res);

    // Initialize the SoftDevice "short" UUID structure that will be used to refer to this vendor UUID.
    uuid.type = shortUUIDType;
    uuid.uuid = (((uint16_t)vendorUUID.uuid128[13]) << 8) | vendorUUID.uuid128[12];

exit:
    return res;
}




#endif // defined(SOFTDEVICE_PRESENT) && SOFTDEVICE_PRESENT
