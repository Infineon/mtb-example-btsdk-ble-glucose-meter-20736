/*
 * Copyright 2016-2022, Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software") is owned by Cypress Semiconductor Corporation
 * or one of its affiliates ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products.  Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 */

/** @file
*
* This file implements the Glucose profile, service, application.
*
* Refer to Bluetooth SIG Glucose Profile 1.0. and Glucose Service
* 1.0 specifications for details.
*
* During initialization the app registers with the LE stack to receive various
* notifications such as connection status change, peer writes etc. When a device is
* successfully bonded, the application saves the peer's Bluetooth Device address to
* the NVRAM. The bonded device can write into client configuration descriptor for
* glucose measurement characteristic to receive notification or indication when
* data is available. That information is also saved in to the NVRAM.
* Simulation allows to send indication or notification on timer or GPIO
* triggering.
*
*
* Features demonstrated
*  - GATT database and Device configuration initialization
*  - Registration with LE stack for various events
*  - Glucose Service 1.0
*  - Processing write requests from the client
*  - Handling client's RACP requests
*  - Send the latest glucose reading or context to the client application.
*
* To demonstrate the app, work through the following steps:
* 1. Plug the WICED eval board into your computer
* 2. Build and download the application (to the WICED board)
* 3. Use the CySmart application or a similar one with glucose meter
*    functionality to connect & bond
* 4. Explore the Service, read/delete/clear RACP records, and write control
*    point value using the client app. Only control point value of 1 is accepted.
* 5. To receive the Glucose measurement and Glucose measurement context
*    notifications from the app - use the application button on the board, after subscribing
*    for the same from the client.
* 6. Press & hold the button down till LED1 blinks and then release it. This causes
*    the app to send the latest glucose reading or context to the client application.
*/

#include "spar_utils.h"
#include "bleprofile.h"
#include "bleapp.h"
#include "blebgm.h"
#include "gpiodriver.h"
#include "lesmpkeys.h"
#include "string.h"
#include "stdio.h"
#include "platform.h"
#include "sparcommon.h"
#include "bleappevent.h"

///////////////////////////////////////////////////////////////////////////////////////////////////
// Forward declarations
///////////////////////////////////////////////////////////////////////////////////////////////////

static void    glucose_meter_create(void);
static int     glucose_meter_write_handler(LEGATTDB_ENTRY_HDR *p);
static void    glucose_meter_advStop(void);
static UINT32  glucose_meter_buttonCb(UINT32 function);
static void    glucose_meter_connection_down();
static void    glucose_meter_connection_up();

// Forward declaration of the ROM function.
extern void   blebgm_appTimerCb(UINT32 arg);
extern void   blebgm_appFineTimerCb(UINT32 arg);
extern void   blebgm_smpBondResult(LESMP_PARING_RESULT  result);
extern void   blebgm_encryptionChanged(HCI_EVT_HDR *evt);
extern void   blebgm_transactionTimeout(void);
extern void   blebgm_create_iopdb();
extern int    blebgmhandleConnParamUpdateRsp(void *l2capHdr);
extern void   blebgm_loadAppData();
extern void   blebgm_reportMeasurement(BLEBGM_GLUCOSE_MEASUREMENT *measurementPtr);
extern void   blebgm_DBInit();

extern void    blebgm_clientConfFlagToCheckAndStore(UINT16 flag, UINT16 flagToCheck, UINT16 flagToStore);
extern UINT16  blebgm_checkClientConfigBeforeRACP();
typedef int (*LEL2CAP_MSGHANDLER)(void*);
extern LEL2CAP_MSGHANDLER lel2cap_handleConnParamUpdateRsp;

extern BLEBGM_GLUCOSE_MEASUREMENT *blebgm_dbGetLastMeasurement(void);
extern BLEBGM_APP_STORAGE  *blebgm_getHostData(UINT8 *adr, UINT8 adrType);
extern int blebgm_setupTargetAdrInScanRsp(void);

//////////////////////////////////////////////////////////////////////////////
//                      global variables
//////////////////////////////////////////////////////////////////////////////

PLACE_IN_DROM const UINT8 glucose_meter_db_data[]=
{
    // GATT service
    PRIMARY_SERVICE_UUID16 (0x0001, UUID_SERVICE_GATT),

    CHARACTERISTIC_UUID16  (0x0002, 0x0003, UUID_CHARACTERISTIC_SERVICE_CHANGED, LEGATTDB_CHAR_PROP_INDICATE, LEGATTDB_PERM_NONE, 4),
        0x00, 0x00, 0x00, 0x00,

    // GAP service
    PRIMARY_SERVICE_UUID16 (0x0014, UUID_SERVICE_GAP),

    CHARACTERISTIC_UUID16 (0x0015, 0x0016, UUID_CHARACTERISTIC_DEVICE_NAME, LEGATTDB_CHAR_PROP_READ, LEGATTDB_PERM_READABLE, 15),
        'L','E',' ','G','l','u','c','o','s','e','m','e','t','e','r',    // "LE Glucosemeter"

    CHARACTERISTIC_UUID16 (0x0017, 0x0018, UUID_CHARACTERISTIC_APPEARANCE, LEGATTDB_CHAR_PROP_READ, LEGATTDB_PERM_READABLE, 2),
        BIT16_TO_8 (APPEARANCE_GENERIC_GLUCOSE_METER),

    // Device Info service
    PRIMARY_SERVICE_UUID16 (0x002d, UUID_SERVICE_DEVICE_INFORMATION),

    CHARACTERISTIC_UUID16 (0x002e, 0x002f, UUID_CHARACTERISTIC_MANUFACTURER_NAME_STRING, LEGATTDB_CHAR_PROP_READ, LEGATTDB_PERM_READABLE, 8),
        'I','n','f','i','n','e','o','n',

    CHARACTERISTIC_UUID16 (0x0030, 0x0031, UUID_CHARACTERISTIC_MODEL_NUMBER_STRING, LEGATTDB_CHAR_PROP_READ, LEGATTDB_PERM_READABLE, 8),
        '1','2','3','4',0x00,0x00,0x00,0x00,

    CHARACTERISTIC_UUID16 (0x0032, 0x0033, UUID_CHARACTERISTIC_SYSTEM_ID, LEGATTDB_CHAR_PROP_READ, LEGATTDB_PERM_READABLE, 8),
        0x00,0x01,0x02,0x03,0x4,0x5,0x6,0x7,

    // Primary service GLUCOSE METER.
    PRIMARY_SERVICE_UUID16 (0x0100, UUID_SERVICE_GLUECOSE_CONCENTRATION),

    CHARACTERISTIC_UUID16 (0x0101, 0x0102, UUID_CHARACTERISTIC_GLUCOSE_MEASUREMENT, LEGATTDB_CHAR_PROP_NOTIFY, LEGATTDB_PERM_READABLE, 3),
        0x00,0x00,0x00,

    CHAR_DESCRIPTOR_UUID16_WRITABLE (0x0103, UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                      LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_CMD |LEGATTDB_PERM_WRITE_REQ, 2),
        0x00,0x00,

    CHARACTERISTIC_UUID16 (0x0104, 0x0105, UUID_CHARACTERISTIC_GLUCOSE_MEASUREMENT_CONTEXT, LEGATTDB_CHAR_PROP_NOTIFY, LEGATTDB_PERM_READABLE, 3),
        0x00,0x00,0x00,

    CHAR_DESCRIPTOR_UUID16_WRITABLE (0x0106, UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                      LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_CMD |LEGATTDB_PERM_WRITE_REQ, 2),
        0x00,0x00,

#ifdef BLE_AUTH_WRITE
    CHARACTERISTIC_UUID16_WRITABLE (0x0107, 0x0108, UUID_CHARACTERISTIC_GLUCOSE_RACP,
                           LEGATTDB_CHAR_PROP_INDICATE | LEGATTDB_CHAR_PROP_WRITE,
                           LEGATTDB_PERM_WRITE_CMD | LEGATTDB_PERM_WRITE_REQ | LEGATTDB_PERM_READABLE |
                           LEGATTDB_PERM_AUTH_WRITABLE | LEGATTDB_PERM_VARIABLE_LENGTH, 18),
#else
    CHARACTERISTIC_UUID16_WRITABLE (0x0107, 0x0108, UUID_CHARACTERISTIC_GLUCOSE_RACP,
                           LEGATTDB_CHAR_PROP_INDICATE | LEGATTDB_CHAR_PROP_WRITE,
                           LEGATTDB_PERM_WRITE_CMD | LEGATTDB_PERM_WRITE_REQ | LEGATTDB_PERM_READABLE | LEGATTDB_PERM_VARIABLE_LENGTH, 18),
#endif
        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
        0x00,0x00,

    CHAR_DESCRIPTOR_UUID16_WRITABLE (0x0109, UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                      LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_CMD |LEGATTDB_PERM_WRITE_REQ, 2),
        0x00,0x00,

    CHARACTERISTIC_UUID16 (0x010a, 0x010b, UUID_CHARACTERISTIC_GLUCOSE_FEATURES, LEGATTDB_CHAR_PROP_READ, LEGATTDB_PERM_READABLE, 2),
        0x04,0x04                  // features that we support.
};

// modify configuration to set 5 seconds idle timeout
const BLE_PROFILE_CFG glucose_meter_cfg =
{
    /* .fine_timer_interval            =*/ 3000, // UINT16 ; //ms
    /* .default_adv                    =*/ 4,    // HIGH_UNDIRECTED_DISCOVERABLE
    /* .button_adv_toggle              =*/ 0,    // pairing button make adv toggle (if 1) or always on (if 0)
    /* .high_undirect_adv_interval     =*/ 32,   // slots
    /* .low_undirect_adv_interval      =*/ 2048, // slots
    /* .high_undirect_adv_duration     =*/ 30,   // seconds
    /* .low_undirect_adv_duration      =*/ 30,  // seconds
    /* .high_direct_adv_interval       =*/ 0,    // seconds
    /* .low_direct_adv_interval        =*/ 0,    // seconds
    /* .high_direct_adv_duration       =*/ 0,    // seconds
    /* .low_direct_adv_duration        =*/ 0,    // seconds
    /* .local_name                     =*/ "LE Glucosemeter",   // [LOCAL_NAME_LEN_MAX];
    /* .cod                            =*/ {BIT16_TO_8(APPEARANCE_GENERIC_GLUCOSE_METER), 0x00}, // [COD_LEN];
    /* .ver                            =*/ "1.00",               // [VERSION_LEN];
#ifdef BLE_SECURITY_REQUEST
    /* .encr_required                  =*/ SECURITY_ENABLED|SECURITY_REQUEST,
#else
    /* .encr_required                  =*/ 0,    // if 1, encryption is needed before sending indication/notification
#endif
    /* .disc_required                  =*/ 1,    // if 1, disconnection after confirmation
    /* .test_enable                    =*/ 1,    // if 1 TEST MODE is enabled
    /* .tx_power_level                 =*/ 0x04, // dbm
    /* .con_idle_timeout               =*/ 100,    // second  0-> no timeout
    /* .powersave_timeout              =*/ 30,    // second  0-> no timeout
    /* .hdl                            =*/ {0x102, 0x105, 0x108, 0x10b, 0x00},                                               // GATT HANDLE number
    /* .serv                           =*/ {UUID_SERVICE_GLUECOSE_CONCENTRATION, UUID_SERVICE_GLUECOSE_CONCENTRATION,
                                            UUID_SERVICE_GLUECOSE_CONCENTRATION, UUID_SERVICE_GLUECOSE_CONCENTRATION, 0x00}, // GATT service UUID
    /* .cha                            =*/ {UUID_CHARACTERISTIC_GLUCOSE_MEASUREMENT, UUID_CHARACTERISTIC_GLUCOSE_MEASUREMENT_CONTEXT,
                                            UUID_CHARACTERISTIC_GLUCOSE_RACP, UUID_CHARACTERISTIC_GLUCOSE_FEATURES, 0x00},   // GATT characteristic UUID
    /* .findme_locator_enable          =*/ 0,    // if 1 Find me locator is enable
    /* .findme_alert_level             =*/ 0,    // alert level of find me
    /* .client_grouptype_enable        =*/ 0,    // if 1 grouptype read can be used
    /* .linkloss_button_enable         =*/ 0,    // if 1 linkloss button is enable
    /* .pathloss_check_interval        =*/ 0,    // second
    /* .alert_interval                 =*/ 0,    // interval of alert
    /* .high_alert_num                 =*/ 0,    // number of alert for each interval
    /* .mild_alert_num                 =*/ 0,    // number of alert for each interval
    /* .status_led_enable              =*/ 1,    // if 1 status LED is enable
    /* .status_led_interval            =*/ 0,    // second
    /* .status_led_con_blink           =*/ 0,    // blink num of connection
    /* .status_led_dir_adv_blink       =*/ 0,    // blink num of dir adv
    /* .status_led_un_adv_blink        =*/ 0,    // blink num of undir adv
    /* .led_on_ms                      =*/ 0,    // led blink on duration in ms
    /* .led_off_ms                     =*/ 0,    // led blink off duration in ms
    /* .buz_on_ms                      =*/ 0,    // buzzer on duration in ms
    /* .button_power_timeout           =*/ 0,    // seconds
    /* .button_client_timeout          =*/ 1,    // seconds
    /* .button_discover_timeout        =*/ 3,    // seconds
    /* .button_filter_timeout          =*/ 0,    // seconds
#ifdef BLE_UART_LOOPBACK_TRACE
    15, //UINT8 button_uart_timeout; // seconds
#endif
};

// This compilation flag enables a dummy database to be used
// for IOP testing purposes. When the real glucose meter functionality are
// available, it should be disabled.
#define BLEBGM_IOP_ADVANCE_DB

#ifdef BLEBGM_IOP_ADVANCE_DB_BIG
#ifdef BLE_P1
#define BLEBGM_IOP_ADVANCE_DB_SIZE  20 //Flash can support 500 //10
#else
#define BLEBGM_IOP_ADVANCE_DB_SIZE  200 //Flash can support 500 //10
#endif
#else
#define BLEBGM_IOP_ADVANCE_DB_SIZE  10
#endif

#define GAP_CONN_PARAM_TIMEOUT 30
typedef struct
{
    // NVRAM storage for application.
    BLEBGM_APP_STORAGE blebgm_appStorage[BLEBGM_MAX_BONDED_HOST];

#ifdef BLEBGM_IOP_ADVANCE_DB
    BLEBGM_GLUCOSE_MEASUREMENT *blebgm_iopData;
    BLEBGM_GLUCOSE_MEASUREMENT_CONTEXT *blebgm_iopContextData;
    int *blebgm_iopDataValid;
#endif

    // current operation.
    int blebgm_racpOperation;
    // current operator.
    int blebgm_racpOperator;
    // current filter type.
    int blebgm_racpFilterType;

    UINT32 blebgm_racpPktFormatStatus;

    // this is the currently reporting sequence number.
    UINT16 blebgm_racpCurSeqNum;
    // use max of 7 bytes to hold the time stamp info.
    UINT8 blebgm_racpFilterMin[TIMESTAMP_LEN];
    // use max of 7 bytes to hold the time stamp info.
    UINT8 blebgm_racpFilterMax[TIMESTAMP_LEN];

    // RACP operation related information.
    BLEBGM_RACP_STATE blebgm_racpState;

    BOOL8 blebgm_racpOperationAbort;

    //this is for counting sent notification
    UINT16 blebgm_racpNotificationCnt;

    // This is the RACP handle.
    int blebgm_measurementHandle;
    int blebgm_contextHandle;
    int blebgm_racpHandle;
    int blebgm_featureHandle;

    int blebgm_measurementCCCHandle;
    int blebgm_contextCCCHandle;
    int blebgm_racpCCCHandle;

    // Currently indicated Measurement.
    BLEBGM_GLUCOSE_MEASUREMENT *blebgm_curMeasureRecPtr;

    UINT32 blebgm_apptimer_count;
    UINT32 blebgm_appfinetimer_count;
    BD_ADDR blebgm_remote_addr;

    UINT8 blebgm_bat_enable;
#ifdef BLE_CONNECTION_PARAMETER_UPDATE
    UINT32 blebgm_conparam_timeout;
    UINT8 blebgm_conparam_timeout_enable;
#endif
    UINT8 blebgm_null_operator_nsrr;
} tBgmAppState;

extern tBgmAppState *bgmAppState;
extern int blebgm_iopDataEntries;

///////////////////////////////////////////////////////////////////////////////////////////////////
// Function definitions
///////////////////////////////////////////////////////////////////////////////////////////////////
// Following structure defines UART configuration
const BLE_PROFILE_PUART_CFG glucose_meter_puart_cfg =
{
    /*.baudrate   =*/ 115200,
    /*.txpin      =*/ PUARTENABLE | GPIO_PIN_UART_TX,
    /*.rxpin      =*/ PUARTENABLE | GPIO_PIN_UART_RX,
};

// Following structure defines GPIO configuration used by the application
const BLE_PROFILE_GPIO_CFG glucose_meter_gpio_cfg =
{
    /*.gpio_pin =*/
    {
    GPIO_PIN_WP,      // This need to be used to enable/disable NVRAM write protect
    GPIO_PIN_BUTTON,  // Button GPIO is configured to trigger either direction of interrupt
    GPIO_PIN_LED,     // LED GPIO, optional to provide visual effects
    GPIO_PIN_BATTERY, // Battery monitoring GPIO. When it is lower than particular level, it will give notification to the application
    GPIO_PIN_BUZZER,  // Buzzer GPIO, optional to provide audio effects
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 // other GPIOs are not used
    },
    /*.gpio_flag =*/
    {
    GPIO_SETTINGS_WP,
    GPIO_SETTINGS_BUTTON,
    GPIO_SETTINGS_LED,
    GPIO_SETTINGS_BATTERY,
    GPIO_SETTINGS_BUZZER,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
    }
};

// Application initialization
APPLICATION_INIT()
{
    // Register the GATT DB, configurations and the application create function.
    bleapp_set_cfg((UINT8 *)glucose_meter_db_data, sizeof(glucose_meter_db_data), (void *)&glucose_meter_cfg,
        (void *)&glucose_meter_puart_cfg, (void *)&glucose_meter_gpio_cfg, glucose_meter_create);

    // BLE_APP_DISABLE_TRACING();     ////// Uncomment to disable all tracing
    BLE_APP_ENABLE_TRACING_ON_PUART();
}

// The application create function.  Do not call blebgm Create because we need to overwrite
// write callback
void glucose_meter_create(void)
{
    ble_trace0("blebgm_Create\n");

    bgmAppState = (tBgmAppState *)cfa_mm_Sbrk(sizeof(tBgmAppState));
    memset(bgmAppState, 0x00, sizeof(tBgmAppState));

    //initialize the default value of bgmAppState
    bgmAppState->blebgm_null_operator_nsrr = 1;
    bgmAppState->blebgm_racpState = BLEBGM_RACP_IDLE;

#ifdef BLEBGM_IOP_ADVANCE_DB
    blebgm_create_iopdb();
#endif
    bgmAppState->blebgm_iopContextData[blebgm_iopDataEntries-1].medication = 0xd030;

    // this would generate the adv payload and scan rsp payload.
    bleprofile_Init(bleprofile_p_cfg);
    bleprofile_GPIOInit(bleprofile_gpio_p_cfg);

    blebgm_DBInit(); //load handle number

    // register connection up and connection down handler.
    bleprofile_regAppEvtHandler(BLECM_APP_EVT_LINK_UP, glucose_meter_connection_up);
    bleprofile_regAppEvtHandler(BLECM_APP_EVT_LINK_DOWN, glucose_meter_connection_down);
    bleprofile_regAppEvtHandler(BLECM_APP_EVT_ADV_TIMEOUT, glucose_meter_advStop);

    // register buttone callback.
    bleprofile_regButtonFunctionCb(glucose_meter_buttonCb);

    // handler for Encryption changed.
    blecm_regEncryptionChangedHandler(blebgm_encryptionChanged);
    // handler for Bond result
    lesmp_regSMPResultCb((LESMP_SINGLE_PARAM_CB) blebgm_smpBondResult);
    {
        extern void lesmp_setReturnInsufficientAuthentication(void);
        lesmp_setReturnInsufficientAuthentication();
    }

#ifdef BLE_CONNECTION_PARAMETER_UPDATE
    lel2cap_handleConnParamUpdateRsp = blebgmhandleConnParamUpdateRsp;
#endif

    // ATT timeout cb
    leatt_regTransactionTimeoutCb((LEATT_NO_PARAM_CB) blebgm_transactionTimeout);

    // use RAM version of the write handler
    legattdb_regWriteHandleCb((LEGATTDB_WRITE_CB)glucose_meter_write_handler);

    bleprofile_regTimerCb(blebgm_appFineTimerCb, blebgm_appTimerCb);
    bleprofile_StartTimer();

    // load the Bonded host info.
    blebgm_loadAppData();

    glucose_meter_connection_down();
}

UINT32 glucose_meter_checkGetRecordformat(UINT8 *ptr, INT32 len)
{
    ble_trace0("glucose_meter_checkGetRecordformat()\n");
    UINT32 status = 0;

    if (len < 2)
    {
        status = BLEBGM_RACP_RSP_CODE_INVALID_OPERATOR;
    }
    else
    {
        switch(ptr[1])
        {
            case BLEBGM_RACP_OPERATOR_NULL:
                status = BLEBGM_RACP_RSP_CODE_INVALID_OPERATOR;
                break;
            case BLEBGM_RACP_OPERATOR_ALL_STORED_RECORDS:
            case BLEBGM_RACP_OPERATOR_FIRST_RECORD:
            case BLEBGM_RACP_OPERATOR_LAST_RECORD:
                if (len > 2)
                {
                    status = BLEBGM_RACP_RSP_CODE_INVALID_OPERAND;
                }
                break;
            case BLEBGM_RACP_OPERATOR_LESS_OR_EQUAL:
            case BLEBGM_RACP_OPERATOR_LARGER_OR_EQUAL:
                if (len > 2)
                {
                    // check the filter type.
                    if ((ptr[2] > BLEBGM_RACP_FILTER_TYPE_FACING_TIME) ||
                        (ptr[2] < BLEBGM_RACP_FILTER_TYPE_SEQUENCE_NUMBER))
                    {
                        // undefined filter type.
                        status = BLEBGM_RACP_RSP_CODE_FILTER_TYPE_NOT_SUPPORTED;
                    }

                    if (ptr[2] == BLEBGM_RACP_FILTER_TYPE_SEQUENCE_NUMBER)
                    {
                        if (len != (3 + 2))
                        {
                            status = BLEBGM_RACP_RSP_CODE_INVALID_OPERAND;
                        }
                    }
                    else if (ptr[2] == BLEBGM_RACP_FILTER_TYPE_FACING_TIME)
                    {
                        if (len != (3 + 7))
                        {
                            status = BLEBGM_RACP_RSP_CODE_INVALID_OPERAND;
                        }
                    }
                }
                else
                {
                    status = BLEBGM_RACP_RSP_CODE_INVALID_OPERAND;
                }
                break;
            case BLEBGM_RACP_OPERATOR_IN_RANGE:
                if (len > 2)
                {
                    // check the filter type.
                    if ((ptr[2] > BLEBGM_RACP_FILTER_TYPE_FACING_TIME) ||
                        (ptr[2] < BLEBGM_RACP_FILTER_TYPE_SEQUENCE_NUMBER))
                    {
                        // undefined filter type.
                        status = BLEBGM_RACP_RSP_CODE_FILTER_TYPE_NOT_SUPPORTED;
                    }

                    if (ptr[2] == BLEBGM_RACP_FILTER_TYPE_SEQUENCE_NUMBER)
                    {
                        if ((len != (3 + 2 * 2)) ||
                            ((ptr[3] + (ptr[4] << 8)) > (ptr[5] + (ptr[6] << 8))))
                        {
                            status = BLEBGM_RACP_RSP_CODE_INVALID_OPERAND;
                        }
                    }
                    else if (ptr[2] == BLEBGM_RACP_FILTER_TYPE_FACING_TIME)
                    {
                        if ((len != (3 + 7 * 2)) ||
                                    (blebgm_getTimeStampInseconds(&ptr[3], 0, 0) > blebgm_getTimeStampInseconds(&ptr[10], 0, 0)))
                        {
                            status = BLEBGM_RACP_RSP_CODE_INVALID_OPERAND;
                        }
                    }
                }
                else
                {
                    status = BLEBGM_RACP_RSP_CODE_INVALID_OPERAND;
                }
                break;
            default:
                // This is out of range.
                status = BLEBGM_RACP_RSP_CODE_OPERATOR_NOT_SUPPORTED;
                break;
        }
    }

    return status;
}

UINT32 glucose_meter_checkRACPformat(UINT8 *ptr, INT32 len)
{
    UINT32 status = 0;

    ble_trace1("chkRacpFmt %d\n", ptr[0]);
    ble_tracen((char *)ptr, len);

    switch(ptr[0])
    {
        case BLEBGM_RACP_REPORT_RECORDS:
            status = glucose_meter_checkGetRecordformat(ptr, len);
            break;

        case BLEBGM_RACP_CLEAR_RECORDS:
            status = glucose_meter_checkGetRecordformat(ptr, len);
            break;

        case BLEBGM_RACP_ABORT_REPORTING:
            status = blebgm_checkAbortReportingformat(ptr, len);
            break;

        case BLEBGM_RACP_REQ_NUM_OF_RECORDS:
            status = glucose_meter_checkGetRecordformat(ptr, len);
            break;

        case BLEBGM_RACP_RSP_NUM_OF_RECORDS:
        case BLEBGM_RACP_RSP_CODE:
        default:
            status = BLEBGM_RACP_RSP_CODE_OP_CODE_NOT_SUPPORTED;

            break;
    }
    return status;
}

int glucose_meter_write_handler(LEGATTDB_ENTRY_HDR *p)
{
    UINT16 handle  = legattdb_getHandle(p);
    int len        = legattdb_getAttrValueLen(p);
    UINT8 *attrPtr = legattdb_getAttrValue(p);

    ble_trace3("GM Write handle =0x%04x, length = 0x%04x racpHandle:%x\n", handle, len, bgmAppState->blebgm_racpHandle);
    ble_tracen((char *) attrPtr, len);

    if (handle == bgmAppState->blebgm_racpHandle)
    {
        if (bleprofile_p_cfg->test_enable)
        {
            BLEBGM_RACP_HDR *racpHdr = (BLEBGM_RACP_HDR *) attrPtr;

            switch (bgmAppState->blebgm_racpState)
            {
                case BLEBGM_RACP_IDLE:
                    // We need to check for Client Characteristic Configuration
                    // before we accept the racp operation.
                    if (!(blebgm_checkClientConfigBeforeRACP()==
                         (BLEBGM_CCC_MEASUREMENT|
                          BLEBGM_CCC_MEASUREMENT_CONTEXT|
                          BLEBGM_CCC_RACP)))
                    {
                        // RACP operation prediction has not met.
                        return BLEBGM_RACP_RSP_CLIENT_CHAR_CONF_IMPROPERLY;
                    }
                    break;

                case BLEBGM_RACP_COMPLETE:
                case BLEBGM_RACP_ACTIVE:
                case BLEBGM_RACP_PEND:
                    // We got another request while busy processing
                    // earlier one. Abort has already been checked so this is
                    // not abort.
                    if (racpHdr->opCode != BLEBGM_RACP_ABORT_REPORTING)
                    {
                        return BLEBGM_RACP_RSP_PROCEDURE_ALREADY_IN_PROGRESS;
                    }
                    break;
                default:
                    //
                    ble_trace0("RACPReqUnknownState\n");
                    break;
            }

            // We need to check the racp operation for packet
            // integrity.
            bgmAppState->blebgm_racpPktFormatStatus = glucose_meter_checkRACPformat(attrPtr, len);

            if (!(bgmAppState->blebgm_racpPktFormatStatus))
            {
                // only if there are no errors we want to proceed with the RACP
                // operation.

                // look for abort first.
                if (racpHdr->opCode == BLEBGM_RACP_ABORT_REPORTING)
                {
                    // Here is an abort.
                    bgmAppState->blebgm_racpOperationAbort = TRUE;
                }
                else
                {
                    // we got a new request.
                    bgmAppState->blebgm_racpState  = BLEBGM_RACP_PEND;
                    // reset count
                    bgmAppState->blebgm_racpNotificationCnt = 0;
                }
            }
        }
    }
    else if (handle == bgmAppState->blebgm_measurementCCCHandle)
    {
        blebgm_clientConfFlagToCheckAndStore(*attrPtr,
                LEATT_CLIENT_CONFIG_NOTIFICATION,
                BLEBGM_APP_FLAG_MEASUREMENT_CONF);
    }
    else if (handle == bgmAppState->blebgm_contextCCCHandle)
    {
        blebgm_clientConfFlagToCheckAndStore(*attrPtr,
                LEATT_CLIENT_CONFIG_NOTIFICATION,
                BLEBGM_APP_FLAG_MEASUREMENT_CONTEXT_CONF);
    }
    else if (handle == bgmAppState->blebgm_racpCCCHandle)
    {
        blebgm_clientConfFlagToCheckAndStore(*attrPtr,
                LEATT_CLIENT_CONFIG_INDICATION ,
                BLEBGM_APP_FLAG_RACP_CONF);
    }

    return 0;
}


void glucose_meter_connection_up(void)
{
    int adrType   = lesmpkeys_getPeerAdrType();
    UINT8 *bdAdr  = lesmpkeys_getPeerAdr();
    BLEBGM_APP_STORAGE *hostData;
    UINT8     measurement = 0;
    UINT8     measurementContext = 0;
    UINT8     racp= 0;
    BLEPROFILE_DB_PDU dbPdu;

#ifdef BLE_CONNECTION_PARAMETER_UPDATE
    lel2cap_sendConnParamUpdateReq(16, 32, 0, 100);

    // save timeout
    bgmAppState->blebgm_conparam_timeout = bgmAppState->blebgm_apptimer_count + GAP_CONN_PARAM_TIMEOUT;
    bgmAppState->blebgm_conparam_timeout_enable = 1;
#endif

    // setup the client configuration.
    hostData = blebgm_getHostData(bdAdr, adrType);

    if (hostData)
    {
        ble_trace0("GotHostData\n");

        if (hostData->misc & BLEBGM_APP_FLAG_MEASUREMENT_CONF)
        {
            // measurement client configuration is set.
            measurement = LEATT_CLIENT_CONFIG_NOTIFICATION;
        }

        if (hostData->misc & BLEBGM_APP_FLAG_MEASUREMENT_CONTEXT_CONF)
        {
            // measurement context client configuration is set.
            measurementContext = LEATT_CLIENT_CONFIG_NOTIFICATION;
        }

        if (hostData->misc & BLEBGM_APP_FLAG_RACP_CONF)
        {
            // RACP client configuration is set.
            racp= LEATT_CLIENT_CONFIG_INDICATION;
        }
    }
    else
    {
         ble_trace0("No HostData\n");
    }

    // 2 bytes.
    dbPdu.len = 2;
    dbPdu.pdu[0] = measurement;
    dbPdu.pdu[1] = 0x00;
    bleprofile_WriteHandle(bgmAppState->blebgm_measurementCCCHandle, &dbPdu);

    dbPdu.pdu[0] = measurementContext;
    bleprofile_WriteHandle(bgmAppState->blebgm_contextCCCHandle, &dbPdu);

    dbPdu.pdu[0] = racp;
    bleprofile_WriteHandle(bgmAppState->blebgm_racpCCCHandle, &dbPdu);

    bleprofile_Discoverable(NO_DISCOVERABLE, NULL);

    // clear this pointer.
    bgmAppState->blebgm_curMeasureRecPtr = NULL;

    // set the state to idle.
    bgmAppState->blebgm_racpState = BLEBGM_RACP_IDLE;

    if ((bleprofile_p_cfg->encr_required&SECURITY_REQUEST) == SECURITY_REQUEST)
    {
        lesmp_sendSecurityRequest();
    }

}

void glucose_meter_connection_down(void)
{
    ble_trace1("DiscReason:%02x\n", emconinfo_getDiscReason());

    // clear this pointer.
    bgmAppState->blebgm_curMeasureRecPtr = NULL;

    // set the state to idle.
    bgmAppState->blebgm_racpState = BLEBGM_RACP_IDLE;

    // We may not want to blindly do this. This function call is used for
    // IOP test cases.
    // Setup the Target Address in the scan response payload.
    blebgm_setupTargetAdrInScanRsp();

    // Mandatory discovery mode
    if (bleprofile_p_cfg->default_adv == MANDATORY_DISCOVERABLE)
    {
        bleprofile_Discoverable(HIGH_UNDIRECTED_DISCOVERABLE, NULL);
    }
    // check NVRAM for previously paired BD_ADDR
    else
    {
        bleprofile_Discoverable(bleprofile_p_cfg->default_adv, NULL);
    }
}

void glucose_meter_advStop(void)
{
    ble_trace0("ADV stop\n");

    glucose_meter_connection_down();
}

UINT32 glucose_meter_buttonCb(UINT32 function)
{

    switch(function)
    {
        case BUTTON_CLIENT:
        ble_trace0("glucose_meter_buttonCb client button\n\n");
#ifdef BLEBGM_IOP_ADVANCE_DB
    {
        BLEBGM_GLUCOSE_MEASUREMENT *measurementPtr = NULL;

        // for IOP test this button will add one more record.
        if (!(bgmAppState->blebgm_iopDataValid[blebgm_iopDataEntries-1]))
        {
            // the last one is availble to reuse.
            // this record becomes valid.
            bgmAppState->blebgm_iopDataValid[blebgm_iopDataEntries-1] = TRUE;

            // increase the sequence number of last record by one.
            bgmAppState->blebgm_iopData[blebgm_iopDataEntries -1].seqNum++;

            bgmAppState->blebgm_iopContextData[blebgm_iopDataEntries -1].seqNum =
                bgmAppState->blebgm_iopData[blebgm_iopDataEntries -1].seqNum;

            ble_trace1("blebgm_buttonCb, last one. put %d idx back.\n", blebgm_iopDataEntries -1);
        }
        else
        {
            int i;

            for (i = blebgm_iopDataEntries-2; i >= 0; i--)
            {
                if (!(bgmAppState->blebgm_iopDataValid[i]))
                {
                    // we find a available spot.
                    int j;
                    // slide all the records down.
                    for (j = i; j < blebgm_iopDataEntries - 1; j++)
                    {

                        ble_trace1("blebgm_buttonCb, %d idx is deleted.\n", i);
                        BT_MEMCPY(&(bgmAppState->blebgm_iopData[j]),
                                &(bgmAppState->blebgm_iopData[j+1]),
                                sizeof(BLEBGM_GLUCOSE_MEASUREMENT));

                        BT_MEMCPY(&(bgmAppState->blebgm_iopContextData[j]),
                                &(bgmAppState->blebgm_iopContextData[j+1]),
                                sizeof(BLEBGM_GLUCOSE_MEASUREMENT_CONTEXT));
                    }
                    // this record becomes valid.
                    bgmAppState->blebgm_iopDataValid[i]=TRUE;

                    break;
                }
            }

            // increase the sequence number of last record by one.
            bgmAppState->blebgm_iopData[blebgm_iopDataEntries -1].seqNum++;

            bgmAppState->blebgm_iopContextData[blebgm_iopDataEntries -1].seqNum =
                bgmAppState->blebgm_iopData[blebgm_iopDataEntries -1].seqNum;
            ble_trace1("blebgm_buttonCb, put %d idx back.\n", blebgm_iopDataEntries -1);
        }

        measurementPtr = blebgm_dbGetLastMeasurement();
        if(measurementPtr)
        {
            blebgm_reportMeasurement(measurementPtr);
        }
    }
#endif
            break;
        default:
            break;
    }

    return 0;
}
