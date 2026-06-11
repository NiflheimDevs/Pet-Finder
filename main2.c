
/*****************************************************************************
*  Copyright Statement:
*  --------------------
*  This software is protected by Copyright and the information contained
*  herein is confidential. The software may not be copied and the information
*  contained herein may not be used or disclosed except with the written
*  permission of Quectel Co., Ltd. 2013
*
*****************************************************************************/

#ifndef __PETFINDER__   //have to be changed to ifdef(but the coloring changes)

#include "ql_type.h"
//RIL feature
#include "custom_feature_def.h"
#include "ril.h"
#include "ril_network.h"
#include "ril_gps.h"
//gprs-related apis
#include "ql_gprs.h"
#include "ql_socket.h"
//for message loop
#include "ql_system.h"
//debug
#include "ql_error.h"
#include "ql_stdlib.h"
#include "ql_trace.h"
//mqtt
#include "ril_mqtt.h"
//timer
#include "ql_timer.h"


/*===========================================================================
 * [2] Configuration - everything you might want to change is here
 *==========================================================================*/
#define APN      ""
#define APN_USER   ""
#define APN_PASS   ""

//MQTT Broker setting
#define MQTT_HOST ""
#define MQTT_PORT 1883
#define MQTT_CLIENT_ID      "petfinder-01"       /* must be unique on broker*/
#define MQTT_USERNAME       NULL                 /* NULL,NULL = anonymous   */
#define MQTT_PASSWORD       NULL
#define MQTT_PUB_TOPIC      "petfinder/01/loc"

#define PUBLISH_INTERVAL_MS 10 * 1000
#define RESET_INTERVAL_MS   24 * 60 * 60 * 1000 /* 24 h reboot */


// Timer IDs
#define TIMER_STATE_MACHINE_ID   101
#define TIMER_PUBLISH_ID         102
#define TIMER_RESET_ID           103


/*===========================================================================
 * STATE MACHINE
 *===========================================================================*/
typedef enum {
    STATE_BOOT = 0,
    STATE_WAIT_SIM,
    STATE_WAIT_GSM,
    STATE_WAIT_GPRS,
    STATE_PDP_ACTIVATING,
    STATE_MQTT_OPENING,
    STATE_MQTT_CONNECTING,
    STATE_READY_PUBLISH
} Enum_AppState;

static Enum_AppState m_current_state = STATE_BOOT;
static u32 m_publish_timer_interval = PUBLISH_INTERVAL_MS;
static Enum_ConnectID m_mqtt_conn_id = ConnectID_0; // Usually 0 for first profile
static u32 m_mqtt_msg_id = 0;


static void state_machine_process();
static void acquire_and_send_location();

static void Timer_Callback(u32 timerId, void* param){
    switch(timerId){
        case TIMER_STATE_MACHINE_ID:
            state_machine_process();
            break;
        case TIMER_PUBLISH_ID:
            if(m_current_state == STATE_READY_PUBLISH){
                acquire_and_send_location();
            }
            break;
        case TIMER_RESET_ID:
            //24 hours maintenance reboot for longterm stability
            Ql_Debug_Trace("24-hour timer expired, resetting module\r\n");
            Ql_Reset(0);
            break;
        default:
            break;
    }
}


void proc_main_task(s32 taskId){
    ST_MSG msg;

    while(1)
    {
        Ql_OS_GetMessage(&msg);
        switch(msg.message)
        {
            case MSG_ID_RIL_READY:
                Ql_Debug_Trace("RIL is ready\r\n");
                Ql_RIL_Initialize();
                break;
            case MSG_ID_URC_INDICATION:
                {
                    switch(msg.param1)
                    {
                        //simcard state kick off the timer once sim is Ready
                        case URC_SIM_CARD_STATE_IND:
                            {
                                Ql_Debug_Trace("Sim Status : %d\r\n", msg.param2);
                                if(SIM_STAT_READY == msg.param2){
                                    // This is where we will start the state machine later                                }
                                }
                            }
                            break;
                        case URC_GSM_NW_STATE_IND:
                            Ql_Debug_Trace("[PF] GSM network state changed\r\n");
                            break;

                        case URC_GPRS_NW_STATE_IND:
                            Ql_Debug_Trace("[PF] GPRS network state changed\r\n");
                            break;

                        default:    
                            break;
                    }
                }
                break;
            default:
                break;
        }
        
    }
    
}




#endif //__PETFINDER__