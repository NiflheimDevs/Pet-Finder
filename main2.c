
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




/*===========================================================================
 * STATE MACHINE
 *===========================================================================*/
typedef enum {
    STATE_WAIT_GPRS,    /* Waiting for GPRS to register + PDP to activate */
    STATE_CFG_MQTT,     /* Configure MQTT client (version, recv-len) */
    STATE_OPEN_MQTT,    /* Open TCP socket to broker */
    STATE_CONN_MQTT,    /* Send MQTT CONNECT packet */
    STATE_SUB_MQTT,     /* Subscribe to topic */
    STATE_IDLE,         /* Connected — publish on timer, receive on callback */
    STATE_RECONNECTING  /* Tearing down and restarting from OPEN */
} Enum_AppState;





//APN Config
#define APN      ""
#define USERID   ""
#define PASSWD   ""

//MQTT Broker setting
#define HOST_NAME ""
#define HOST_PORT 1883

//reset In order to keep a stable running state
#define RESET_INTERVAL_MS  24 * 60 * 60 * 1000

//mqtt Client Credentials(for now i wont use)
u8 clientID[] = "\0";
u8 username[] = "\0";
u8 password[] = "\0";


u8 topic[] = "\0";
u8 data[]  = "\0";

//Message ID
u32 pub_message_id = 0;
u32 sub_message_id = 0;

//Timer So we dont need blocking loop
#define MQTT_TIMER_ID         0x200
#define MQTT_TIMER_PERIOD     500   /* ms */

/* URC parameter pointer — filled by the modem for each async event */
MQTT_Urc_Param_t*	  mqtt_urc_param_ptr = NULL;
ST_MQTT_topic_info_t  mqtt_topic_info_t;

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