
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
#include "ril_location.h"
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


/*===========================================================================
 * STATE MACHINE ENGINE
 *===========================================================================*/
static void state_machine_process(){
    s32 ret;
    s32 gsm_state = 0;
    s32 gprs_state = 0;

    switch(m_current_state){
        case STATE_BOOT:
            break;
        case STATE_WAIT_SIM:
            ret = RIL_SIM_GetSimState(&gsm_state);
            if(RIL_AT_SUCCESS == ret && SIM_STAT_READY == gsm_state){
                Ql_Debug_Trace("SIM is ready\r\n");
                m_current_state = STATE_WAIT_GSM;
            }
            break;
        case STATE_WAIT_GSM:
            ret = RIL_NW_GetGSMState(&gsm_state);
            if(gsm_state == NW_STAT_REGISTERED || gsm_state == NW_STAT_REGISTERED_ROAMING){
                Ql_Debug_Trace("GSM network registered\r\n");
                RIL_GPS_Open(1);        // Activate GPS engine in background
                m_current_state = STATE_WAIT_GPRS;
            }
            break;
        case STATE_WAIT_GPRS:
            ret = RIL_NW_GetGPRSState(&gprs_state);
            if(gprs_state == NW_STAT_REGISTERED || gprs_state == NW_STAT_REGISTERED_ROAMING){
                Ql_Debug_Trace("GPRS network registered\r\n");
                // Activate PDP context with APN settings
                RIL_NW_SetGPRSContext(0);
                RIL_NW_SetAPN(1, APN, APN_USER, APN_PASS);
                m_current_state = STATE_PDP_ACTIVATING;
            }
            else if (gprs_state == NW_STAT_NOT_REGISTERED)
            {
                // Fall back to GSM check if cellular registration drops completely
                m_current_state = STATE_WAIT_GSM;
            }
            break;
        case STATE_PDP_ACTIVATING:
            Ql_Debug_Trace("Activating PDP context\r\n");
            ret = RIL_NW_OpenPDPContext();
            if(ret == RIL_AT_SUCCESS){
                Ql_Debug_Trace("PDP context activated\r\n");
                m_current_state = STATE_MQTT_OPENING;
            }
            else{
                Ql_Debug_Trace("Failed to activate PDP context\r\n");
            }
            break;
        case STATE_MQTT_OPENING:
            ret = RIL_MQTT_QMTOPEN(m_mqtt_conn_id, MQTT_HOST, MQTT_PORT);   //do i need teo cast??
            if(ret == RIL_AT_SUCCESS){
                Ql_Debug_Trace("MQTT socket opened\r\n");
                m_current_state = STATE_MQTT_CONNECTING;
            }
            else{
                Ql_Debug_Trace("Failed to open MQTT socket\r\n");
            }
            break;
        case STATE_MQTT_CONNECTING:
            ret = RIL_MQTT_QMTCONN(m_mqtt_conn_id, MQTT_CLIENT_ID, MQTT_USERNAME, MQTT_PASSWORD);    //need cast?
            if(ret == RIL_AT_SUCCESS){
                Ql_Debug_Trace("MQTT CONNECT sent\r\n");
                m_current_state = STATE_READY_PUBLISH;
                Ql_Timer_Start(TIMER_PUBLISH_ID, m_publish_timer_interval, TRUE); // Start publish timer
            }
            else{
                Ql_Debug_Trace("Failed to send MQTT CONNECT\r\n");
                RIL_MQTT_QMTCLOSE(m_mqtt_conn_id);
                m_current_state = STATE_MQTT_OPENING;
            }
            break;
        case STATE_READY_PUBLISH: 
            ret = RIL_NW_GetGPRSState(&gprs_state);
            if (gprs_state != NW_STAT_REGISTERED && gprs_state != NW_STAT_REGISTERED_ROAMING)
            {
                Ql_Debug_Trace("[PF] Connection lost! Pausing publication loop...\r\n");
                Ql_Timer_Stop(TIMER_PUBLISH_ID);
                RIL_MQTT_QMTCLOSE(m_mqtt_conn_id);
                m_current_state = STATE_WAIT_GPRS;
            }
            break;
    }
}



/*===========================================================================
 * TELEMETRY ACQUISITION & TRANSMISSION
 *===========================================================================*/
static void acquire_and_send_location(){
    u8 gps_buffer[128] = {0};
    char payload[150] = {0};
    float latitude = 0.0;
    float longitude = 0.0;
    bool has_fix = FALSE;
    s32 ret;

    Ql_Debug_Trace("Acquiring GPS location\r\n");
    ret = RIL_GPS_Read(1, gps_buffer);
    if(ret == RIL_AT_SUCCESS){
        char *ptr = Ql_strstr((char*)gps_buffer, "+QGPSLOC:");
        if(ptr != NULL){
            char time_buf[16], hdop_buf[16], alt_buf[16], fix_buf[16];
            // Safe parse utility function provided by ril_system
            // Extract Latitude & Longitude tokens out of the response
            s32 parsed = Ql_sscanf(ptr, "+QGPSLOC: %[^,],%f,%f", time_buf, &latitude, &longitude);
            if (parsed >= 3 && latitude != 0.0 && longitude != 0.0)
            {
                has_fix = TRUE;
                Ql_Debug_Trace("[PF] GPS Fix acquired successfully.\r\n");
            }
        }
    }

    // Fall back to Cell-Tower Positioning (QuecLocator) if GPS lacks line of sight
    if (!has_fix)
    {
        ST_LocInfo cell_loc;
        Ql_memset(&cell_loc, 0, sizeof(ST_LocInfo));
        
        Ql_Debug_Trace("[PF] GPS signal weak/indoors. Querying cellular cell towers...\r\n");
        ret = RIL_GetLocation_Ex(&cell_loc);
        if (ret == RIL_AT_SUCCESS)
        {
            latitude = cell_loc.latitude;
            longitude = cell_loc.longitude;
            has_fix = TRUE;
            Ql_Debug_Trace("[PF] Cell-Tower location fallback successful.\r\n");
        }
        else
        {
            Ql_Debug_Trace("[PF] Failed to acquire localization from all sensors (Error: %d).\r\n", ret);
        }
    }

    // Publish telemetry data if location acquisition was successful
    if (has_fix)
    {
        m_mqtt_msg_id++;
        if (m_mqtt_msg_id > 65535) m_mqtt_msg_id = 1;

        // Package measurements into a standardized JSON packet
        Ql_sprintf(payload, "{\"clientId\":\"%s\",\"lat\":%.6f,\"lng\":%.6f}", 
                   MQTT_CLIENT_ID, latitude, longitude);

        Ql_Debug_Trace("[PF] Transmitting packet to topic '%s': %s\r\n", MQTT_PUB_TOPIC, payload);

        // Publish to MQTT Broker using QoS 1 (At Least Once delivery guarantee)
        ret = RIL_MQTT_QMTPUB(m_mqtt_conn_id, m_mqtt_msg_id, QOS1_AT_LEASET_ONCE, 0, 
                             (u8*)MQTT_PUB_TOPIC, Ql_strlen(payload), (u8*)payload);
                             
        if (ret != RIL_AT_SUCCESS)
        {
            Ql_Debug_Trace("[PF] MQTT Publish failure detected (Error: %d). Forcing reconnection...\r\n", ret);
            RIL_MQTT_QMTCLOSE(m_mqtt_conn_id);
            m_current_state = STATE_MQTT_OPENING;
        }
    }


}




void proc_main_task(s32 taskId){
    ST_MSG msg;

    Ql_Debug_Trace("Booting pet Finder");

    Ql_Timer_Register(TIMER_STATE_MACHINE_ID, Timer_Callback, NULL);
    Ql_Timer_Register(TIMER_PUBLISH_ID, Timer_Callback, NULL);
    Ql_Timer_Register(TIMER_RESET_ID, Timer_Callback, NULL);

    Ql_Timer_Start(TIMER_STATE_MACHINE_ID, 2000, TRUE);

    Ql_Timer_Start(TIMER_RESET_ID, RESET_INTERVAL_MS, FALSE);   //should be repeatedly

    while(1)
    {
        Ql_OS_GetMessage(&msg);
        switch(msg.message)
        {
            case MSG_ID_RIL_READY:
                Ql_Debug_Trace("RIL is ready\r\n");
                Ql_RIL_Initialize();
                m_current_state = STATE_WAIT_SIM;
                break;
            case MSG_ID_URC_INDICATION:
                {
                    switch(msg.param1)
                    {
                        //simcard state kick off the timer once sim is Ready
                        case URC_SIM_CARD_STATE_IND:
                            {
                                Ql_Debug_Trace("Sim Status : %d\r\n", msg.param2);
                                if(SIM_STAT_READY == msg.param2 && m_current_state < STATE_WAIT_SIM){
                                    // This is where we will start the state machine later
                                    m_current_state = STATE_WAIT_SIM;                                
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