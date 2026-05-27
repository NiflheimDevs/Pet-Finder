
/*****************************************************************************
*  Copyright Statement:
*  --------------------
*  This software is protected by Copyright and the information contained
*  herein is confidential. The software may not be copied and the information
*  contained herein may not be used or disclosed except with the written
*  permission of Quectel Co., Ltd. 2013
*
*****************************************************************************/

#ifndef __PETFINDER__

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


//GPRS Config
#define APN      ""
#define USERID   ""
#define PASSWD   ""

//MQTT config
#define HOST_NAME ""
#define HOST_PORT 1883


void proc_main_task(s32 taskId){
    ST_MSG msg;

    while(1){
        Ql_OS_GetMessage(&msg);
    }
    
}




#endif //__PETFINDER__