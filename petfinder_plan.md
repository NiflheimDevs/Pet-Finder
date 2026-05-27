# Pet Finder — MC60 OpenCPU Project Plan

## What It Does

- Gets GPS coordinates from GNSS engine
- Falls back to cell tower location (QuecLocator) if GPS has no fix
- Publishes location to an MQTT broker every N seconds
- Resets itself every 24 hours for stability

---

## Code Flow

```
POWER ON
    │
    ├── MSG_ID_RIL_READY → Ql_RIL_Initialize()
    │
    ├── URC_CFUN_STATE_IND
    ├── URC_SIM_CARD_STATE_IND
    ├── URC_GSM_NW_STATE_IND
    ├── URC_GPRS_NW_STATE_IND (NW_STAT_REGISTERED)
    │       │
    │       └── App_Start()
    │               ├── activate GPRS (PDP context)
    │               ├── MQTT connect to broker
    │               ├── start GNSS
    │               ├── start GPS timer (every N seconds)
    │               └── start reset timer (24 hours)
    │
    └── MESSAGE LOOP (sleeps between events)

EVERY N SECONDS (GPS timer):
    ├── try GPS fix
    │     ├── got fix → MQTT publish {lat, lon, src:"gps"}
    │     └── no fix → try QuecLocator
    │                     ├── got cell location → publish {lat, lon, src:"cell"}
    │                     └── no location → publish {status:"no_fix", last_lat, last_lon}
    │
    └── wait for next timer

EVERY 24 HOURS (reset timer):
    └── Ql_Reset(0) → module reboots → flow starts from top

ERROR HANDLING (anytime):
    ├── MQTT disconnected → reconnect
    └── GPRS dropped → re-activate PDP → reconnect MQTT
```

---

## File Structure (single file: custom/main.c)

```
┌─────────────────────────────────────────┐
│  INCLUDES                               │
│  CONFIGURATION (placeholders)           │
│  STATIC VARIABLES                       │
│  CALLBACK DECLARATIONS                  │
│  STARTUP FUNCTION (App_Start)           │
│  GPS FUNCTIONS                          │
│  MQTT WRAPPERS                          │
│  CALLBACK IMPLEMENTATIONS               │
│  ENTRY POINT (proc_main_task)           │
└─────────────────────────────────────────┘
```

Since `example_mqtt.c` exists in the SDK, MQTT is handled by the SDK's RIL MQTT functions — no need to build raw MQTT packets. Refer to `example_mqtt.c` for the exact function names (likely `RIL_MQTT_Open`, `RIL_MQTT_Connect`, `RIL_MQTT_Pub`, etc.) and adapt as needed.

---

## Code Template

```c
// ================================================
// INCLUDES
// ================================================
#include "custom_feature_def.h"
#include "ril.h"
#include "ril_network.h"
#include "ril_gps.h"
#include "ql_system.h"
#include "ql_stdlib.h"
#include "ql_trace.h"
#include "ql_error.h"
#include "ql_gprs.h"
#include "ql_timer.h"
// MQTT-related headers — check example_mqtt.c for actual names
// e.g. #include "ril_mqtt.h"

// ================================================
// CONFIGURATION (fill in later)
// ================================================
#define APN_NAME            "internet"      // TODO: actual APN
#define APN_USER            ""              // TODO: if needed
#define APN_PASS            ""              // TODO: if needed

#define MQTT_BROKER_HOST    "broker.example.com"   // TODO: actual broker
#define MQTT_BROKER_PORT    1883
#define MQTT_CLIENT_ID      "petfinder01"
#define MQTT_USERNAME       ""              // TODO: if broker requires
#define MQTT_PASSWORD       ""              // TODO: if broker requires
#define MQTT_TOPIC          "petfinder/device1/location"

#define PDP_CONTEXT_ID      0
#define GPS_TIMER_ID        1
#define RESET_TIMER_ID      2
#define GPS_INTERVAL_MS     30000           // 30 seconds
#define RESET_INTERVAL_MS   86400000        // 24 hours

// ================================================
// STATIC VARIABLES
// ================================================

// GPRS config
static ST_GprsConfig m_gprsConfig = {
    APN_NAME,
    APN_USER,
    APN_PASS,
    0, NULL, NULL
};

// Last known GPS position
static double m_lastLat     = 0.0;
static double m_lastLon     = 0.0;
static bool   m_hasLastFix  = FALSE;

// State flags
static bool m_gprsReady     = FALSE;
static bool m_mqttConnected = FALSE;

// ================================================
// CALLBACK DECLARATIONS
// ================================================
static void Callback_OnTimer(u32 timerId, void* param);
static void Callback_GPRS_Deactived(u8 contextId, s32 errCode, void* customParam);
// MQTT callbacks if the example uses them

// ================================================
// STARTUP — runs when GPRS is registered
// ================================================
static void App_Start(void)
{
    s32 ret;

    // 1. Register GPRS callback
    ST_PDPContxt_Callback gprsCallback = {
        NULL,
        Callback_GPRS_Deactived
    };
    ret = Ql_GPRS_Register(PDP_CONTEXT_ID, &gprsCallback, NULL);
    if (ret != GPRS_PDP_SUCCESS) {
        Ql_Debug_Trace("GPRS register failed: %d\r\n", ret);
        return;
    }

    // 2. Configure PDP
    ret = Ql_GPRS_Config(PDP_CONTEXT_ID, &m_gprsConfig);
    if (ret != GPRS_PDP_SUCCESS) {
        Ql_Debug_Trace("GPRS config failed: %d\r\n", ret);
        return;
    }

    // 3. Activate PDP context
    ret = Ql_GPRS_ActivateEx(PDP_CONTEXT_ID, TRUE);
    if (ret != GPRS_PDP_SUCCESS) {
        Ql_Debug_Trace("GPRS activate failed: %d\r\n", ret);
        return;
    }
    m_gprsReady = TRUE;
    Ql_Debug_Trace("GPRS active\r\n");

    // 4. MQTT connect (follow example_mqtt.c)
    // TODO: open MQTT client
    // TODO: connect to MQTT_BROKER_HOST:MQTT_BROKER_PORT
    //       with MQTT_CLIENT_ID, MQTT_USERNAME, MQTT_PASSWORD
    //
    // pseudocode:
    //   RIL_MQTT_Open(MQTT_BROKER_HOST, MQTT_BROKER_PORT);
    //   RIL_MQTT_Connect(MQTT_CLIENT_ID, MQTT_USERNAME, MQTT_PASSWORD);
    //   m_mqttConnected = TRUE;

    // 5. Start GNSS
    // TODO: power on GNSS engine
    // pseudocode: RIL_GPS_Open(1);

    // 6. Start GPS polling timer
    Ql_Timer_Register(GPS_TIMER_ID, Callback_OnTimer, NULL);
    Ql_Timer_Start(GPS_TIMER_ID, GPS_INTERVAL_MS, TRUE);

    // 7. Start 24-hour reset timer
    Ql_Timer_Register(RESET_TIMER_ID, Callback_OnTimer, NULL);
    Ql_Timer_Start(RESET_TIMER_ID, RESET_INTERVAL_MS, TRUE);

    Ql_Debug_Trace("Pet finder running\r\n");
}

// ================================================
// GPS — read fix, publish to MQTT (with fallback)
// ================================================
static void GPS_ReadAndPublish(void)
{
    char payload[128];

    // TODO: read GPS fix
    // pseudocode:
    //   if (RIL_GPS_GetPosition(&lat, &lon) == success) {
    //       m_lastLat    = lat;
    //       m_lastLon    = lon;
    //       m_hasLastFix = TRUE;
    //       Ql_sprintf(payload,
    //           "{\"lat\":%f,\"lon\":%f,\"src\":\"gps\"}", lat, lon);
    //       MQTT_Publish(payload);
    //       return;
    //   }

    // TODO: GPS failed → try QuecLocator (cell tower)
    // pseudocode:
    //   if (RIL_QuecLocator_Get(&lat, &lon) == success) {
    //       Ql_sprintf(payload,
    //           "{\"lat\":%f,\"lon\":%f,\"src\":\"cell\"}", lat, lon);
    //       MQTT_Publish(payload);
    //       return;
    //   }

    // Both failed → publish last known + status
    if (m_hasLastFix) {
        Ql_sprintf(payload,
            "{\"status\":\"no_fix\",\"last_lat\":%f,\"last_lon\":%f}",
            m_lastLat, m_lastLon);
    } else {
        Ql_sprintf(payload, "{\"status\":\"no_fix\"}");
    }
    MQTT_Publish(payload);
}

// ================================================
// MQTT WRAPPER
// ================================================
static void MQTT_Publish(char* payload)
{
    if (!m_mqttConnected) {
        Ql_Debug_Trace("MQTT not connected, skipping publish\r\n");
        return;
    }

    // TODO: publish payload to MQTT_TOPIC
    // pseudocode: RIL_MQTT_Pub(MQTT_TOPIC, payload);

    Ql_Debug_Trace("Published: %s\r\n", payload);
}

// ================================================
// CALLBACK IMPLEMENTATIONS
// ================================================
static void Callback_OnTimer(u32 timerId, void* param)
{
    if (timerId == GPS_TIMER_ID) {
        GPS_ReadAndPublish();
    } else if (timerId == RESET_TIMER_ID) {
        Ql_Debug_Trace("24h reset\r\n");
        Ql_Reset(0);
    }
}

static void Callback_GPRS_Deactived(u8 contextId, s32 errCode, void* customParam)
{
    Ql_Debug_Trace("GPRS dropped: %d\r\n", errCode);
    m_gprsReady     = FALSE;
    m_mqttConnected = FALSE;
    // TODO: re-activate PDP, reconnect MQTT
}

// ================================================
// ENTRY POINT
// ================================================
void proc_main_task(s32 taskId)
{
    ST_MSG msg;
    Ql_Debug_Trace("Pet Finder starting\r\n");

    while (TRUE) {
        Ql_OS_GetMessage(&msg);
        switch (msg.message) {

            case MSG_ID_RIL_READY:
                Ql_Debug_Trace("RIL ready\r\n");
                Ql_RIL_Initialize();
                break;

            case MSG_ID_URC_INDICATION:
                switch (msg.param1) {
                    case URC_SYS_INIT_STATE_IND:
                        Ql_Debug_Trace("Sys init: %d\r\n", msg.param2);
                        break;
                    case URC_SIM_CARD_STATE_IND:
                        Ql_Debug_Trace("SIM: %d\r\n", msg.param2);
                        break;
                    case URC_GSM_NW_STATE_IND:
                        Ql_Debug_Trace("GSM: %d\r\n", msg.param2);
                        break;
                    case URC_GPRS_NW_STATE_IND:
                        Ql_Debug_Trace("GPRS: %d\r\n", msg.param2);
                        if (NW_STAT_REGISTERED == msg.param2) {
                            App_Start();
                        }
                        break;
                    case URC_CFUN_STATE_IND:
                        Ql_Debug_Trace("CFUN: %d\r\n", msg.param2);
                        break;
                    default:
                        break;
                }
                break;

            default:
                break;
        }
    }
}
```

---

## TODO Before Filling In

1. Open `example/example_mqtt.c` and copy the exact MQTT function calls (open, connect, publish, disconnect)
2. Ask teacher: All-in-one or Stand-alone GNSS mode? — determines which GPS API to use
3. Fill in real APN, broker host, credentials when known
4. Decide topic format and JSON schema with whoever's building the receiving server