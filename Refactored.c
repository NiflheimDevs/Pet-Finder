/*===========================================================================
 * main2.c  –  Pet Finder firmware
 * Target : Quectel MC60  (OpenCPU, ARM7)
 *
 * Build  : set  C_PREDEF=-D __PETFINDER__  in gcc_makefile, then make clean/new
 *===========================================================================*/
#ifdef __PETFINDER__

/*---------------------------------------------------------------------------
 * [1] Includes
 *-------------------------------------------------------------------------*/
// Standard OpenCPU types & utilities
#include "ql_type.h"
#include "ql_stdlib.h"
#include "ql_trace.h"
#include "ql_error.h"
#include "ql_timer.h"
#include "ql_system.h"

// RIL – modem / network layer
#include "ril.h"
#include "ril_network.h"
#include "ril_sim.h"
#include "ril_gps.h"
#include "ril_location.h"
#include "ril_mqtt.h"

// Raw GPRS stack (PDP context lives here)
#include "ql_gprs.h"

/*===========================================================================
 * [2] Configuration  –  everything you need to touch is in this block
 *=========================================================================*/

// ── Cellular ──────────────────────────────────────────────────────────────
#define APN               ""          // e.g. "internet"
#define APN_USER          ""          // leave empty if carrier needs none
#define APN_PASS          ""

// ── MQTT broker ───────────────────────────────────────────────────────────
#define MQTT_HOST         ""          // e.g. "broker.hivemq.com"
#define MQTT_PORT         1883
#define MQTT_CLIENT_ID    "petfinder-01"   // must be unique on the broker
#define MQTT_USERNAME     NULL             // NULL = anonymous
#define MQTT_PASSWORD     NULL
#define MQTT_PUB_TOPIC    "petfinder/loc"

// ── Timing ────────────────────────────────────────────────────────────────
#define PUBLISH_INTERVAL_MS       (5  * 1000UL)         //   5 s between publishes
#define STATE_MACHINE_INTERVAL_MS (1  * 1000UL)         //   1 s health-check tick
#define RESET_INTERVAL_MS         (24UL * 60 * 60 * 1000) // 24 h maintenance reboot

// ── Fault thresholds ──────────────────────────────────────────────────────
#define MAX_PUBLISH_FAILURES   5    // consecutive pub failures → MQTT reconnect
#define MAX_MQTT_FAILURES      3    // consecutive MQTT open/conn failures → PDP reset
#define MAX_PDP_FAILURES       3    // consecutive PDP failures → wait for GPRS again

/*===========================================================================
 * [3] Timer IDs  (must be unique across the whole OpenCPU task)
 *=========================================================================*/
#define TIMER_ID_STATE_MACHINE   101
#define TIMER_ID_PUBLISH         102
#define TIMER_ID_RESET           103

/*===========================================================================
 * [4] State machine
 *
 * IMPORTANT: RIL_MQTT_QMTOPEN / QMTCONN return RIL_AT_SUCCESS just to say
 * "the modem accepted the AT command". The actual result arrives later as a
 * URC (+QMTOPEN / +QMTCONN) handled in the message loop.
 * Never assume connection success from those return values alone — that is
 * why every async command has a matching _WAIT state.
 *=========================================================================*/
typedef enum
{
    // ── Boot sequence ─────────────────────────────────────────────────────
    STATE_BOOT = 0,         // waiting for MSG_ID_RIL_READY
    STATE_WAIT_SIM,         // RIL up       – polling until SIM is ready
    STATE_WAIT_GSM,         // SIM ready    – polling until GSM registered
    STATE_WAIT_GPRS,        // GSM up       – polling until GPRS registered
    STATE_PDP_ACTIVATING,   // GPRS up      – calling OpenPDPContext (blocking)

    // ── MQTT bring-up (each send → wait pair) ────────────────────────────
    STATE_MQTT_CFG,         // PDP up       – configure MQTT version and recv len
    STATE_MQTT_OPENING,     // cfg done     – sending QMTOPEN AT command
    STATE_MQTT_OPEN_WAIT,   // waiting for +QMTOPEN URC from modem
    STATE_MQTT_CONNECTING,  // sending QMTCONN AT command
    STATE_MQTT_CONN_WAIT,   // waiting for +QMTCONN URC from modem

    // ── Normal operation ──────────────────────────────────────────────────
    STATE_PUBLISHING,       // fully connected – publish timer is running

    // ── Recovery ──────────────────────────────────────────────────────────
    STATE_MQTT_DISCONNECTING, // sent QMTDISC – waiting for clean MQTT bye
    STATE_MQTT_CLOSING,       // sent QMTCLOSE – waiting for +QMTCLOSE URC
    STATE_BACKOFF             // cooling down before the next retry attempt
} AppState;

/*===========================================================================
 * [5] Module-level variables
 *=========================================================================*/
static AppState      m_state            = STATE_BOOT;
static Enum_ConnectID m_conn_id         = ConnectID_0;
static u32           m_msg_id           = 0;

// Failure counters for the escalation ladder
static u8            m_pub_fail_count   = 0;
static u8            m_mqtt_fail_count  = 0;
static u8            m_pdp_fail_count   = 0;

// Backoff: where to go after the cooldown expires, and how many ticks left
static AppState      m_backoff_next     = STATE_MQTT_OPENING;
static u8            m_backoff_ticks    = 0;   // each tick = STATE_MACHINE_INTERVAL_MS

// GPS engine guard - prevents calling RIL_GPS_Open more than once
static bool          m_gps_opened       = FALSE;

/*===========================================================================
 * [6] Forward declarations
 *=========================================================================*/
static void state_machine_tick(void);
static void acquire_and_send_location(void);
static void teardown_mqtt(void);
static void teardown_pdp(void);
static void enter_backoff(u8 ticks, AppState next_state);

/*===========================================================================
 * [7] Helper functions
 *=========================================================================*/

/*---------------------------------------------------------------------------
 * enter_backoff()
 *
 * Call this from any failure point. The state machine will sit in
 * STATE_BACKOFF for (ticks x STATE_MACHINE_INTERVAL_MS) milliseconds,
 * then automatically flip to next_state.
 *
 * Examples:
 *   enter_backoff(5,  STATE_MQTT_OPENING);  //  5s cooldown, retry MQTT
 *   enter_backoff(15, STATE_WAIT_GPRS);     // 15s cooldown, wait for GPRS
 *-------------------------------------------------------------------------*/
static void enter_backoff(u8 ticks, AppState next_state)
{
    Ql_Debug_Trace("[BACKOFF] waiting %d ticks before state %d\r\n",
                   ticks, (s32)next_state);
    m_backoff_ticks = ticks;
    m_backoff_next  = next_state;
    m_state         = STATE_BACKOFF;
}

/*---------------------------------------------------------------------------
 * teardown_mqtt()
 *
 * Gracefully tears down the MQTT layer only - PDP context stays alive.
 * Always call this before teardown_pdp(), never skip straight to QMTCLOSE.
 *
 * Sequence: stop publish timer -> QMTDISC (polite bye to broker) ->
 *           flip to STATE_MQTT_DISCONNECTING so the state machine tick
 *           can follow up with QMTCLOSE once DISC is done.
 *-------------------------------------------------------------------------*/
static void teardown_mqtt(void)
{
    Ql_Debug_Trace("[MQTT] tearing down MQTT layer\r\n");

    // Stop the publish loop immediately - no more publishes during teardown
    Ql_Timer_Stop(TIMER_ID_PUBLISH);

    // Ask broker to close the session cleanly (sends MQTT DISCONNECT packet)
    // We don't check the return value here - if DISC fails we still proceed
    // to CLOSE, the broker will time us out on its end anyway
    RIL_MQTT_QMTDISC(m_conn_id);

    m_state = STATE_MQTT_DISCONNECTING;
}

/*---------------------------------------------------------------------------
 * teardown_pdp()
 *
 * Tears down everything: MQTT first, then the PDP context underneath.
 * After this the module has no data connectivity - state machine will
 * wait for GPRS re-registration before climbing back up.
 *-------------------------------------------------------------------------*/
static void teardown_pdp(void)
{
    Ql_Debug_Trace("[PDP] tearing down PDP context\r\n");

    // Clean up MQTT first (stops publish timer, sends DISC)
    teardown_mqtt();

    // Close the PDP context - module loses IP connectivity here
    RIL_NW_ClosePDPContext();

    // Increment failure counter and escalate if threshold is reached
    m_pdp_fail_count++;
    if (m_pdp_fail_count >= MAX_PDP_FAILURES)
    {
        Ql_Debug_Trace("[PDP] too many PDP failures, resetting module\r\n");
        Ql_Reset(0);
    }

    enter_backoff(15, STATE_WAIT_GPRS); // 15s before trying GPRS again
}

/*===========================================================================
 * [8] Timer callback
 *
 * Single handler for all three timers. Keeps the callback table simple -
 * one registration, one place to read.
 *=========================================================================*/
static void Timer_Callback(u32 timerId, void *param)
{
    switch (timerId)
    {
    case TIMER_ID_STATE_MACHINE:
        state_machine_tick();
        break;

    case TIMER_ID_PUBLISH:
        // Only fire location work when we are actually connected
        if (m_state == STATE_PUBLISHING)
        {
            acquire_and_send_location();
        }
        break;

    case TIMER_ID_RESET:
        // 24-hour maintenance reboot - keeps the module fresh long-term
        Ql_Debug_Trace("[RESET] 24-hour timer expired, rebooting\r\n");
        Ql_Reset(0);
        break;

    default:
        break;
    }
}

/*===========================================================================
 * [9] State machine tick
 *
 * Called every STATE_MACHINE_INTERVAL_MS by TIMER_ID_STATE_MACHINE.
 * Two kinds of states live here:
 *
 *   ACTIVE states  - do work (poll, send a command), then flip state
 *   WAIT states    - just run a timeout watchdog; the URC handler in
 *                    proc_main_task() is responsible for the happy-path
 *                    transition out of these states
 *=========================================================================*/

// Tick counters for wait-state timeouts (reset whenever we enter that state)
static u8 m_open_wait_ticks  = 0;   // ticks spent in STATE_MQTT_OPEN_WAIT
static u8 m_conn_wait_ticks  = 0;   // ticks spent in STATE_MQTT_CONN_WAIT
static u8 m_close_wait_ticks = 0;   // ticks spent in STATE_MQTT_CLOSING

// How many ticks before we give up waiting for a URC (30 x 1s = 30s)
#define MQTT_OPEN_TIMEOUT_TICKS  30
#define MQTT_CONN_TIMEOUT_TICKS  30
#define MQTT_CLOSE_TIMEOUT_TICKS 15

static void state_machine_tick(void)
{
    s32 ret;
    s32 gsm_state  = 0;
    s32 gprs_state = 0;

    switch (m_state)
    {
    /*----------------------------------------------------------------------
     * Boot - nothing to do here, we are waiting for MSG_ID_RIL_READY URC
     * which will flip us to STATE_WAIT_SIM in proc_main_task
     *--------------------------------------------------------------------*/
    case STATE_BOOT:
        break;

    /*----------------------------------------------------------------------
     * Wait for SIM - poll every tick until SIM_STAT_READY
     *--------------------------------------------------------------------*/
    case STATE_WAIT_SIM:
    {
        s32 sim_state = 0;
        ret = RIL_SIM_GetSimState(&sim_state);
        if (ret == RIL_AT_SUCCESS && sim_state == SIM_STAT_READY)
        {
            Ql_Debug_Trace("[SM] SIM ready\r\n");
            m_state = STATE_WAIT_GSM;
        }
        break;
    }

    /*----------------------------------------------------------------------
     * Wait for GSM registration
     * Once registered: power on GPS engine in the background so it gets
     * a head start on acquiring satellites while we bring up data
     *--------------------------------------------------------------------*/
    case STATE_WAIT_GSM:
        ret = RIL_NW_GetGSMState(&gsm_state);
        if (gsm_state == NW_STAT_REGISTERED ||
            gsm_state == NW_STAT_REGISTERED_ROAMING)
        {
            Ql_Debug_Trace("[SM] GSM registered\r\n");
            if (!m_gps_opened)
            {
                RIL_GPS_Open(1); // background - don't block on this
                m_gps_opened = TRUE;
            }
            m_state = STATE_WAIT_GPRS;
        }
        break;

    /*----------------------------------------------------------------------
     * Wait for GPRS registration
     * If we lose GSM entirely fall back - no point waiting for GPRS
     * if the base station is gone
     *--------------------------------------------------------------------*/
    case STATE_WAIT_GPRS:
        ret = RIL_NW_GetGPRSState(&gprs_state);
        if (gprs_state == NW_STAT_REGISTERED ||
            gprs_state == NW_STAT_REGISTERED_ROAMING)
        {
            Ql_Debug_Trace("[SM] GPRS registered\r\n");
            m_state = STATE_PDP_ACTIVATING;
        }
        else if (gprs_state == NW_STAT_NOT_REGISTERED)
        {
            // Lost GSM entirely - fall all the way back
            Ql_Debug_Trace("[SM] GSM lost while waiting for GPRS\r\n");
            m_state = STATE_WAIT_GSM;
        }
        break;

    /*----------------------------------------------------------------------
     * Activate PDP context (blocking call)
     * Set context + APN first, then open. On failure back off and retry,
     * teardown_pdp() will escalate to a module reset after MAX_PDP_FAILURES
     *--------------------------------------------------------------------*/
    case STATE_PDP_ACTIVATING:
        Ql_Debug_Trace("[SM] activating PDP context\r\n");
        RIL_NW_SetGPRSContext(0);
        RIL_NW_SetAPN(1, APN, APN_USER, APN_PASS);
        ret = RIL_NW_OpenPDPContext();
        if (ret == RIL_AT_SUCCESS)
        {
            Ql_Debug_Trace("[SM] PDP context active\r\n");
            m_pdp_fail_count = 0;       // reset PDP failure counter on success
            m_state = STATE_MQTT_CFG;
        }
        else
        {
            Ql_Debug_Trace("[SM] PDP activation failed (ret=%d)\r\n", ret);
            m_pdp_fail_count++;
            if (m_pdp_fail_count >= MAX_PDP_FAILURES)
            {
                Ql_Debug_Trace("[SM] too many PDP failures, resetting\r\n");
                Ql_Reset(0);
            }
            enter_backoff(15, STATE_WAIT_GPRS);
        }
        break;

    /*----------------------------------------------------------------------
     * Configure MQTT client before opening the socket.
     * These two calls must happen after PDP is active and before QMTOPEN:
     *   Showrecvlen  - makes the modem include payload length in the recv URC
     *   Version      - select MQTT 3.1.1 (more widely supported than 3.1.0)
     * Both are synchronous and fast - no wait state needed.
     * On failure we backoff and retry cfg, not the whole PDP stack.
     *--------------------------------------------------------------------*/
    case STATE_MQTT_CFG:
    {
        s32 cfg_ret;
        // Must be called before QMTOPEN per the SDK example
        RIL_MQTT_QMTCFG_Showrecvlen(m_conn_id, ShowFlag_1);

        cfg_ret = RIL_MQTT_QMTCFG_Version_Select(m_conn_id, Version_3_1_1);
        if (cfg_ret == RIL_AT_SUCCESS)
        {
            Ql_Debug_Trace("[SM] MQTT cfg done (v3.1.1, show recv len)\r\n");
            m_state = STATE_MQTT_OPENING;
        }
        else
        {
            Ql_Debug_Trace("[SM] MQTT cfg failed (ret=%d), retrying\r\n", cfg_ret);
            enter_backoff(3, STATE_MQTT_CFG);
        }
        break;
    }

    /*----------------------------------------------------------------------
     * Send QMTOPEN then immediately enter the wait state.
     * The URC handler will advance us to STATE_MQTT_CONNECTING on success.
     *--------------------------------------------------------------------*/
    case STATE_MQTT_OPENING:
        Ql_Debug_Trace("[SM] opening MQTT socket\r\n");
        ret = RIL_MQTT_QMTOPEN(m_conn_id, (u8 *)MQTT_HOST, MQTT_PORT);
        if (ret == RIL_AT_SUCCESS)
        {
            // Command accepted by modem - now wait for +QMTOPEN URC
            m_open_wait_ticks = 0;
            m_state = STATE_MQTT_OPEN_WAIT;
        }
        else
        {
            Ql_Debug_Trace("[SM] QMTOPEN command rejected (ret=%d)\r\n", ret);
            m_mqtt_fail_count++;
            if (m_mqtt_fail_count >= MAX_MQTT_FAILURES)
            {
                Ql_Debug_Trace("[SM] too many MQTT failures, tearing down PDP\r\n");
                m_mqtt_fail_count = 0;
                teardown_pdp();
            }
            else
            {
                enter_backoff(5, STATE_MQTT_OPENING);
            }
        }
        break;

    /*----------------------------------------------------------------------
     * WAIT state - just run the timeout watchdog.
     * Happy path: URC handler sets m_state = STATE_MQTT_CONNECTING
     * Timeout   : give up, increment failure counter, backoff
     *--------------------------------------------------------------------*/
    case STATE_MQTT_OPEN_WAIT:
        m_open_wait_ticks++;
        if (m_open_wait_ticks >= MQTT_OPEN_TIMEOUT_TICKS)
        {
            Ql_Debug_Trace("[SM] +QMTOPEN URC timeout\r\n");
            m_mqtt_fail_count++;
            if (m_mqtt_fail_count >= MAX_MQTT_FAILURES)
            {
                Ql_Debug_Trace("[SM] too many MQTT failures, tearing down PDP\r\n");
                m_mqtt_fail_count = 0;
                teardown_pdp();
            }
            else
            {
                RIL_MQTT_QMTCLOSE(m_conn_id); // clean up stale socket
                enter_backoff(5, STATE_MQTT_OPENING);
            }
        }
        break;

    /*----------------------------------------------------------------------
     * Send QMTCONN then immediately enter the wait state.
     * The URC handler will advance us to STATE_PUBLISHING on success.
     *--------------------------------------------------------------------*/
    case STATE_MQTT_CONNECTING:
        Ql_Debug_Trace("[SM] sending MQTT CONNECT\r\n");
        ret = RIL_MQTT_QMTCONN(m_conn_id,
                                (u8 *)MQTT_CLIENT_ID,
                                (u8 *)MQTT_USERNAME,
                                (u8 *)MQTT_PASSWORD);
        if (ret == RIL_AT_SUCCESS)
        {
            // Command accepted - now wait for +QMTCONN URC
            m_conn_wait_ticks = 0;
            m_state = STATE_MQTT_CONN_WAIT;
        }
        else
        {
            Ql_Debug_Trace("[SM] QMTCONN command rejected (ret=%d)\r\n", ret);
            m_mqtt_fail_count++;
            if (m_mqtt_fail_count >= MAX_MQTT_FAILURES)
            {
                m_mqtt_fail_count = 0;
                teardown_pdp();
            }
            else
            {
                teardown_mqtt(); // close socket, backoff, retry open
            }
        }
        break;

    /*----------------------------------------------------------------------
     * WAIT state - just run the timeout watchdog.
     * Happy path: URC handler sets m_state = STATE_PUBLISHING
     * Timeout   : tear down MQTT and retry from OPENING
     *--------------------------------------------------------------------*/
    case STATE_MQTT_CONN_WAIT:
        m_conn_wait_ticks++;
        if (m_conn_wait_ticks >= MQTT_CONN_TIMEOUT_TICKS)
        {
            Ql_Debug_Trace("[SM] +QMTCONN URC timeout\r\n");
            m_mqtt_fail_count++;
            if (m_mqtt_fail_count >= MAX_MQTT_FAILURES)
            {
                m_mqtt_fail_count = 0;
                teardown_pdp();
            }
            else
            {
                teardown_mqtt();
            }
        }
        break;

    /*----------------------------------------------------------------------
     * Normal running state - just health check GPRS every tick.
     * Publish work happens in acquire_and_send_location() via publish timer,
     * not here - this case only detects connectivity loss.
     *--------------------------------------------------------------------*/
    case STATE_PUBLISHING:
        ret = RIL_NW_GetGPRSState(&gprs_state);
        if (gprs_state != NW_STAT_REGISTERED &&
            gprs_state != NW_STAT_REGISTERED_ROAMING)
        {
            Ql_Debug_Trace("[SM] GPRS lost during publishing, tearing down\r\n");
            teardown_pdp();
        }
        break;

    /*----------------------------------------------------------------------
     * We sent QMTDISC in teardown_mqtt() - now send QMTCLOSE to drop
     * the TCP socket underneath and move to CLOSING wait
     *--------------------------------------------------------------------*/
    case STATE_MQTT_DISCONNECTING:
        Ql_Debug_Trace("[SM] closing MQTT socket\r\n");
        RIL_MQTT_QMTCLOSE(m_conn_id);
        m_close_wait_ticks = 0;
        m_state = STATE_MQTT_CLOSING;
        break;

    /*----------------------------------------------------------------------
     * WAIT state for QMTCLOSE.
     * Happy path: URC handler +QMTCLOSE sets m_state = STATE_MQTT_OPENING
     * Timeout   : assume closed anyway and move on
     *--------------------------------------------------------------------*/
    case STATE_MQTT_CLOSING:
        m_close_wait_ticks++;
        if (m_close_wait_ticks >= MQTT_CLOSE_TIMEOUT_TICKS)
        {
            Ql_Debug_Trace("[SM] +QMTCLOSE URC timeout, assuming closed\r\n");
            m_mqtt_fail_count++;
            enter_backoff(5, STATE_MQTT_CFG);
        }
        break;

    /*----------------------------------------------------------------------
     * Backoff - count down ticks and flip to m_backoff_next when done
     *--------------------------------------------------------------------*/
    case STATE_BACKOFF:
        if (m_backoff_ticks > 0)
        {
            m_backoff_ticks--;
        }
        else
        {
            Ql_Debug_Trace("[BACKOFF] done, moving to state %d\r\n",
                           (s32)m_backoff_next);
            m_state = m_backoff_next;
        }
        break;

    default:
        break;
    }
}

/*===========================================================================
 * [10] Location acquisition and MQTT publish
 *
 * Called every PUBLISH_INTERVAL_MS by TIMER_ID_PUBLISH.
 * Only runs when m_state == STATE_PUBLISHING (enforced in Timer_Callback).
 *
 * Strategy:
 *   1. Try GPS first  - best accuracy, needs open sky
 *   2. Fall back to cell tower positioning (RIL_GetLocation_Ex) if GPS
 *      has no fix - coarser but works indoors and in urban canyons
 *   3. If both fail - increment failure counter, escalate if needed
 *
 * Payload format:
 *   {"clientId":"petfinder-01","lat":52.123456,"lng":4.123456,"src":"gps"}
 *   {"clientId":"petfinder-01","lat":52.123456,"lng":4.123456,"src":"cell"}
 *=========================================================================*/

// RMC sentence from GPS gives us: time, status, lat, lon, speed, heading
// Buffer needs to hold the full NMEA sentence back from the modem
#define GPS_READ_ITEM       "RMC"
#define GPS_BUFFER_LEN      256
#define PAYLOAD_BUFFER_LEN  160   // enough for the JSON + some headroom

static void acquire_and_send_location(void)
{
    u8   gps_buf[GPS_BUFFER_LEN]     = {0};
    char payload[PAYLOAD_BUFFER_LEN] = {0};
    float latitude  = 0.0f;
    float longitude = 0.0f;
    bool  has_fix   = FALSE;
    const char *src = "gps";
    s32 ret;

    /*----------------------------------------------------------------------
     * Step 1 — Try GPS
     * RIL_GPS_Read returns the raw NMEA RMC sentence in gps_buf.
     * An RMC sentence looks like:
     *   $GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A
     *                  ^ 'A' = active (valid fix), 'V' = void (no fix)
     *                          ^ lat ddmm.mmm  ^ lon dddmm.mmm
     *
     * We check the status field first - if it is 'V' the coordinates are
     * garbage and we must not use them.
     *--------------------------------------------------------------------*/
    Ql_Debug_Trace("[LOC] reading GPS\r\n");
    ret = RIL_GPS_Read((u8 *)GPS_READ_ITEM, gps_buf);

    if (ret == RIL_AT_SUCCESS && Ql_strlen((char *)gps_buf) > 0)
    {
        // Find the status field - second comma-delimited token after $GPRMC
        // Format: $GPRMC,<time>,<status>,<lat>,<N/S>,<lon>,<E/W>,...
        char *p = Ql_strstr((char *)gps_buf, "$GPRMC");
        if (p != NULL)
        {
            char time_buf[16] = {0};
            char status_buf[4] = {0};
            char lat_buf[16]  = {0};
            char ns_buf[4]    = {0};
            char lon_buf[16]  = {0};
            char ew_buf[4]    = {0};

            // Parse the fixed NMEA RMC fields
            s32 parsed = Ql_sscanf(p, "$GPRMC,%[^,],%[^,],%[^,],%[^,],%[^,],%[^,]",
                                   time_buf, status_buf,
                                   lat_buf, ns_buf,
                                   lon_buf, ew_buf);

            if (parsed >= 6 && status_buf[0] == 'A')
            {
                // Convert NMEA ddmm.mmmm format to decimal degrees
                // Latitude:  ddmm.mmmm  → dd + mm.mmmm/60
                // Longitude: dddmm.mmmm → ddd + mm.mmmm/60
                float raw_lat = Ql_atof(lat_buf);
                float raw_lon = Ql_atof(lon_buf);

                int lat_deg = (int)(raw_lat / 100);
                int lon_deg = (int)(raw_lon / 100);

                latitude  = lat_deg + (raw_lat - lat_deg * 100) / 60.0f;
                longitude = lon_deg + (raw_lon - lon_deg * 100) / 60.0f;

                // Apply hemisphere sign
                if (ns_buf[0] == 'S') latitude  = -latitude;
                if (ew_buf[0] == 'W') longitude = -longitude;

                has_fix = TRUE;
                src     = "gps";
                Ql_Debug_Trace("[LOC] GPS fix: %.6f, %.6f\r\n", latitude, longitude);
            }
            else
            {
                Ql_Debug_Trace("[LOC] GPS status V (no fix yet)\r\n");
            }
        }
    }
    else
    {
        Ql_Debug_Trace("[LOC] GPS read failed (ret=%d)\r\n", ret);
    }

    /*----------------------------------------------------------------------
     * Step 2 — Cell tower fallback
     * RIL_GetLocation_Ex is synchronous and uses the QuecLocator service
     * to map the visible cell towers to a lat/lon via Quectel's cloud.
     * Accuracy is ~300m–2km depending on cell density, but it works
     * anywhere with GSM signal — indoors, underground, anywhere.
     *--------------------------------------------------------------------*/
    if (!has_fix)
    {
        ST_LocInfo cell_loc;
        Ql_memset(&cell_loc, 0, sizeof(ST_LocInfo));

        Ql_Debug_Trace("[LOC] no GPS fix, trying cell tower positioning\r\n");
        ret = RIL_GetLocation_Ex(&cell_loc);
        if (ret == RIL_AT_SUCCESS &&
            (cell_loc.latitude != 0.0f || cell_loc.longitude != 0.0f))
        {
            latitude  = cell_loc.latitude;
            longitude = cell_loc.longitude;
            has_fix   = TRUE;
            src       = "cell";
            Ql_Debug_Trace("[LOC] cell fix: %.6f, %.6f\r\n", latitude, longitude);
        }
        else
        {
            Ql_Debug_Trace("[LOC] cell tower positioning failed (ret=%d)\r\n", ret);
        }
    }

    /*----------------------------------------------------------------------
     * Step 3 — Publish if we have a location, escalate if we don't
     *--------------------------------------------------------------------*/
    if (!has_fix)
    {
        // Both sources failed - count it but don't publish garbage
        m_pub_fail_count++;
        Ql_Debug_Trace("[LOC] no location from any source, fail count=%d\r\n",
                       m_pub_fail_count);

        if (m_pub_fail_count >= MAX_PUBLISH_FAILURES)
        {
            // Persistent location failure - something is deeply wrong,
            // tear down MQTT and reconnect from scratch
            Ql_Debug_Trace("[LOC] too many location failures, reconnecting MQTT\r\n");
            m_pub_fail_count = 0;
            teardown_mqtt();
            enter_backoff(5, STATE_MQTT_OPENING);
        }
        return;
    }

    // Build JSON payload
    m_msg_id++;
    if (m_msg_id > 65535) m_msg_id = 1;

    Ql_sprintf(payload,
               "{\"clientId\":\"%s\",\"lat\":%.6f,\"lng\":%.6f,\"src\":\"%s\"}",
               MQTT_CLIENT_ID, latitude, longitude, src);

    Ql_Debug_Trace("[PUB] topic=%s payload=%s\r\n", MQTT_PUB_TOPIC, payload);

    ret = RIL_MQTT_QMTPUB(m_conn_id,
                           m_msg_id,
                           QOS1_AT_LEASET_ONCE,
                           0,                        // retain = false
                           (u8 *)MQTT_PUB_TOPIC,
                           Ql_strlen(payload),
                           (u8 *)payload);

    if (ret == RIL_AT_SUCCESS)
    {
        // Successful publish - reset all failure counters
        m_pub_fail_count  = 0;
        m_mqtt_fail_count = 0;
        Ql_Debug_Trace("[PUB] published successfully (msgId=%d)\r\n", m_msg_id);
    }
    else
    {
        m_pub_fail_count++;
        Ql_Debug_Trace("[PUB] publish failed (ret=%d), fail count=%d\r\n",
                       ret, m_pub_fail_count);

        if (m_pub_fail_count >= MAX_PUBLISH_FAILURES)
        {
            // Broker is not responding - tear down MQTT and reconnect
            Ql_Debug_Trace("[PUB] too many publish failures, reconnecting MQTT\r\n");
            m_pub_fail_count = 0;
            teardown_mqtt();
            enter_backoff(5, STATE_MQTT_OPENING);
        }
    }
}

/*===========================================================================
 * [11] MQTT receive callback
 *
 * Called by the RIL layer when a message arrives on any subscribed topic.
 * Registered once in proc_main_task via Ql_Mqtt_Recv_Register().
 * For Pet Finder we are publish-only so this just logs — extend here if
 * you later want to receive remote commands (e.g. ping, config update).
 *=========================================================================*/
static void on_mqtt_recv(u8 *buffer, u32 length)
{
    Ql_Debug_Trace("[MQTT] incoming message (%d bytes): %s\r\n", length, buffer);
}

/*===========================================================================
 * [12] proc_main_task  –  OpenCPU entry point
 *
 * This is the task entry function called by the RTOS on boot.
 * Responsibilities:
 *   - Register timers and MQTT recv callback once at startup
 *   - Run the message loop forever
 *   - Handle URCs that drive state machine transitions:
 *       URC_SIM_CARD_STATE_IND  → kick off state machine timer
 *       URC_GSM_NW_STATE_IND    → log network changes
 *       URC_GPRS_NW_STATE_IND   → log network changes
 *       URC_MQTT_OPEN           → advance from OPEN_WAIT to CONNECTING
 *       URC_MQTT_CONN           → advance from CONN_WAIT to PUBLISHING
 *       URC_MQTT_CLOSE          → advance from CLOSING to MQTT_OPENING
 *       URC_MQTT_DISC           → log only, CLOSE follows in state machine
 *=========================================================================*/
void proc_main_task(s32 taskId)
{
    ST_MSG msg;

    Ql_Debug_Trace("[BOOT] Pet Finder starting\r\n");

    /*----------------------------------------------------------------------
     * Register all three timers up front.
     * None are started yet — the state machine timer starts when SIM is
     * ready, publish timer starts when MQTT is connected, reset timer
     * starts immediately for the 24-hour watchdog.
     *--------------------------------------------------------------------*/
    Ql_Timer_Register(TIMER_ID_STATE_MACHINE, Timer_Callback, NULL);
    Ql_Timer_Register(TIMER_ID_PUBLISH,       Timer_Callback, NULL);
    Ql_Timer_Register(TIMER_ID_RESET,         Timer_Callback, NULL);

    // 24-hour maintenance reboot - one-shot, starts immediately
    Ql_Timer_Start(TIMER_ID_RESET, RESET_INTERVAL_MS, FALSE);

    // Register MQTT receive callback for any downlink messages
    Ql_Mqtt_Recv_Register(on_mqtt_recv);

    /*----------------------------------------------------------------------
     * Message loop - runs forever
     *--------------------------------------------------------------------*/
    while (TRUE)
    {
        Ql_OS_GetMessage(&msg);

        switch (msg.message)
        {
        /*------------------------------------------------------------------
         * RIL layer is up - initialize it and wait for SIM URC
         *----------------------------------------------------------------*/
        case MSG_ID_RIL_READY:
            Ql_Debug_Trace("[BOOT] RIL ready\r\n");
            Ql_RIL_Initialize();
            m_state = STATE_WAIT_SIM;
            break;

        /*------------------------------------------------------------------
         * URC dispatch
         *----------------------------------------------------------------*/
        case MSG_ID_URC_INDICATION:
        {
            switch (msg.param1)
            {
            /*--------------------------------------------------------------
             * SIM ready - start the state machine polling timer
             * We start it here (on the URC) rather than waiting for the
             * first state machine tick to notice SIM ready, which gives us
             * the fastest possible startup.
             *------------------------------------------------------------*/
            case URC_SIM_CARD_STATE_IND:
                Ql_Debug_Trace("[URC] SIM state: %d\r\n", msg.param2);
                if (SIM_STAT_READY == msg.param2)
                {
                    Ql_Debug_Trace("[URC] SIM ready, starting state machine\r\n");
                    m_state = STATE_WAIT_GSM;
                    Ql_Timer_Start(TIMER_ID_STATE_MACHINE,
                                   STATE_MACHINE_INTERVAL_MS, TRUE);
                }
                break;

            /*--------------------------------------------------------------
             * Network state changes - log them, state machine polls
             * independently so we don't need to act here
             *------------------------------------------------------------*/
            case URC_GSM_NW_STATE_IND:
                Ql_Debug_Trace("[URC] GSM network state changed: %d\r\n",
                               msg.param2);
                break;

            case URC_GPRS_NW_STATE_IND:
                Ql_Debug_Trace("[URC] GPRS network state changed: %d\r\n",
                               msg.param2);
                break;

            /*--------------------------------------------------------------
             * +QMTOPEN URC - result of RIL_MQTT_QMTOPEN
             * param2 is MQTT_Urc_Param_t* - result==0 means success
             *
             * Happy path : flip to STATE_MQTT_CONNECTING
             * Failure    : increment counter, backoff or teardown PDP
             *------------------------------------------------------------*/
            case URC_MQTT_OPEN:
            {
                MQTT_Urc_Param_t *urc = (MQTT_Urc_Param_t *)msg.param2;
                if (urc != NULL && urc->result == 0)
                {
                    Ql_Debug_Trace("[URC] +QMTOPEN success\r\n");
                    m_mqtt_fail_count = 0;
                    m_state = STATE_MQTT_CONNECTING;
                }
                else
                {
                    Ql_Debug_Trace("[URC] +QMTOPEN failed (result=%d)\r\n",
                                   urc ? urc->result : -1);
                    m_mqtt_fail_count++;
                    if (m_mqtt_fail_count >= MAX_MQTT_FAILURES)
                    {
                        m_mqtt_fail_count = 0;
                        teardown_pdp();
                    }
                    else
                    {
                        RIL_MQTT_QMTCLOSE(m_conn_id);
                        enter_backoff(5, STATE_MQTT_OPENING);
                    }
                }
                break;
            }

            /*--------------------------------------------------------------
             * +QMTCONN URC - result of RIL_MQTT_QMTCONN
             *
             * Happy path : start publish timer, flip to STATE_PUBLISHING
             * Failure    : tear down MQTT, retry from OPENING
             *------------------------------------------------------------*/
            case URC_MQTT_CONN:
            {
                MQTT_Urc_Param_t *urc = (MQTT_Urc_Param_t *)msg.param2;
                if (urc != NULL && urc->result == 0)
                {
                    Ql_Debug_Trace("[URC] +QMTCONN success, starting publish timer\r\n");
                    m_mqtt_fail_count = 0;
                    m_pub_fail_count  = 0;
                    m_state = STATE_PUBLISHING;
                    Ql_Timer_Start(TIMER_ID_PUBLISH,
                                   PUBLISH_INTERVAL_MS, TRUE);
                }
                else
                {
                    Ql_Debug_Trace("[URC] +QMTCONN failed (result=%d)\r\n",
                                   urc ? urc->result : -1);
                    m_mqtt_fail_count++;
                    if (m_mqtt_fail_count >= MAX_MQTT_FAILURES)
                    {
                        m_mqtt_fail_count = 0;
                        teardown_pdp();
                    }
                    else
                    {
                        teardown_mqtt();
                    }
                }
                break;
            }

            /*--------------------------------------------------------------
             * +QMTDISC URC - result of RIL_MQTT_QMTDISC
             * We don't act on this - the state machine tick in
             * STATE_MQTT_DISCONNECTING will send QMTCLOSE on the next tick.
             * Logging only.
             *------------------------------------------------------------*/
            case URC_MQTT_DISC:
            {
                MQTT_Urc_Param_t *urc = (MQTT_Urc_Param_t *)msg.param2;
                Ql_Debug_Trace("[URC] +QMTDISC result=%d\r\n",
                               urc ? urc->result : -1);
                break;
            }

            /*--------------------------------------------------------------
             * +QMTCLOSE URC - result of RIL_MQTT_QMTCLOSE
             *
             * Happy path : socket is closed, go back to MQTT_OPENING
             * Failure    : assume closed anyway, same transition
             * Either way : increment mqtt fail counter since we closed
             *              for a reason, backoff before retrying
             *------------------------------------------------------------*/
            case URC_MQTT_CLOSE:
            {
                MQTT_Urc_Param_t *urc = (MQTT_Urc_Param_t *)msg.param2;
                Ql_Debug_Trace("[URC] +QMTCLOSE result=%d\r\n",
                               urc ? urc->result : -1);
                // Whether close succeeded or failed the socket is gone.
                // Go back to CFG - configuration must be re-applied before
                // each new QMTOPEN, not just once at boot.
                enter_backoff(5, STATE_MQTT_CFG);
                break;
            }

            default:
                break;
            }
            break; // MSG_ID_URC_INDICATION
        }

        default:
            break;
        }
    }
}

/*===========================================================================
 * END OF FILE
 *=========================================================================*/

#endif /* __PETFINDER__ */