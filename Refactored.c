/*===========================================================================
 * main2.c  –  Pet Finder firmware
 * Target : Quectel MC60  (OpenCPU, ARM7)
 *
 * Build  : set  C_PREDEF=-D __PETFINDER__  in gcc_makefile, then make clean/new
 *===========================================================================*/
#ifndef __PETFINDER__

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
#define PUBLISH_INTERVAL_MS       (5  * 1000UL)        //  5 s between publishes
#define STATE_MACHINE_INTERVAL_MS (1   * 1000UL)        //   1 s health-check tick
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
    STATE_MQTT_OPENING,     // sending QMTOPEN AT command
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

/*===========================================================================PART2
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

static bool m_gps_opened = FALSE;

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
                RIL_GPS_Open(1);
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
            m_state = STATE_MQTT_OPENING;
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
            enter_backoff(5, STATE_MQTT_OPENING);
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
 * EOF - acquire_and_send_location() and proc_main_task() in next parts
 *=========================================================================*/


#endif /* __PETFINDER__ */