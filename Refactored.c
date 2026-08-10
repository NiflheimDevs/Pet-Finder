/*===========================================================================
 * petfinder.c  -  Pet Finder firmware
 * Target : Quectel MC60  (OpenCPU, ARM7)
 *
 * Build  : set  C_PREDEF=-D __PETFINDER__  in gcc_makefile, then make clean/new
 *
 * Logs   : UART_PORT1 @ 115200 8N1
 *===========================================================================*/
#ifdef __PETFINDER__

/*---------------------------------------------------------------------------
 * [1] Includes
 *-------------------------------------------------------------------------*/
#include "custom_feature_def.h"
#include "ql_type.h"
#include "ql_stdlib.h"
#include "ql_common.h"
#include "ql_trace.h"
#include "ql_error.h"
#include "ql_timer.h"
#include "ql_system.h"
#include "ql_uart.h"

#include "ril.h"
#include "ril_util.h"
#include "ril_network.h"
#include "ril_sim.h"
#include "ril_gps.h"
#include "ril_location.h"
#include "ril_mqtt.h"

#include "ql_gprs.h"

/*===========================================================================
 * [2] Configuration
 *=========================================================================*/

/* -- Cellular ----------------------------------------------------------- */
#define APN               "mtnirancell"
#define APN_USER          ""
#define APN_PASS          ""

/* -- MQTT broker -------------------------------------------------------- */
#define MQTT_HOST         "45.67.139.65"
#define MQTT_PORT         1883
#define MQTT_PUB_TOPIC    "petfinder/loc"

/*
 * NOTE: these are u8 arrays, NOT #defines set to NULL.
 * The RIL layer calls Ql_strlen() on these pointers when it builds the
 * AT command - passing a real NULL is a null-deref, not "anonymous".
 * If you ever want anonymous, use empty arrays "" - never NULL.
 */
static u8 m_client_id[] = "petfinder-01";
static u8 m_username[]  = "petfinder-01";
static u8 m_password[]  = "123qweasd";

/* -- Timing ------------------------------------------------------------- */
#define PUBLISH_INTERVAL_MS       (5UL * 1000UL)   /* 5 s between publishes  */
#define STATE_MACHINE_INTERVAL_MS (1UL * 1000UL)   /* 1 s health-check tick  */

/* 24 h maintenance reboot, counted in state-machine ticks rather than a
 * timer. An 86,400,000 ms interval is outside what some OpenCPU builds
 * accept for Ql_Timer_Start, so we count instead. */
#define RESET_AFTER_TICKS         (24UL * 60UL * 60UL)   /* 86400 x 1 s      */

/* -- Fault thresholds --------------------------------------------------- */
#define MAX_PUBLISH_FAILURES   5
#define MAX_MQTT_FAILURES      3
#define MAX_PDP_FAILURES       3

/*===========================================================================
 * [3] Debug output
 *
 * The original refactor used Ql_Debug_Trace() only, which goes to the
 * *debug* UART - a different physical port from UART1. This macro (lifted
 * from the SDK examples) writes to UART_PORT1 so you can watch bring-up
 * on the same port you are already wired to.
 *=========================================================================*/
#define DEBUG_ENABLE 1
#if DEBUG_ENABLE > 0
  #define DEBUG_PORT   UART_PORT1
  #define DBG_BUF_LEN  512
  static char DBG_BUFFER[DBG_BUF_LEN];
  #define APP_DEBUG(FORMAT,...) {\
      Ql_memset(DBG_BUFFER, 0, DBG_BUF_LEN);\
      Ql_sprintf(DBG_BUFFER, FORMAT, ##__VA_ARGS__); \
      if (UART_PORT2 == (DEBUG_PORT)) \
      {\
          Ql_Debug_Trace(DBG_BUFFER);\
      } else {\
          Ql_UART_Write((Enum_SerialPort)(DEBUG_PORT), (u8*)(DBG_BUFFER), \
                        Ql_strlen((const char *)(DBG_BUFFER)));\
      }\
  }
#else
  #define APP_DEBUG(FORMAT,...)
#endif

/*===========================================================================
 * [4] Timer IDs
 *
 * MUST be greater than 0xFF - see ql_timer.h, which defines
 * TIMER_ID_USER_START as 0x100. Decimal IDs like 101/102 are 0x65/0x66,
 * below the floor, and Ql_Timer_Register rejects them with
 * QL_RET_ERR_PARAM. The failure is silent at runtime: no timer ever
 * fires and the app hangs after "RIL ready" with no further output.
 *
 * Each task supports at most 10 stack timers. We use 2.
 *=========================================================================*/
#define TIMER_ID_STATE_MACHINE   (TIMER_ID_USER_START + 1)   /* 0x101 */
#define TIMER_ID_PUBLISH         (TIMER_ID_USER_START + 2)   /* 0x102 */

/*===========================================================================
 * [5] State machine
 *
 * IMPORTANT: RIL_MQTT_QMTOPEN / QMTCONN / QMTPUB return RIL_AT_SUCCESS just
 * to say "the modem accepted the AT command". The actual result arrives
 * later as a URC (+QMTOPEN / +QMTCONN / +QMTPUB).
 * Never assume success from those return values alone - that is why every
 * async command has a matching _WAIT state or URC handler.
 *=========================================================================*/
typedef enum
{
    /* -- Boot sequence -------------------------------------------------- */
    STATE_BOOT = 0,         /* waiting for MSG_ID_RIL_READY                 */
    STATE_WAIT_SIM,         /* polling until SIM is ready                   */
    STATE_WAIT_GSM,         /* polling until GSM registered                 */
    STATE_WAIT_GPRS,        /* polling until GPRS registered                */
    STATE_PDP_ACTIVATING,   /* calling OpenPDPContext (blocking)            */

    /* -- MQTT bring-up -------------------------------------------------- */
    STATE_MQTT_CFG,         /* configure MQTT version and recv len          */
    STATE_MQTT_OPENING,     /* sending QMTOPEN                              */
    STATE_MQTT_OPEN_WAIT,   /* waiting for +QMTOPEN URC                     */
    STATE_MQTT_CONNECTING,  /* sending QMTCONN                              */
    STATE_MQTT_CONN_WAIT,   /* waiting for +QMTCONN URC                     */

    /* -- Normal operation ----------------------------------------------- */
    STATE_PUBLISHING,       /* connected - publish timer running            */

    /* -- Recovery ------------------------------------------------------- */
    STATE_MQTT_DISCONNECTING,
    STATE_MQTT_CLOSING,
    STATE_BACKOFF
} AppState;

/*===========================================================================
 * [6] Module-level variables
 *=========================================================================*/
static AppState       m_state           = STATE_BOOT;
static Enum_ConnectID m_conn_id         = ConnectID_0;
static u32            m_msg_id          = 0;

static u8   m_pub_fail_count  = 0;
static u8   m_mqtt_fail_count = 0;
static u8   m_pdp_fail_count  = 0;

static AppState m_backoff_next  = STATE_MQTT_OPENING;
static u16      m_backoff_ticks = 0;

static bool m_gps_opened = FALSE;

/* Uptime tick counter for the 24 h maintenance reboot */
static u32  m_uptime_ticks = 0;

/*
 * Re-entrancy guard.
 * RIL_NW_OpenPDPContext() and RIL_GetLocation_Ex() are synchronous AT
 * calls that can block for tens of seconds. Without this guard a periodic
 * timer can fire again mid-call and re-enter the state machine.
 */
static bool m_busy = FALSE;

/* Buffers kept static, not on the callback stack - OpenCPU task stacks
 * are small and this runs inside a timer callback. */
static u8   m_gps_buf[512];
static char m_payload[192];

/* Tick counters for wait-state timeouts */
static u8 m_open_wait_ticks  = 0;
static u8 m_conn_wait_ticks  = 0;
static u8 m_close_wait_ticks = 0;

#define MQTT_OPEN_TIMEOUT_TICKS  30   /* 30 x 1 s */
#define MQTT_CONN_TIMEOUT_TICKS  30
#define MQTT_CLOSE_TIMEOUT_TICKS 15

/*===========================================================================
 * [7] Forward declarations
 *=========================================================================*/
static void state_machine_tick(void);
static void acquire_and_send_location(void);
static void teardown_mqtt(void);
static void teardown_pdp(void);
static void enter_backoff(u16 ticks, AppState next_state);

/*===========================================================================
 * [8] Helpers - recovery
 *=========================================================================*/

static void enter_backoff(u16 ticks, AppState next_state)
{
    APP_DEBUG("[BACKOFF] waiting %d ticks, then state %d\r\n",
              ticks, (s32)next_state);
    m_backoff_ticks = ticks;
    m_backoff_next  = next_state;
    m_state         = STATE_BACKOFF;
}

/*---------------------------------------------------------------------------
 * teardown_mqtt()
 *
 * Tears down the MQTT layer only - PDP context stays alive.
 * Sends QMTDISC (polite bye) and flips to STATE_MQTT_DISCONNECTING, where
 * the next tick issues QMTCLOSE.
 *-------------------------------------------------------------------------*/
static void teardown_mqtt(void)
{
    APP_DEBUG("[MQTT] tearing down MQTT layer\r\n");

    Ql_Timer_Stop(TIMER_ID_PUBLISH);

    /* Return value ignored on purpose - if DISC fails we still proceed to
     * CLOSE and let the broker time us out on its end. */
    RIL_MQTT_QMTDISC(m_conn_id);

    m_state = STATE_MQTT_DISCONNECTING;
}

/*---------------------------------------------------------------------------
 * teardown_pdp()
 *
 * FIX: the previous version called teardown_mqtt() (which sets state to
 * DISCONNECTING so a later tick can send QMTCLOSE), then immediately
 * overwrote that state with enter_backoff(). QMTCLOSE was never sent and
 * ConnectID_0 stayed allocated, so the next QMTOPEN would be rejected.
 * We now close the socket explicitly and synchronously here.
 *-------------------------------------------------------------------------*/
static void teardown_pdp(void)
{
    APP_DEBUG("[PDP] tearing down PDP context\r\n");

    Ql_Timer_Stop(TIMER_ID_PUBLISH);

    RIL_MQTT_QMTDISC(m_conn_id);    /* polite MQTT DISCONNECT   */
    RIL_MQTT_QMTCLOSE(m_conn_id);   /* free the connect ID      */
    RIL_NW_ClosePDPContext();       /* drop IP connectivity     */

    m_pdp_fail_count++;
    if (m_pdp_fail_count >= MAX_PDP_FAILURES)
    {
        APP_DEBUG("[PDP] too many PDP failures, resetting module\r\n");
        Ql_Reset(0);
    }

    enter_backoff(15, STATE_WAIT_GPRS);
}

/*===========================================================================
 * [9] NMEA parsing - no sscanf, no atof, no float printing
 *
 * Coordinates are carried as signed micro-degrees (1e-6 deg) in s32.
 * Range check: 180 deg = 180,000,000 - comfortably inside s32.
 *=========================================================================*/

/*---------------------------------------------------------------------------
 * nmea_get_field()
 *
 * Copies comma-delimited field `idx` of `s` into `out`.
 * Field 0 is the sentence header ("$GPRMC").
 *
 * Unlike sscanf with %[^,], this handles EMPTY fields correctly, which
 * matters because a no-fix sentence looks like:
 *     $GPRMC,,V,,,,,,,,,,N*53
 * Returns TRUE if the field existed (even if empty).
 *-------------------------------------------------------------------------*/
static bool nmea_get_field(const char *s, u8 idx, char *out, u32 out_size)
{
    u8  field = 0;
    u32 n     = 0;

    if (s == NULL || out == NULL || out_size == 0)
    {
        return FALSE;
    }

    Ql_memset(out, 0, out_size);

    while (*s != '\0' && *s != '\r' && *s != '\n' && *s != '*')
    {
        if (*s == ',')
        {
            field++;
            if (field > idx)
            {
                break;
            }
            s++;
            continue;
        }

        if (field == idx && n < (out_size - 1))
        {
            out[n++] = *s;
        }
        s++;
    }

    out[n] = '\0';
    return (field >= idx) ? TRUE : FALSE;
}

/*---------------------------------------------------------------------------
 * nmea_to_udeg()
 *
 * Converts an NMEA coordinate string to signed micro-degrees.
 *   latitude  is  ddmm.mmmm   (2 degree digits)
 *   longitude is dddmm.mmmm   (3 degree digits)
 *
 * We do not hardcode the digit count - we locate the decimal point and
 * take the two digits before it as minutes, everything earlier as degrees.
 * That works for both forms and for any minute precision.
 *
 * Integer maths only. Worst case intermediate:
 *   minutes_scaled = 599999 -> x100 = 59,999,900   (fits s32)
 *-------------------------------------------------------------------------*/
static bool nmea_to_udeg(const char *s, s32 *out_udeg)
{
    s32 len, dot = -1, i;
    s32 deg_digits;
    s32 deg = 0, min_int = 0, min_frac = 0, frac_div = 1;
    s32 minutes_scaled;

    if (s == NULL || out_udeg == NULL)
    {
        return FALSE;
    }

    len = (s32)Ql_strlen((char *)s);
    if (len < 4)
    {
        return FALSE;   /* empty or truncated field */
    }

    for (i = 0; i < len; i++)
    {
        if (s[i] == '.')
        {
            dot = i;
            break;
        }
    }
    if (dot < 3)
    {
        return FALSE;   /* need at least 1 degree digit + 2 minute digits */
    }

    deg_digits = dot - 2;
    if (deg_digits < 1 || deg_digits > 3)
    {
        return FALSE;
    }

    for (i = 0; i < deg_digits; i++)
    {
        if (s[i] < '0' || s[i] > '9') return FALSE;
        deg = deg * 10 + (s[i] - '0');
    }
    for (i = deg_digits; i < dot; i++)
    {
        if (s[i] < '0' || s[i] > '9') return FALSE;
        min_int = min_int * 10 + (s[i] - '0');
    }

    /* Up to 4 fractional digits of minutes */
    for (i = dot + 1; i < len && (i - dot) <= 4; i++)
    {
        if (s[i] < '0' || s[i] > '9') break;
        min_frac  = min_frac * 10 + (s[i] - '0');
        frac_div *= 10;
    }
    while (frac_div < 10000)
    {
        min_frac *= 10;
        frac_div *= 10;
    }

    /* minutes x 10000, converted to micro-degrees:
     *   udeg = deg*1e6 + (min_scaled/1e4)/60 * 1e6
     *        = deg*1e6 + min_scaled*100/60                                */
    minutes_scaled = min_int * 10000 + min_frac;
    *out_udeg = deg * 1000000 + (minutes_scaled * 100) / 60;

    return TRUE;
}

/*---------------------------------------------------------------------------
 * udeg_to_str()
 *
 * Formats signed micro-degrees as "-12.345678".
 *
 * Deliberately does NOT use %f or %06d - zero-padded and float conversion
 * specifiers are not reliably implemented across OpenCPU sprintf builds,
 * and a silently mangled payload is very hard to debug in the field.
 * Digits are emitted by hand. `out` needs >= 14 bytes.
 *-------------------------------------------------------------------------*/
static void udeg_to_str(s32 udeg, char *out)
{
    s32 whole, frac, i, n = 0;
    s32 divisor = 100000;

    if (udeg < 0)
    {
        out[n++] = '-';
        udeg = -udeg;
    }

    whole = udeg / 1000000;
    frac  = udeg % 1000000;

    /* Integer part - Ql_sprintf handles plain %d reliably */
    {
        char tmp[8];
        Ql_memset(tmp, 0, sizeof(tmp));
        Ql_sprintf(tmp, "%d", whole);
        for (i = 0; tmp[i] != '\0'; i++)
        {
            out[n++] = tmp[i];
        }
    }

    out[n++] = '.';

    /* Six fractional digits, most significant first */
    for (i = 0; i < 6; i++)
    {
        out[n++] = (char)('0' + ((frac / divisor) % 10));
        divisor /= 10;
    }

    out[n] = '\0';
}

/*===========================================================================
 * [10] Timer callback
 *=========================================================================*/
static void Timer_Callback(u32 timerId, void *param)
{
    /* Guard against re-entry while a blocking AT call is in flight */
    if (m_busy)
    {
        return;
    }
    m_busy = TRUE;

    switch (timerId)
    {
    case TIMER_ID_STATE_MACHINE:
        state_machine_tick();
        break;

    case TIMER_ID_PUBLISH:
        if (m_state == STATE_PUBLISHING)
        {
            acquire_and_send_location();
        }
        break;

    default:
        break;
    }

    m_busy = FALSE;
}

/*===========================================================================
 * [11] State machine tick
 *
 * ACTIVE states - do work, then flip state
 * WAIT states   - run a timeout watchdog only; the URC handler in
 *                 proc_main_task() drives the happy-path transition
 *=========================================================================*/
static void state_machine_tick(void)
{
    s32 ret;
    s32 gsm_state  = 0;
    s32 gprs_state = 0;

    /* 24 h maintenance reboot */
    m_uptime_ticks++;
    if (m_uptime_ticks >= RESET_AFTER_TICKS)
    {
        APP_DEBUG("[RESET] 24 h uptime reached, rebooting\r\n");
        Ql_Reset(0);
    }

    switch (m_state)
    {
    case STATE_BOOT:
        /* waiting for MSG_ID_RIL_READY */
        break;

    /*----------------------------------------------------------------------
     * Wait for SIM.
     * This polling path is now genuinely reachable: the state machine timer
     * starts on MSG_ID_RIL_READY, so we no longer depend on catching the
     * SIM URC. If that URC is missed or fired early, we still progress.
     *--------------------------------------------------------------------*/
    case STATE_WAIT_SIM:
    {
        s32 sim_state = 0;
        ret = RIL_SIM_GetSimState(&sim_state);
        if (ret == RIL_AT_SUCCESS && sim_state == SIM_STAT_READY)
        {
            APP_DEBUG("[SM] SIM ready\r\n");
            m_state = STATE_WAIT_GSM;
        }
        break;
    }

    /*----------------------------------------------------------------------
     * Wait for GSM registration.
     * Power the GPS engine on here so it gets a head start acquiring
     * satellites while we bring the data path up.
     *--------------------------------------------------------------------*/
    case STATE_WAIT_GSM:
        ret = RIL_NW_GetGSMState(&gsm_state);
        if (gsm_state == NW_STAT_REGISTERED ||
            gsm_state == NW_STAT_REGISTERED_ROAMING)
        {
            APP_DEBUG("[SM] GSM registered\r\n");
            if (!m_gps_opened)
            {
                s32 gret = RIL_GPS_Open(1);
                APP_DEBUG("[SM] GPS power on, ret=%d\r\n", gret);
                m_gps_opened = TRUE;
            }
            m_state = STATE_WAIT_GPRS;
        }
        break;

    case STATE_WAIT_GPRS:
        ret = RIL_NW_GetGPRSState(&gprs_state);
        if (gprs_state == NW_STAT_REGISTERED ||
            gprs_state == NW_STAT_REGISTERED_ROAMING)
        {
            APP_DEBUG("[SM] GPRS registered\r\n");
            m_state = STATE_PDP_ACTIVATING;
        }
        else if (gprs_state == NW_STAT_NOT_REGISTERED)
        {
            APP_DEBUG("[SM] GSM lost while waiting for GPRS\r\n");
            m_state = STATE_WAIT_GSM;
        }
        break;

    /*----------------------------------------------------------------------
     * Activate PDP context. RIL_NW_OpenPDPContext() is blocking and can
     * take a long time - the m_busy guard in Timer_Callback stops the next
     * tick re-entering us while it runs.
     *--------------------------------------------------------------------*/
    case STATE_PDP_ACTIVATING:
        APP_DEBUG("[SM] activating PDP context (APN=%s)\r\n", APN);
        RIL_NW_SetGPRSContext(0);
        RIL_NW_SetAPN(1, APN, APN_USER, APN_PASS);
        ret = RIL_NW_OpenPDPContext();
        if (ret == RIL_AT_SUCCESS)
        {
            APP_DEBUG("[SM] PDP context active\r\n");
            m_pdp_fail_count = 0;
            m_state = STATE_MQTT_CFG;
        }
        else
        {
            APP_DEBUG("[SM] PDP activation failed (ret=%d)\r\n", ret);
            m_pdp_fail_count++;
            if (m_pdp_fail_count >= MAX_PDP_FAILURES)
            {
                APP_DEBUG("[SM] too many PDP failures, resetting\r\n");
                Ql_Reset(0);
            }
            enter_backoff(15, STATE_WAIT_GPRS);
        }
        break;

    /*----------------------------------------------------------------------
     * Configure MQTT before opening the socket. Must be re-applied before
     * every QMTOPEN, not just once at boot.
     *--------------------------------------------------------------------*/
    case STATE_MQTT_CFG:
    {
        s32 cfg_ret;
        RIL_MQTT_QMTCFG_Showrecvlen(m_conn_id, ShowFlag_1);

        cfg_ret = RIL_MQTT_QMTCFG_Version_Select(m_conn_id, Version_3_1_1);
        if (cfg_ret == RIL_AT_SUCCESS)
        {
            APP_DEBUG("[SM] MQTT cfg done (v3.1.1)\r\n");
            m_state = STATE_MQTT_OPENING;
        }
        else
        {
            APP_DEBUG("[SM] MQTT cfg failed (ret=%d), retrying\r\n", cfg_ret);
            enter_backoff(3, STATE_MQTT_CFG);
        }
        break;
    }

    case STATE_MQTT_OPENING:
        APP_DEBUG("[SM] opening MQTT socket to %s:%d\r\n", MQTT_HOST, MQTT_PORT);
        ret = RIL_MQTT_QMTOPEN(m_conn_id, (u8 *)MQTT_HOST, MQTT_PORT);
        if (ret == RIL_AT_SUCCESS)
        {
            m_open_wait_ticks = 0;
            m_state = STATE_MQTT_OPEN_WAIT;
        }
        else
        {
            APP_DEBUG("[SM] QMTOPEN rejected (ret=%d)\r\n", ret);
            m_mqtt_fail_count++;
            if (m_mqtt_fail_count >= MAX_MQTT_FAILURES)
            {
                m_mqtt_fail_count = 0;
                teardown_pdp();
            }
            else
            {
                enter_backoff(5, STATE_MQTT_OPENING);
            }
        }
        break;

    case STATE_MQTT_OPEN_WAIT:
        m_open_wait_ticks++;
        if (m_open_wait_ticks >= MQTT_OPEN_TIMEOUT_TICKS)
        {
            APP_DEBUG("[SM] +QMTOPEN timeout\r\n");
            m_mqtt_fail_count++;
            if (m_mqtt_fail_count >= MAX_MQTT_FAILURES)
            {
                m_mqtt_fail_count = 0;
                teardown_pdp();
            }
            else
            {
                RIL_MQTT_QMTCLOSE(m_conn_id);
                m_close_wait_ticks = 0;
                m_state = STATE_MQTT_CLOSING;
            }
        }
        break;

    case STATE_MQTT_CONNECTING:
        APP_DEBUG("[SM] sending MQTT CONNECT (id=%s)\r\n", m_client_id);
        ret = RIL_MQTT_QMTCONN(m_conn_id, m_client_id, m_username, m_password);
        if (ret == RIL_AT_SUCCESS)
        {
            m_conn_wait_ticks = 0;
            m_state = STATE_MQTT_CONN_WAIT;
        }
        else
        {
            APP_DEBUG("[SM] QMTCONN rejected (ret=%d)\r\n", ret);
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

    case STATE_MQTT_CONN_WAIT:
        m_conn_wait_ticks++;
        if (m_conn_wait_ticks >= MQTT_CONN_TIMEOUT_TICKS)
        {
            APP_DEBUG("[SM] +QMTCONN timeout\r\n");
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
     * Normal running state - health check only. Publishing happens on the
     * publish timer, not here.
     *--------------------------------------------------------------------*/
    case STATE_PUBLISHING:
        ret = RIL_NW_GetGPRSState(&gprs_state);
        if (gprs_state != NW_STAT_REGISTERED &&
            gprs_state != NW_STAT_REGISTERED_ROAMING)
        {
            APP_DEBUG("[SM] GPRS lost while publishing\r\n");
            teardown_pdp();
        }
        break;

    case STATE_MQTT_DISCONNECTING:
        APP_DEBUG("[SM] closing MQTT socket\r\n");
        RIL_MQTT_QMTCLOSE(m_conn_id);
        m_close_wait_ticks = 0;
        m_state = STATE_MQTT_CLOSING;
        break;

    case STATE_MQTT_CLOSING:
        m_close_wait_ticks++;
        if (m_close_wait_ticks >= MQTT_CLOSE_TIMEOUT_TICKS)
        {
            APP_DEBUG("[SM] +QMTCLOSE timeout, assuming closed\r\n");
            enter_backoff(5, STATE_MQTT_CFG);
        }
        break;

    case STATE_BACKOFF:
        if (m_backoff_ticks > 0)
        {
            m_backoff_ticks--;
        }
        else
        {
            APP_DEBUG("[BACKOFF] done, state %d\r\n", (s32)m_backoff_next);
            m_state = m_backoff_next;
        }
        break;

    default:
        break;
    }
}

/*===========================================================================
 * [12] Location acquisition and MQTT publish
 *
 * Strategy:
 *   1. GPS first  - best accuracy, needs open sky
 *   2. Cell tower fallback (RIL_GetLocation_Ex) if GPS has no fix
 *   3. Both fail  - count it, escalate if persistent
 *
 * Payload:
 *   {"clientId":"petfinder-01","lat":35.123456,"lng":51.123456,"src":"gps"}
 *=========================================================================*/
#define GPS_READ_ITEM  "RMC"

static void acquire_and_send_location(void)
{
    s32  lat_udeg = 0;
    s32  lng_udeg = 0;
    bool has_fix  = FALSE;
    const char *src = "gps";
    char lat_str[16];
    char lng_str[16];
    s32  ret;

    /*----------------------------------------------------------------------
     * Step 1 - GPS
     *
     * RMC layout:
     *   $GPRMC,time,status,lat,N/S,lon,E/W,...
     *      0    1     2     3   4    5   6
     * status 'A' = valid fix, 'V' = void. On 'V' the coordinate fields are
     * usually empty, which is exactly the case the old sscanf-based parser
     * could not represent.
     *--------------------------------------------------------------------*/
    Ql_memset(m_gps_buf, 0, sizeof(m_gps_buf));
    ret = RIL_GPS_Read((u8 *)GPS_READ_ITEM, m_gps_buf);

    if (ret == RIL_AT_SUCCESS && Ql_strlen((char *)m_gps_buf) > 0)
    {
        char *p = Ql_strstr((char *)m_gps_buf, "RMC");

        if (p != NULL)
        {
            char status[4];
            char lat_f[16], ns[4];
            char lon_f[16], ew[4];

            /* Rewind to the '$' so field 0 is the sentence header.
             * The modem may prefix the sentence with "+QGNSSRD: ". */
            while (p > (char *)m_gps_buf && *p != '$')
            {
                p--;
            }

            nmea_get_field(p, 2, status, sizeof(status));
            nmea_get_field(p, 3, lat_f,  sizeof(lat_f));
            nmea_get_field(p, 4, ns,     sizeof(ns));
            nmea_get_field(p, 5, lon_f,  sizeof(lon_f));
            nmea_get_field(p, 6, ew,     sizeof(ew));

            if (status[0] == 'A')
            {
                if (nmea_to_udeg(lat_f, &lat_udeg) &&
                    nmea_to_udeg(lon_f, &lng_udeg))
                {
                    if (ns[0] == 'S') lat_udeg = -lat_udeg;
                    if (ew[0] == 'W') lng_udeg = -lng_udeg;

                    has_fix = TRUE;
                    src     = "gps";

                    udeg_to_str(lat_udeg, lat_str);
                    udeg_to_str(lng_udeg, lng_str);
                    APP_DEBUG("[LOC] GPS fix: %s, %s\r\n", lat_str, lng_str);
                }
                else
                {
                    APP_DEBUG("[LOC] RMC status A but coords unparseable\r\n");
                }
            }
            else
            {
                APP_DEBUG("[LOC] GPS no fix yet (status=%s)\r\n",
                          status[0] ? status : "empty");
            }
        }
        else
        {
            APP_DEBUG("[LOC] no RMC sentence in GPS response\r\n");
        }
    }
    else
    {
        APP_DEBUG("[LOC] GPS read failed (ret=%d)\r\n", ret);
    }

    /*----------------------------------------------------------------------
     * Step 2 - cell tower fallback via QuecLocator.
     * Coarser (~300 m - 2 km) but works indoors. Blocking call.
     *--------------------------------------------------------------------*/
    if (!has_fix)
    {
        ST_LocInfo cell_loc;
        Ql_memset(&cell_loc, 0, sizeof(ST_LocInfo));

        APP_DEBUG("[LOC] trying cell tower positioning\r\n");
        ret = RIL_GetLocation_Ex(&cell_loc);

        if (ret == RIL_AT_SUCCESS &&
            (cell_loc.latitude != 0.0 || cell_loc.longitude != 0.0))
        {
            /* Float maths is fine on this part - only float *printing* is
             * unreliable, so we convert to integer micro-degrees here. */
            lat_udeg = (s32)(cell_loc.latitude  * 1000000.0);
            lng_udeg = (s32)(cell_loc.longitude * 1000000.0);

            has_fix = TRUE;
            src     = "cell";

            udeg_to_str(lat_udeg, lat_str);
            udeg_to_str(lng_udeg, lng_str);
            APP_DEBUG("[LOC] cell fix: %s, %s\r\n", lat_str, lng_str);
        }
        else
        {
            APP_DEBUG("[LOC] cell positioning failed (ret=%d)\r\n", ret);
        }
    }

    /*----------------------------------------------------------------------
     * Step 3 - publish, or escalate
     *--------------------------------------------------------------------*/
    if (!has_fix)
    {
        m_pub_fail_count++;
        APP_DEBUG("[LOC] no location, fail count=%d\r\n", m_pub_fail_count);

        if (m_pub_fail_count >= MAX_PUBLISH_FAILURES)
        {
            APP_DEBUG("[LOC] persistent location failure, reconnecting\r\n");
            m_pub_fail_count = 0;
            teardown_mqtt();
        }
        return;
    }

    udeg_to_str(lat_udeg, lat_str);
    udeg_to_str(lng_udeg, lng_str);

    m_msg_id++;
    if (m_msg_id > 65535) m_msg_id = 1;

    Ql_memset(m_payload, 0, sizeof(m_payload));
    Ql_sprintf(m_payload,
               "{\"clientId\":\"%s\",\"lat\":%s,\"lng\":%s,\"src\":\"%s\"}",
               (char *)m_client_id, lat_str, lng_str, src);

    APP_DEBUG("[PUB] %s -> %s\r\n", MQTT_PUB_TOPIC, m_payload);

    ret = RIL_MQTT_QMTPUB(m_conn_id,
                          m_msg_id,
                          QOS1_AT_LEASET_ONCE,
                          0,                       /* retain = false */
                          (u8 *)MQTT_PUB_TOPIC,
                          Ql_strlen(m_payload),
                          (u8 *)m_payload);

    /*
     * NOTE: RIL_AT_SUCCESS here means only that the modem ACCEPTED the
     * command. Whether the broker actually got it arrives later as
     * +QMTPUB, handled in the URC_MQTT_PUB case below - that is where the
     * failure counters are cleared. Do not reset them here.
     */
    if (ret != RIL_AT_SUCCESS)
    {
        m_pub_fail_count++;
        APP_DEBUG("[PUB] QMTPUB rejected (ret=%d), fails=%d\r\n",
                  ret, m_pub_fail_count);

        if (m_pub_fail_count >= MAX_PUBLISH_FAILURES)
        {
            APP_DEBUG("[PUB] too many failures, reconnecting MQTT\r\n");
            m_pub_fail_count = 0;
            teardown_mqtt();
        }
    }
}

/*===========================================================================
 * [13] Callbacks
 *=========================================================================*/

static void mqtt_recv(u8 *buffer, u32 length)
{
    APP_DEBUG("[MQTT] rx (%d bytes): %s\r\n", length, buffer);
}

/* UART handler - we only log, but the port must be registered for the
 * APP_DEBUG macro to have somewhere to write. */
static void CallBack_UART_Hdlr(Enum_SerialPort port, Enum_UARTEventType msg,
                               bool level, void *customizedPara)
{
    /* Nothing to do - Pet Finder takes no console commands. */
}

/*===========================================================================
 * [14] proc_main_task  -  OpenCPU entry point
 *=========================================================================*/
void proc_main_task(s32 taskId)
{
    ST_MSG msg;
    s32 ret;

    /*----------------------------------------------------------------------
     * Open UART1 first so every subsequent APP_DEBUG is actually visible.
     * Ql_Debug_Trace is used for these two lines only, because the UART
     * is not up yet.
     *--------------------------------------------------------------------*/
    ret = Ql_UART_Register(UART_PORT1, CallBack_UART_Hdlr, NULL);
    if (ret < QL_RET_OK)
    {
        Ql_Debug_Trace("Fail to register UART1, ret=%d\r\n", ret);
    }
    ret = Ql_UART_Open(UART_PORT1, 115200, FC_NONE);
    if (ret < QL_RET_OK)
    {
        Ql_Debug_Trace("Fail to open UART1, ret=%d\r\n", ret);
    }

    APP_DEBUG("\r\n<-- Pet Finder starting -->\r\n");

    /*----------------------------------------------------------------------
     * Register timers and CHECK the return codes. A rejected registration
     * is otherwise completely silent - the timer simply never fires and
     * the app stalls with no error anywhere.
     * QL_RET_OK is 0; anything else here means the ID is bad or in use.
     *--------------------------------------------------------------------*/
    ret = Ql_Timer_Register(TIMER_ID_STATE_MACHINE, Timer_Callback, NULL);
    APP_DEBUG("[BOOT] state timer register, ret=%d\r\n", ret);
    if (ret != QL_RET_OK)
    {
        APP_DEBUG("[BOOT] FATAL: state machine timer rejected\r\n");
    }

    ret = Ql_Timer_Register(TIMER_ID_PUBLISH, Timer_Callback, NULL);
    APP_DEBUG("[BOOT] publish timer register, ret=%d\r\n", ret);
    if (ret != QL_RET_OK)
    {
        APP_DEBUG("[BOOT] FATAL: publish timer rejected\r\n");
    }

    ret = Ql_Mqtt_Recv_Register(mqtt_recv);
    APP_DEBUG("[BOOT] MQTT recv callback registered, ret=%d\r\n", ret);

    while (TRUE)
    {
        Ql_OS_GetMessage(&msg);

        switch (msg.message)
        {
        /*------------------------------------------------------------------
         * RIL is up. Start the state machine timer HERE, not on the SIM
         * URC. If that URC is missed or arrives before this loop is
         * running, the old code would sit in STATE_WAIT_SIM forever with
         * no timer to poll it.
         *----------------------------------------------------------------*/
        case MSG_ID_RIL_READY:
            APP_DEBUG("[BOOT] RIL ready\r\n");
            Ql_RIL_Initialize();
            m_state = STATE_WAIT_SIM;
            {
                s32 tret = Ql_Timer_Start(TIMER_ID_STATE_MACHINE,
                                          STATE_MACHINE_INTERVAL_MS, TRUE);
                APP_DEBUG("[BOOT] state machine timer start, ret=%d\r\n", tret);
            }
            break;

        case MSG_ID_URC_INDICATION:
        {
            switch (msg.param1)
            {
            /*--------------------------------------------------------------
             * SIM ready - fast path only. The timer is already running, so
             * this just skips up to a second of polling latency.
             *------------------------------------------------------------*/
            case URC_SIM_CARD_STATE_IND:
                APP_DEBUG("[URC] SIM state: %d\r\n", msg.param2);
                if (SIM_STAT_READY == msg.param2 && m_state == STATE_WAIT_SIM)
                {
                    m_state = STATE_WAIT_GSM;
                }
                break;

            case URC_GSM_NW_STATE_IND:
                APP_DEBUG("[URC] GSM state: %d\r\n", msg.param2);
                break;

            case URC_GPRS_NW_STATE_IND:
                APP_DEBUG("[URC] GPRS state: %d\r\n", msg.param2);
                break;

            /*--------------------------------------------------------------
             * +QMTOPEN - socket open result
             *------------------------------------------------------------*/
            case URC_MQTT_OPEN:
            {
                MQTT_Urc_Param_t *urc = (MQTT_Urc_Param_t *)msg.param2;
                if (urc != NULL && urc->result == 0)
                {
                    APP_DEBUG("[URC] +QMTOPEN ok\r\n");
                    m_mqtt_fail_count = 0;
                    m_state = STATE_MQTT_CONNECTING;
                }
                else
                {
                    APP_DEBUG("[URC] +QMTOPEN failed (result=%d)\r\n",
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
                        m_close_wait_ticks = 0;
                        m_state = STATE_MQTT_CLOSING;
                    }
                }
                break;
            }

            /*--------------------------------------------------------------
             * +QMTCONN - broker accepted (or rejected) our credentials.
             * result != 0 here with a correct host usually means bad
             * username/password or the client ID is already in use.
             *------------------------------------------------------------*/
            case URC_MQTT_CONN:
            {
                MQTT_Urc_Param_t *urc = (MQTT_Urc_Param_t *)msg.param2;
                if (urc != NULL && urc->result == 0)
                {
                    APP_DEBUG("[URC] +QMTCONN ok, starting publish timer\r\n");
                    m_mqtt_fail_count = 0;
                    m_pub_fail_count  = 0;
                    m_state = STATE_PUBLISHING;
                    Ql_Timer_Start(TIMER_ID_PUBLISH,
                                   PUBLISH_INTERVAL_MS, TRUE);
                }
                else
                {
                    APP_DEBUG("[URC] +QMTCONN failed (result=%d)\r\n",
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
             * +QMTPUB - the REAL publish result.
             * This is what clears the publish failure counter. Previously
             * the counter was cleared on the QMTPUB return value, which
             * only meant the modem accepted the command, so a broker that
             * silently dropped messages was never detected.
             *------------------------------------------------------------*/
            case URC_MQTT_PUB:
            {
                MQTT_Urc_Param_t *urc = (MQTT_Urc_Param_t *)msg.param2;
                if (urc != NULL && urc->result == 0)
                {
                    APP_DEBUG("[URC] +QMTPUB ok\r\n");
                    m_pub_fail_count  = 0;
                    m_mqtt_fail_count = 0;
                }
                else
                {
                    m_pub_fail_count++;
                    APP_DEBUG("[URC] +QMTPUB failed (result=%d), fails=%d\r\n",
                              urc ? urc->result : -1, m_pub_fail_count);

                    if (m_pub_fail_count >= MAX_PUBLISH_FAILURES)
                    {
                        APP_DEBUG("[URC] too many publish failures\r\n");
                        m_pub_fail_count = 0;
                        teardown_mqtt();
                    }
                }
                break;
            }

            case URC_MQTT_DISC:
            {
                MQTT_Urc_Param_t *urc = (MQTT_Urc_Param_t *)msg.param2;
                APP_DEBUG("[URC] +QMTDISC result=%d\r\n",
                          urc ? urc->result : -1);
                break;
            }

            /*--------------------------------------------------------------
             * +QMTCLOSE - socket gone either way. Go back to CFG, since
             * configuration must be re-applied before each QMTOPEN.
             * Guarded so a stray close URC during boot cannot yank us out
             * of the network-registration states.
             *------------------------------------------------------------*/
            case URC_MQTT_CLOSE:
            {
                MQTT_Urc_Param_t *urc = (MQTT_Urc_Param_t *)msg.param2;
                APP_DEBUG("[URC] +QMTCLOSE result=%d\r\n",
                          urc ? urc->result : -1);

                if (m_state == STATE_MQTT_CLOSING ||
                    m_state == STATE_MQTT_DISCONNECTING ||
                    m_state == STATE_MQTT_OPEN_WAIT   ||
                    m_state == STATE_MQTT_CONN_WAIT   ||
                    m_state == STATE_PUBLISHING)
                {
                    Ql_Timer_Stop(TIMER_ID_PUBLISH);
                    enter_backoff(5, STATE_MQTT_CFG);
                }
                break;
            }

            default:
                break;
            }
            break; /* MSG_ID_URC_INDICATION */
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