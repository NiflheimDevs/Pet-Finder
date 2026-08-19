/*===========================================================================
 * petfinder.c  -  Pet Finder firmware
 * Target : Quectel MC60  (OpenCPU, ARM7)
 *
 * Build  : set  C_PREDEF=-D __PETFINDER__  in gcc_makefile, then make clean/new
 *
 * Logs   : UART_PORT1 @ 115200 8N1
 *
 * SMS Configuration:
 *   Send an SMS to the device SIM number with comma-separated key=value
 *   pairs (no prefix needed). "user" and "pass" are required; "ip", "port",
 *   and "id" are optional and only override the current value if present.
 *     user=myuser,pass=mypass
 *     ip=45.67.139.65,port=1883,user=myuser,pass=mypass
 *   Settings are saved to UFS and survive reboot.
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
#include "ql_fs.h"

#include "ril.h"
#include "ril_util.h"
#include "ril_network.h"
#include "ril_sim.h"
#include "ril_gps.h"
#include "ril_location.h"
#include "ril_mqtt.h"
#include "ril_sms.h"
#include "ril_system.h"   /* needed for URC_SYS_INIT_STATE_IND / SYS_STATE_SMSOK */

#include "ql_gprs.h"

/*===========================================================================
 * [2] Configuration - compile-time defaults
 *
 * These values are used only if no config file exists on UFS.
 * Once an SMS CONFIG command is received and saved, the file values take
 * precedence on every subsequent boot.
 *=========================================================================*/

/* -- Cellular ----------------------------------------------------------- */
#define APN               "mtnirancell"
#define APN_USER          ""
#define APN_PASS          ""

/* -- MQTT broker defaults ----------------------------------------------- */
#define DEFAULT_MQTT_HOST  "45.67.139.65"
#define DEFAULT_MQTT_PORT  1883
#define DEFAULT_MQTT_USER  "petfinder-01"
#define DEFAULT_MQTT_PASS  "123qweasd"
#define DEFAULT_CLIENT_ID  "petfinder-01"

/* Topic is now built at runtime as "<user>/loc" - see rebuild_topic() */

/* -- Config file on UFS -------------------------------------------------
 * NOTE: UFS files use a plain relative filename - there is NO "UFS:" prefix
 * in this SDK. Only SD card paths need a prefix ("SD:filename.ext"). Using
 * "UFS:" here made Ql_FS_Open() try to open a file literally named
 * "UFS:pf_cfg.txt", which fails every time (fd < 0), so the saved config
 * was never actually persisted (or reloaded) across reboots.
 * ------------------------------------------------------------------------ */
#define CFG_FILE_PATH     "pf_cfg.txt"

/* -- Field size limits -------------------------------------------------- */
#define CFG_HOST_LEN   64
#define CFG_USER_LEN   64
#define CFG_PASS_LEN   64
#define CFG_ID_LEN     32

/* -- Timing ------------------------------------------------------------- */
#define PUBLISH_INTERVAL_MS       (5UL * 1000UL)
#define STATE_MACHINE_INTERVAL_MS (1UL * 1000UL)
#define RESET_AFTER_TICKS         (24UL * 60UL * 60UL)

/* -- Fault thresholds --------------------------------------------------- */
#define MAX_PUBLISH_FAILURES   5
#define MAX_MQTT_FAILURES      3
#define MAX_PDP_FAILURES       3

/*===========================================================================
 * [3] Debug output
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
 * [4] Timer IDs  (must be > 0xFF)
 *=========================================================================*/
#define TIMER_ID_STATE_MACHINE   (TIMER_ID_USER_START + 1)   /* 0x101 */
#define TIMER_ID_PUBLISH         (TIMER_ID_USER_START + 2)   /* 0x102 */

/*===========================================================================
 * [5] State machine
 *=========================================================================*/
typedef enum
{
    STATE_BOOT = 0,
    STATE_WAIT_SIM,
    STATE_WAIT_GSM,
    STATE_WAIT_GPRS,
    STATE_PDP_ACTIVATING,
    STATE_MQTT_CFG,
    STATE_MQTT_OPENING,
    STATE_MQTT_OPEN_WAIT,
    STATE_MQTT_CONNECTING,
    STATE_MQTT_CONN_WAIT,
    STATE_PUBLISHING,
    STATE_MQTT_DISCONNECTING,
    STATE_MQTT_CLOSING,
    STATE_BACKOFF
} AppState;

/*===========================================================================
 * [6] Runtime config struct
 *
 * Loaded from UFS at boot; updated and saved when a valid CONFIG SMS arrives.
 *=========================================================================*/
typedef struct
{
    u8 host[CFG_HOST_LEN];
    u16 port;
    u8 user[CFG_USER_LEN];
    u8 pass[CFG_PASS_LEN];
    u8 client_id[CFG_ID_LEN];
} BrokerConfig;

static BrokerConfig m_cfg;

/*===========================================================================
 * [7] Module-level variables
 *=========================================================================*/
static AppState       m_state           = STATE_BOOT;
static Enum_ConnectID m_conn_id         = ConnectID_0;
static u32            m_msg_id          = 0;

static u8   m_pub_fail_count  = 0;
static u8   m_mqtt_fail_count = 0;
static u8   m_pdp_fail_count  = 0;

static AppState m_backoff_next  = STATE_MQTT_OPENING;
static u16      m_backoff_ticks = 0;

static bool m_gps_opened    = FALSE;
static u32  m_uptime_ticks  = 0;
static bool m_busy          = FALSE;
static bool m_sms_ready     = FALSE;

static u8   m_gps_buf[512];
static char m_payload[300];   /* was 192 - now holds gps+cell+ts+tz */
static char m_topic[CFG_USER_LEN + 8];   /* "<user>/loc" */

static u8 m_open_wait_ticks  = 0;
static u8 m_conn_wait_ticks  = 0;
static u8 m_close_wait_ticks = 0;

#define MQTT_OPEN_TIMEOUT_TICKS  30
#define MQTT_CONN_TIMEOUT_TICKS  30
#define MQTT_CLOSE_TIMEOUT_TICKS 15

/*===========================================================================
 * [8] Forward declarations
 *=========================================================================*/
static void state_machine_tick(void);
static void acquire_and_send_location(void);
static void teardown_mqtt(void);
static void teardown_pdp(void);
static void enter_backoff(u16 ticks, AppState next_state);
static bool sms_init(void);
static void rebuild_topic(void);
static void update_gps_timestamp(const char *time_field, const char *date_field);
static void get_gps_timestamp(char *ts_out, u32 ts_size,
                               char *tz_out, u32 tz_size);

/*===========================================================================
 * [9] Config load / save  (UFS file)
 *
 * File format (plain text, one value per line):
 *   host=45.67.139.65
 *   port=1883
 *   user=petfinder-01
 *   pass=123qweasd
 *   id=petfinder-01
 *=========================================================================*/

/*---------------------------------------------------------------------------
 * cfg_parse_field()
 * Finds "key=value\n" in buf and copies value into out (max out_size-1 chars).
 * Returns TRUE on success.
 *-------------------------------------------------------------------------*/
static bool cfg_parse_field(const char *buf, const char *key,
                             char *out, u32 out_size)
{
    const char *p;
    const char *end;
    u32 n;

    p = Ql_strstr((char *)buf, (char *)key);
    if (p == NULL) return FALSE;

    p += Ql_strlen((char *)key);
    if (*p != '=') return FALSE;
    p++;

    end = p;
    while (*end != '\0' && *end != '\n' && *end != '\r') end++;

    n = (u32)(end - p);
    if (n == 0 || n >= out_size) return FALSE;

    Ql_memcpy(out, p, n);
    out[n] = '\0';
    return TRUE;
}

/*---------------------------------------------------------------------------
 * cfg_load()
 * Reads settings from UFS. Falls back to compile-time defaults on any error.
 *-------------------------------------------------------------------------*/
static void cfg_load(void)
{
    s32  fd;
    u8   buf[256];
    u32  rd = 0;
    char tmp[CFG_HOST_LEN];

    /* Apply defaults first so we always have valid values */
    Ql_memset(&m_cfg, 0, sizeof(m_cfg));
    Ql_strcpy((char *)m_cfg.host,      DEFAULT_MQTT_HOST);
    m_cfg.port = DEFAULT_MQTT_PORT;
    Ql_strcpy((char *)m_cfg.user,      DEFAULT_MQTT_USER);
    Ql_strcpy((char *)m_cfg.pass,      DEFAULT_MQTT_PASS);
    Ql_strcpy((char *)m_cfg.client_id, DEFAULT_CLIENT_ID);

    fd = Ql_FS_Open((u8 *)CFG_FILE_PATH, QL_FS_READ_ONLY);
    if (fd < 0)
    {
        APP_DEBUG("[CFG] no config file, using defaults\r\n");
        return;
    }

    Ql_memset(buf, 0, sizeof(buf));
    Ql_FS_Read(fd, buf, sizeof(buf) - 1, &rd);
    Ql_FS_Close(fd);

    if (rd == 0)
    {
        APP_DEBUG("[CFG] config file empty, using defaults\r\n");
        return;
    }

    /* Parse each field - if any is missing the default stays */
    if (cfg_parse_field((char *)buf, "host", tmp, sizeof(tmp)))
        Ql_strcpy((char *)m_cfg.host, tmp);

    if (cfg_parse_field((char *)buf, "port", tmp, sizeof(tmp)))
        m_cfg.port = (u16)Ql_atoi(tmp);

    if (cfg_parse_field((char *)buf, "user", tmp, sizeof(tmp)))
        Ql_strcpy((char *)m_cfg.user, tmp);

    if (cfg_parse_field((char *)buf, "pass", tmp, sizeof(tmp)))
        Ql_strcpy((char *)m_cfg.pass, tmp);

    if (cfg_parse_field((char *)buf, "id", tmp, sizeof(tmp)))
        Ql_strcpy((char *)m_cfg.client_id, tmp);

    APP_DEBUG("[CFG] loaded from file: host=%s port=%d user=%s\r\n",
              m_cfg.host, m_cfg.port, m_cfg.user);
}

/*---------------------------------------------------------------------------
 * cfg_save()
 * Writes current m_cfg to UFS, overwriting any previous file.
 *-------------------------------------------------------------------------*/
static bool cfg_save(void)
{
    s32  fd;
    char buf[256];
    u32  written = 0;
    u32  len;

    Ql_memset(buf, 0, sizeof(buf));
    Ql_sprintf(buf, "host=%s\nport=%d\nuser=%s\npass=%s\nid=%s\n",
               m_cfg.host, (s32)m_cfg.port,
               m_cfg.user, m_cfg.pass, m_cfg.client_id);
    len = Ql_strlen(buf);

    /* Delete old file first so we do a clean write */
    Ql_FS_Delete((u8 *)CFG_FILE_PATH);

    fd = Ql_FS_Open((u8 *)CFG_FILE_PATH, QL_FS_CREATE_ALWAYS | QL_FS_READ_WRITE);
    if (fd < 0)
    {
        APP_DEBUG("[CFG] failed to open file for write (fd=%d)\r\n", fd);
        return FALSE;
    }

    Ql_FS_Write(fd, (u8 *)buf, len, &written);
    Ql_FS_Close(fd);

    if (written != len)
    {
        APP_DEBUG("[CFG] write incomplete (%d/%d)\r\n", written, len);
        return FALSE;
    }

    APP_DEBUG("[CFG] config saved to UFS\r\n");
    return TRUE;
}

/*===========================================================================
 * [10] SMS CONFIG parser
 *
 * Expected format:
 *   CONFIG:ip=45.67.139.65,port=1883,user=ali,pass=1234
 *
 * All four fields are required. If any is missing or malformed the SMS is
 * silently ignored.
 *=========================================================================*/

/*---------------------------------------------------------------------------
 * sms_parse_value()
 * Finds "key=value" in a comma-delimited string and copies value into out.
 *-------------------------------------------------------------------------*/
static bool sms_parse_value(const char *msg, const char *key,
                             char *out, u32 out_size)
{
    const char *p;
    const char *end;
    u32 n;

    p = Ql_strstr((char *)msg, (char *)key);
    if (p == NULL) return FALSE;

    p += Ql_strlen((char *)key);
    if (*p != '=') return FALSE;
    p++;

    /* Value ends at comma, CR, LF, or end of string */
    end = p;
    while (*end != '\0' && *end != ',' && *end != '\r' && *end != '\n')
        end++;

    n = (u32)(end - p);
    if (n == 0 || n >= out_size) return FALSE;

    Ql_memcpy(out, p, n);
    out[n] = '\0';
    return TRUE;
}

/*---------------------------------------------------------------------------
 * sms_init()
 * Must be called only after the module reports SYS_STATE_SMSOK (i.e. from
 * the URC_SYS_INIT_STATE_IND handler). Before that point the SMS subsystem
 * hasn't finished loading the SIM's message index, so incoming messages can
 * be silently stored on the SIM with no URC_NEW_SMS_IND ever firing.
 *
 * This forces:
 *   - SMS storage explicitly to the SIM ("SM"), matching what a phone reads
 *   - AT+CNMI into live-notify mode, overriding whatever was left over in
 *     NVRAM from a previous session/firmware
 *-------------------------------------------------------------------------*/
static bool sms_init(void)
{
    s32 ret;
    u32 used = 0, total = 0;

    ret = RIL_SMS_SetStorage(RIL_SMS_STORAGE_TYPE_SM, &used, &total);
    if (ret != RIL_AT_SUCCESS)
    {
        APP_DEBUG("[SMS] SetStorage(SM) failed, ret=%d\r\n", ret);
        return FALSE;
    }
    APP_DEBUG("[SMS] storage=SM, used=%d, total=%d\r\n", used, total);

    /* Force live new-message notification (mode 2,1): report new SMS to
     * the host immediately via +CMTI -> URC_NEW_SMS_IND, instead of just
     * buffering it silently. */
    ret = Ql_RIL_SendATCmd("AT+CNMI=2,1,0,0,0",
                            Ql_strlen("AT+CNMI=2,1,0,0,0"), NULL, NULL, 0);
    if (ret != RIL_AT_SUCCESS)
    {
        APP_DEBUG("[SMS] AT+CNMI set failed, ret=%d\r\n", ret);
        return FALSE;
    }
    APP_DEBUG("[SMS] CNMI notify mode set\r\n");

    m_sms_ready = TRUE;
    return TRUE;
}

/*---------------------------------------------------------------------------
 * handle_config_sms()
 * Called when a new SMS arrives. Parses config fields and saves to UFS.
 *
 * Format: comma-separated key=value pairs, no prefix required.
 *   user= and pass= are REQUIRED.
 *   ip=, port=, id= are OPTIONAL - if omitted, the current value is kept.
 *     e.g.  user=sms,pass=123
 *     e.g.  ip=45.67.139.65,port=1883,user=sms,pass=123
 *
 * Returns TRUE if the SMS contained a valid user=/pass= config update.
 *-------------------------------------------------------------------------*/
static bool handle_config_sms(u32 sms_index)
{
    ST_RIL_SMS_TextInfo *info;
    ST_RIL_SMS_DeliverParam *deliver;
    const char *body;
    char new_host[CFG_HOST_LEN];
    char new_port_str[8];
    char new_user[CFG_USER_LEN];
    char new_pass[CFG_PASS_LEN];
    s32  ret;

    APP_DEBUG("[SMS] message received, index=%d\r\n", sms_index);

    info = (ST_RIL_SMS_TextInfo *)Ql_MEM_Alloc(sizeof(ST_RIL_SMS_TextInfo));
    if (info == NULL)
    {
        APP_DEBUG("[SMS] alloc failed\r\n");
        return FALSE;
    }

    Ql_memset(info, 0, sizeof(ST_RIL_SMS_TextInfo));
    ret = RIL_SMS_ReadSMS_Text(sms_index, LIB_SMS_CHARSET_GSM, info);
    if (ret != RIL_AT_SUCCESS)
    {
        APP_DEBUG("[SMS] read failed (ret=%d)\r\n", ret);
        Ql_MEM_Free(info);
        return FALSE;
    }

    deliver = &info->param.deliverParam;
    body    = (const char *)deliver->data;

    APP_DEBUG("[SMS] from=%s body=%s\r\n", deliver->oa, body);

    /* user and pass are required; ip/port/id are optional overrides */
    if (!sms_parse_value(body, "user", new_user, sizeof(new_user)) ||
        !sms_parse_value(body, "pass", new_pass, sizeof(new_pass)))
    {
        APP_DEBUG("[SMS] ignoring - need at least user=X,pass=X\r\n");
        Ql_MEM_Free(info);
        return FALSE;
    }

    if (sms_parse_value(body, "ip", new_host, sizeof(new_host)))
        Ql_strcpy((char *)m_cfg.host, new_host);

    if (sms_parse_value(body, "port", new_port_str, sizeof(new_port_str)))
        m_cfg.port = (u16)Ql_atoi(new_port_str);

    Ql_strcpy((char *)m_cfg.user, new_user);
    Ql_strcpy((char *)m_cfg.pass, new_pass);

    /* client_id follows the new user unless the SMS explicitly overrides
     * it with an id= field - e.g. user=ali -> client_id becomes "ali",
     * but user=ali,id=ali-collar-2 keeps the explicit id. */
    if (!sms_parse_value(body, "id", (char *)m_cfg.client_id,
                          sizeof(m_cfg.client_id)))
    {
        Ql_strcpy((char *)m_cfg.client_id, new_user);
    }

    /* publish topic is derived from user ("<user>/loc"), rebuild it now */
    rebuild_topic();

    APP_DEBUG("[SMS] new config: host=%s port=%d user=%s client_id=%s topic=%s\r\n",
              m_cfg.host, m_cfg.port, m_cfg.user, m_cfg.client_id, m_topic);

    Ql_MEM_Free(info);

    if (!cfg_save())
    {
        APP_DEBUG("[SMS] WARNING: config applied but NOT saved to file\r\n");
    }

    /* Tear down current MQTT connection so we reconnect with new settings */
    APP_DEBUG("[SMS] restarting MQTT with new settings\r\n");
    teardown_mqtt();

    /* Delete the SMS from storage so it is not re-processed after reboot */
    RIL_SMS_DeleteSMS(sms_index, RIL_SMS_DEL_INDEXED_MSG);

    return TRUE;
}

/*===========================================================================
 * [11] Helpers - recovery
 *=========================================================================*/
static void enter_backoff(u16 ticks, AppState next_state)
{
    APP_DEBUG("[BACKOFF] waiting %d ticks, then state %d\r\n",
              ticks, (s32)next_state);
    m_backoff_ticks = ticks;
    m_backoff_next  = next_state;
    m_state         = STATE_BACKOFF;
}

static void teardown_mqtt(void)
{
    APP_DEBUG("[MQTT] tearing down MQTT layer\r\n");
    Ql_Timer_Stop(TIMER_ID_PUBLISH);
    RIL_MQTT_QMTDISC(m_conn_id);
    m_state = STATE_MQTT_DISCONNECTING;
}

static void teardown_pdp(void)
{
    APP_DEBUG("[PDP] tearing down PDP context\r\n");
    Ql_Timer_Stop(TIMER_ID_PUBLISH);
    RIL_MQTT_QMTDISC(m_conn_id);
    RIL_MQTT_QMTCLOSE(m_conn_id);
    RIL_NW_ClosePDPContext();

    m_pdp_fail_count++;
    if (m_pdp_fail_count >= MAX_PDP_FAILURES)
    {
        APP_DEBUG("[PDP] too many PDP failures, resetting module\r\n");
        Ql_Reset(0);
    }
    enter_backoff(15, STATE_WAIT_GPRS);
}

/*---------------------------------------------------------------------------
 * rebuild_topic()
 * Rebuilds the publish topic as "<user>/loc" from the current m_cfg.user.
 * Call this after cfg_load() at boot and any time m_cfg.user changes
 * (i.e. after a valid CONFIG SMS is applied).
 *-------------------------------------------------------------------------*/
static void rebuild_topic(void)
{
    Ql_memset(m_topic, 0, sizeof(m_topic));
    Ql_sprintf(m_topic, "%s/loc", (char *)m_cfg.user);
    APP_DEBUG("[CFG] topic set to %s\r\n", m_topic);
}

/*---------------------------------------------------------------------------
 * GPS-derived timestamp
 *
 * We deliberately do NOT use network/NITZ time (Ql_GetLocalTime) - Irancell
 * (and Iranian operators generally) frequently don't broadcast NITZ, which
 * left the module's RTC stuck at its unset default (looked like 2004-01-01).
 *
 * Instead we take UTC date+time straight from the GPS RMC sentence (fields
 * 1 and 9 - the same sentence we already parse for lat/lon), which is
 * always UTC and doesn't depend on the operator at all. GPS itself carries
 * no timezone, so a fixed offset is added here in firmware.
 *
 * The most recent GPS-derived timestamp is kept in RAM only (m_last_ts) -
 * no UFS persistence, so it resets on reboot per your earlier call. If we
 * haven't had a single GPS fix yet since boot, we report a placeholder
 * ("0000-00-00 00:00:00") rather than inventing a plausible-looking time.
 *-------------------------------------------------------------------------*/
#define GPS_TZ_OFFSET_MIN   (3 * 60 + 30)   /* +03:30 - Iran. GPS is UTC-only,
                                              * so this is a firmware constant,
                                              * not read from anywhere. Change
                                              * this (and GPS_TZ_STRING below)
                                              * if the device ever operates in
                                              * a different timezone. */
#define GPS_TZ_STRING        "+03:30"

static bool m_time_valid = FALSE;
static char m_last_ts[20] = "0000-00-00 00:00:00";

static bool is_leap_year(s32 y)
{
    return ((y % 4 == 0) && (y % 100 != 0 || y % 400 == 0));
}

static u8 days_in_month(s32 y, s32 mon)
{
    static const u8 dim[12] = {31,28,31,30,31,30,31,31,30,31,30,31};
    if (mon == 2 && is_leap_year(y)) return 29;
    return dim[mon - 1];
}

/*---------------------------------------------------------------------------
 * update_gps_timestamp()
 * time_field: NMEA RMC field 1, "hhmmss.sss" (UTC)
 * date_field: NMEA RMC field 9, "ddmmyy"      (UTC)
 * Parses both, applies the fixed GPS_TZ_OFFSET_MIN, and stores the
 * resulting local date/time string into m_last_ts. Only call this once
 * status=='A' (valid fix) - garbage in these fields otherwise.
 *-------------------------------------------------------------------------*/
static void update_gps_timestamp(const char *time_field, const char *date_field)
{
    s32 hh, mm, ss, dd, mon, yyyy;
    s32 total_min, day_add;

    if (Ql_strlen((char *)time_field) < 6 || Ql_strlen((char *)date_field) < 6)
        return;

    hh = (time_field[0]-'0')*10 + (time_field[1]-'0');
    mm = (time_field[2]-'0')*10 + (time_field[3]-'0');
    ss = (time_field[4]-'0')*10 + (time_field[5]-'0');

    dd  = (date_field[0]-'0')*10 + (date_field[1]-'0');
    mon = (date_field[2]-'0')*10 + (date_field[3]-'0');
    yyyy = 2000 + (date_field[4]-'0')*10 + (date_field[5]-'0');

    if (hh > 23 || mm > 59 || ss > 59 || dd < 1 || dd > 31 || mon < 1 || mon > 12)
        return;   /* malformed field - leave m_last_ts untouched */

    total_min = hh * 60 + mm + GPS_TZ_OFFSET_MIN;
    day_add = 0;
    while (total_min >= 24 * 60) { total_min -= 24 * 60; day_add++; }
    while (total_min < 0)        { total_min += 24 * 60; day_add--; }
    hh = total_min / 60;
    mm = total_min % 60;

    while (day_add > 0)
    {
        dd++;
        if (dd > days_in_month(yyyy, mon))
        {
            dd = 1;
            mon++;
            if (mon > 12) { mon = 1; yyyy++; }
        }
        day_add--;
    }
    while (day_add < 0)
    {
        dd--;
        if (dd < 1)
        {
            mon--;
            if (mon < 1) { mon = 12; yyyy--; }
            dd = days_in_month(yyyy, mon);
        }
        day_add++;
    }

    Ql_sprintf(m_last_ts, "%04d-%02d-%02d %02d:%02d:%02d",
               (s32)yyyy, (s32)mon, (s32)dd, (s32)hh, (s32)mm, (s32)ss);
    m_time_valid = TRUE;
}

/*---------------------------------------------------------------------------
 * get_gps_timestamp()
 * Returns the most recent GPS-derived local timestamp (RAM only - see
 * update_gps_timestamp above). Placeholder date if we've never had a fix.
 * Timezone is always the fixed GPS_TZ_STRING since it's a firmware
 * constant, not something read from a live source.
 *-------------------------------------------------------------------------*/
static void get_gps_timestamp(char *ts_out, u32 ts_size,
                               char *tz_out, u32 tz_size)
{
    if (!m_time_valid)
        APP_DEBUG("[TIME] no GPS fix since boot yet, ts is placeholder\r\n");
    Ql_strcpy(ts_out, m_last_ts);
    Ql_strcpy(tz_out, GPS_TZ_STRING);
    (void)ts_size;
    (void)tz_size;
}

/*===========================================================================
 * [12] NMEA parsing
 *=========================================================================*/
static bool nmea_get_field(const char *s, u8 idx, char *out, u32 out_size)
{
    u8  field = 0;
    u32 n     = 0;

    if (s == NULL || out == NULL || out_size == 0) return FALSE;
    Ql_memset(out, 0, out_size);

    while (*s != '\0' && *s != '\r' && *s != '\n' && *s != '*')
    {
        if (*s == ',')
        {
            field++;
            if (field > idx) break;
            s++;
            continue;
        }
        if (field == idx && n < (out_size - 1))
            out[n++] = *s;
        s++;
    }
    out[n] = '\0';
    return (field >= idx) ? TRUE : FALSE;
}

static bool nmea_to_udeg(const char *s, s32 *out_udeg)
{
    s32 len, dot = -1, i, deg_digits;
    s32 deg = 0, min_int = 0, min_frac = 0, frac_div = 1;
    s32 minutes_scaled;

    if (s == NULL || out_udeg == NULL) return FALSE;
    len = (s32)Ql_strlen((char *)s);
    if (len < 4) return FALSE;

    for (i = 0; i < len; i++)
        if (s[i] == '.') { dot = i; break; }
    if (dot < 3) return FALSE;

    deg_digits = dot - 2;
    if (deg_digits < 1 || deg_digits > 3) return FALSE;

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
    for (i = dot + 1; i < len && (i - dot) <= 4; i++)
    {
        if (s[i] < '0' || s[i] > '9') break;
        min_frac  = min_frac * 10 + (s[i] - '0');
        frac_div *= 10;
    }
    while (frac_div < 10000) { min_frac *= 10; frac_div *= 10; }

    minutes_scaled = min_int * 10000 + min_frac;
    *out_udeg = deg * 1000000 + (minutes_scaled * 100) / 60;
    return TRUE;
}

static void udeg_to_str(s32 udeg, char *out)
{
    s32 whole, frac, i, n = 0, divisor = 100000;
    char tmp[8];

    if (udeg < 0) { out[n++] = '-'; udeg = -udeg; }
    whole = udeg / 1000000;
    frac  = udeg % 1000000;

    Ql_memset(tmp, 0, sizeof(tmp));
    Ql_sprintf(tmp, "%d", whole);
    for (i = 0; tmp[i] != '\0'; i++) out[n++] = tmp[i];
    out[n++] = '.';
    for (i = 0; i < 6; i++)
    {
        out[n++] = (char)('0' + ((frac / divisor) % 10));
        divisor /= 10;
    }
    out[n] = '\0';
}

/*===========================================================================
 * [13] Timer callback
 *=========================================================================*/
static void Timer_Callback(u32 timerId, void *param)
{
    if (m_busy) return;
    m_busy = TRUE;

    switch (timerId)
    {
    case TIMER_ID_STATE_MACHINE:
        state_machine_tick();
        break;
    case TIMER_ID_PUBLISH:
        if (m_state == STATE_PUBLISHING)
            acquire_and_send_location();
        break;
    default:
        break;
    }

    m_busy = FALSE;
}

/*===========================================================================
 * [14] State machine tick
 *=========================================================================*/
static void state_machine_tick(void)
{
    s32 ret;
    s32 gsm_state  = 0;
    s32 gprs_state = 0;

    m_uptime_ticks++;
    if (m_uptime_ticks >= RESET_AFTER_TICKS)
    {
        APP_DEBUG("[RESET] 24 h uptime reached, rebooting\r\n");
        Ql_Reset(0);
    }

    switch (m_state)
    {
    case STATE_BOOT:
        break;

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
            if (m_pdp_fail_count >= MAX_PDP_FAILURES) Ql_Reset(0);
            enter_backoff(15, STATE_WAIT_GPRS);
        }
        break;

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
        APP_DEBUG("[SM] opening MQTT socket to %s:%d\r\n",
                  m_cfg.host, (s32)m_cfg.port);
        ret = RIL_MQTT_QMTOPEN(m_conn_id, m_cfg.host, m_cfg.port);
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
            else enter_backoff(5, STATE_MQTT_OPENING);
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
        APP_DEBUG("[SM] sending MQTT CONNECT (id=%s)\r\n", m_cfg.client_id);
        ret = RIL_MQTT_QMTCONN(m_conn_id,
                                m_cfg.client_id,
                                m_cfg.user,
                                m_cfg.pass);
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
            else teardown_mqtt();
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
            else teardown_mqtt();
        }
        break;

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
        if (m_backoff_ticks > 0) m_backoff_ticks--;
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
 * [15] Location acquisition and MQTT publish
 *=========================================================================*/
#define GPS_READ_ITEM  "RMC"

static void acquire_and_send_location(void)
{
    /* GPS */
    s32  gps_lat_udeg = 0, gps_lng_udeg = 0;
    bool gps_has_fix  = FALSE;
    char gps_lat_str[16], gps_lng_str[16];

    /* Cell */
    s32  cell_lat_udeg = 0, cell_lng_udeg = 0;
    bool cell_has_fix  = FALSE;
    char cell_lat_str[16], cell_lng_str[16];

    char ts_str[24];
    char tz_str[8];

    s32  ret;

    /* ---- GPS: read regardless of whether we'll end up using it ---- */
    Ql_memset(m_gps_buf, 0, sizeof(m_gps_buf));
    ret = RIL_GPS_Read((u8 *)GPS_READ_ITEM, m_gps_buf);

    if (ret == RIL_AT_SUCCESS && Ql_strlen((char *)m_gps_buf) > 0)
    {
        char *p = Ql_strstr((char *)m_gps_buf, "RMC");
        if (p != NULL)
        {
            char status[4], time_f[16], lat_f[16], ns[4], lon_f[16], ew[4], date_f[8];
            while (p > (char *)m_gps_buf && *p != '$') p--;

            nmea_get_field(p, 1, time_f, sizeof(time_f));
            nmea_get_field(p, 2, status, sizeof(status));
            nmea_get_field(p, 3, lat_f,  sizeof(lat_f));
            nmea_get_field(p, 4, ns,     sizeof(ns));
            nmea_get_field(p, 5, lon_f,  sizeof(lon_f));
            nmea_get_field(p, 6, ew,     sizeof(ew));
            nmea_get_field(p, 9, date_f, sizeof(date_f));

            if (status[0] == 'A')
            {
                if (nmea_to_udeg(lat_f, &gps_lat_udeg) &&
                    nmea_to_udeg(lon_f, &gps_lng_udeg))
                {
                    if (ns[0] == 'S') gps_lat_udeg = -gps_lat_udeg;
                    if (ew[0] == 'W') gps_lng_udeg = -gps_lng_udeg;
                    gps_has_fix = TRUE;
                    update_gps_timestamp(time_f, date_f);
                    APP_DEBUG("[LOC] GPS fix\r\n");
                }
            }
            else
            {
                APP_DEBUG("[LOC] GPS no fix yet (status=%s)\r\n",
                          status[0] ? status : "empty");
            }
        }
    }

    /* ---- Cell: always read too, independent of GPS result ---- */
    {
        ST_LocInfo cell_loc;
        Ql_memset(&cell_loc, 0, sizeof(ST_LocInfo));
        ret = RIL_GetLocation_Ex(&cell_loc);
        if (ret == RIL_AT_SUCCESS &&
            (cell_loc.latitude != 0.0 || cell_loc.longitude != 0.0))
        {
            cell_lat_udeg = (s32)(cell_loc.latitude  * 1000000.0);
            cell_lng_udeg = (s32)(cell_loc.longitude * 1000000.0);
            cell_has_fix  = TRUE;
            APP_DEBUG("[LOC] cell fix\r\n");
        }
        else
        {
            APP_DEBUG("[LOC] cell positioning failed (ret=%d)\r\n", ret);
        }
    }

    /* Only skip the publish entirely if BOTH sources came back empty */
    if (!gps_has_fix && !cell_has_fix)
    {
        m_pub_fail_count++;
        APP_DEBUG("[LOC] no location from either source, fail count=%d\r\n",
                  m_pub_fail_count);
        if (m_pub_fail_count >= MAX_PUBLISH_FAILURES)
        {
            m_pub_fail_count = 0;
            teardown_mqtt();
        }
        return;
    }

    /* gps_lat_udeg/gps_lng_udeg (or cell_*) stay 0 if that source had no
     * fix - fix flag in the payload tells the consumer whether to trust
     * the coordinates for that source. */
    udeg_to_str(gps_lat_udeg,  gps_lat_str);
    udeg_to_str(gps_lng_udeg,  gps_lng_str);
    udeg_to_str(cell_lat_udeg, cell_lat_str);
    udeg_to_str(cell_lng_udeg, cell_lng_str);

    get_gps_timestamp(ts_str, sizeof(ts_str), tz_str, sizeof(tz_str));

    m_msg_id++;
    if (m_msg_id > 65535) m_msg_id = 1;

    Ql_memset(m_payload, 0, sizeof(m_payload));
    Ql_sprintf(m_payload,
               "{\"clientId\":\"%s\","
               "\"gps\":{\"lat\":%s,\"lng\":%s,\"fix\":%s},"
               "\"cell\":{\"lat\":%s,\"lng\":%s,\"fix\":%s},"
               "\"ts\":\"%s\",\"tz\":\"%s\"}",
               (char *)m_cfg.client_id,
               gps_lat_str,  gps_lng_str,  gps_has_fix  ? "true" : "false",
               cell_lat_str, cell_lng_str, cell_has_fix ? "true" : "false",
               ts_str, tz_str);

    APP_DEBUG("[PUB] %s -> %s\r\n", m_topic, m_payload);

    ret = RIL_MQTT_QMTPUB(m_conn_id, m_msg_id, QOS1_AT_LEASET_ONCE, 0,
                          (u8 *)m_topic,
                          Ql_strlen(m_payload),
                          (u8 *)m_payload);
    if (ret != RIL_AT_SUCCESS)
    {
        m_pub_fail_count++;
        APP_DEBUG("[PUB] QMTPUB rejected (ret=%d), fails=%d\r\n",
                  ret, m_pub_fail_count);
        if (m_pub_fail_count >= MAX_PUBLISH_FAILURES)
        {
            m_pub_fail_count = 0;
            teardown_mqtt();
        }
    }
}

/*===========================================================================
 * [16] Callbacks
 *=========================================================================*/
static void mqtt_recv(u8 *buffer, u32 length)
{
    APP_DEBUG("[MQTT] rx (%d bytes): %s\r\n", length, buffer);
}

static void CallBack_UART_Hdlr(Enum_SerialPort port, Enum_UARTEventType msg,
                               bool level, void *customizedPara)
{
    /* Nothing to do */
}

/*===========================================================================
 * [17] proc_main_task  -  OpenCPU entry point
 *=========================================================================*/
void proc_main_task(s32 taskId)
{
    ST_MSG msg;
    s32 ret;

    ret = Ql_UART_Register(UART_PORT1, CallBack_UART_Hdlr, NULL);
    if (ret < QL_RET_OK)
        Ql_Debug_Trace("Fail to register UART1, ret=%d\r\n", ret);

    ret = Ql_UART_Open(UART_PORT1, 115200, FC_NONE);
    if (ret < QL_RET_OK)
        Ql_Debug_Trace("Fail to open UART1, ret=%d\r\n", ret);

    APP_DEBUG("\r\n<-- Pet Finder starting -->\r\n");

    /* Load broker config from UFS (or fall back to defaults) */
    cfg_load();
    rebuild_topic();   /* topic = "<user>/loc", derived from loaded config */

    ret = Ql_Timer_Register(TIMER_ID_STATE_MACHINE, Timer_Callback, NULL);
    APP_DEBUG("[BOOT] state timer register, ret=%d\r\n", ret);

    ret = Ql_Timer_Register(TIMER_ID_PUBLISH, Timer_Callback, NULL);
    APP_DEBUG("[BOOT] publish timer register, ret=%d\r\n", ret);

    ret = Ql_Mqtt_Recv_Register(mqtt_recv);
    APP_DEBUG("[BOOT] MQTT recv callback registered, ret=%d\r\n", ret);

    while (TRUE)
    {
        Ql_OS_GetMessage(&msg);

        switch (msg.message)
        {
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
            case URC_SIM_CARD_STATE_IND:
                APP_DEBUG("[URC] SIM state: %d\r\n", msg.param2);
                if (SIM_STAT_READY == msg.param2 && m_state == STATE_WAIT_SIM)
                    m_state = STATE_WAIT_GSM;
                break;

            /*--------------------------------------------------------------
             * Module-level init state changes. We specifically need to
             * wait for SYS_STATE_SMSOK before touching SMS storage/CNMI -
             * doing it earlier is a no-op at best, and on some boots means
             * incoming SMS get silently stored with no notification at all.
             *------------------------------------------------------------*/
            case URC_SYS_INIT_STATE_IND:
                APP_DEBUG("[URC] sys init state: %d\r\n", msg.param2);
                if (SYS_STATE_SMSOK == msg.param2 && !m_sms_ready)
                {
                    if (!sms_init())
                        APP_DEBUG("[SMS] init failed, config-by-SMS will not work\r\n");
                }
                break;

            case URC_GSM_NW_STATE_IND:
                APP_DEBUG("[URC] GSM state: %d\r\n", msg.param2);
                break;

            case URC_GPRS_NW_STATE_IND:
                APP_DEBUG("[URC] GPRS state: %d\r\n", msg.param2);
                break;

            /*--------------------------------------------------------------
             * New SMS arrived - check if it is a CONFIG command
             *------------------------------------------------------------*/
            case URC_NEW_SMS_IND:
                APP_DEBUG("[URC] new SMS index=%d\r\n", msg.param2);
                if (!m_sms_ready)
                    APP_DEBUG("[SMS] WARNING: SMS URC before SMSOK/init - "
                              "processing anyway\r\n");
                handle_config_sms((u32)msg.param2);
                break;

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

            case URC_MQTT_CONN:
            {
                MQTT_Urc_Param_t *urc = (MQTT_Urc_Param_t *)msg.param2;
                if (urc != NULL && urc->result == 0)
                {
                    APP_DEBUG("[URC] +QMTCONN ok, starting publish timer\r\n");
                    m_mqtt_fail_count = 0;
                    m_pub_fail_count  = 0;
                    m_state = STATE_PUBLISHING;
                    Ql_Timer_Start(TIMER_ID_PUBLISH, PUBLISH_INTERVAL_MS, TRUE);
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
                    else teardown_mqtt();
                }
                break;
            }

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

            case URC_MQTT_CLOSE:
            {
                MQTT_Urc_Param_t *urc = (MQTT_Urc_Param_t *)msg.param2;
                APP_DEBUG("[URC] +QMTCLOSE result=%d\r\n",
                          urc ? urc->result : -1);
                if (m_state == STATE_MQTT_CLOSING       ||
                    m_state == STATE_MQTT_DISCONNECTING  ||
                    m_state == STATE_MQTT_OPEN_WAIT      ||
                    m_state == STATE_MQTT_CONN_WAIT      ||
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
            break;
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

