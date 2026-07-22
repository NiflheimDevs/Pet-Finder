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

/*===========================================================================
 * EOF – forward declarations and logic will be added in the next parts
 *=========================================================================*/

#endif /* __PETFINDER__ */