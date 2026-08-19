# Pet Finder

A GPS pet tracker built on the **Quectel MC60** cellular module using its **OpenCPU** capability — custom C firmware runs directly on the module's ARM7 core, no external microcontroller needed.

Firmware file: `petfinder.c`

---

## What It Does

Reads the pet's GPS coordinates and publishes them to an MQTT broker over GPRS every 5 seconds. If GPS has no fix (pet is indoors), it falls back to cell-tower location via QuecLocator.

**Publish-only.** No topics are subscribed. A receive callback is registered so downlink commands can be added later, but nothing subscribes today.

---

## Boot Flow

```
1. Module boots
       │
       ▼
2. RIL ready
   └─ Modem AT command layer is up
   └─ Ql_RIL_Initialize()
   └─ START THE STATE MACHINE TIMER HERE  ← see note below

       │
       ▼
3. SIM ready
   └─ SIM inserted, unlocked, carrier known
   └─ Detected by polling; the SIM URC is only a fast path

       │
       ▼
4. GSM registered
   └─ Base station found, signal acquired
   └─ Power on GPS engine (head start on satellite acquisition)

       │
       ▼
5. GPRS registered
   └─ Data network available
   └─ Set PDP context + APN, activate PDP (blocking call)

       │
       ▼
6. MQTT bring-up
   └─ QMTCFG  → version 3.1.1 + show recv len (re-applied before every open)
   └─ QMTOPEN → wait for +QMTOPEN URC
   └─ QMTCONN → wait for +QMTCONN URC

       │
       ▼
7. Publishing  ← normal running state
   └─ Every 5 s: GPS read → parse RMC → publish; cell fallback if no fix
   └─ Any disconnect or failure → auto-reconnect
```

> **Why the timer starts at step 2, not step 3:** an earlier version started it inside the SIM URC handler. If that URC fired before the message loop was running, or was missed, the app sat in `WAIT_SIM` forever with nothing polling it. The timer now starts as soon as RIL is up, so the SIM polling path is genuinely reachable.

---

## Async vs Sync — the core rule

`QMTOPEN`, `QMTCONN`, and `QMTPUB` return `RIL_AT_SUCCESS` to mean **"the modem accepted the AT command"** — not that it worked. The real result arrives later as a URC (`+QMTOPEN`, `+QMTCONN`, `+QMTPUB`).

Every async command therefore has a matching `_WAIT` state with a timeout watchdog, and **publish success is judged by `+QMTPUB` only.** Failure counters are cleared in the URC handler, never on the send return value.

---

## Auto-Reconnect

Three-tier escalation ladder — no dead states:

| Trigger | Action |
|---|---|
| 5 consecutive publish failures | tear down MQTT, reconnect |
| 3 consecutive MQTT open/conn failures | tear down PDP |
| 3 consecutive PDP failures | module reset |
| GPRS lost while publishing | tear down PDP, wait for re-registration |
| 24 h uptime | module reset |

Teardown order is always **QMTDISC → QMTCLOSE → ClosePDPContext**. Skipping `QMTCLOSE` leaves `ConnectID_0` allocated and the next `QMTOPEN` gets rejected.

The 24 h reboot is a **tick counter inside the state machine** (86400 × 1 s), not a timer. An 86,400,000 ms interval is outside what some OpenCPU builds accept.

---

## Building

In `SDK/make/gcc/gcc_makefile`, set:

```
C_PREDEF=-D __PETFINDER__
```

Then:

```
make clean
make new
```

Flash the output binary to the MC60.

---

## Configuration

All settings are at the top of `petfinder.c`:

| Define | Current value |
|---|---|
| `APN` | `"mtnirancell"` (no user/pass) |
| `MQTT_HOST` | `"45.67.139.65"` |
| `MQTT_PORT` | `1883` |
| `MQTT_PUB_TOPIC` | `"petfinder/loc"` |
| `PUBLISH_INTERVAL_MS` | `5000` |
| `STATE_MACHINE_INTERVAL_MS` | `1000` |

Credentials are **`u8` arrays, not `#define`s set to `NULL`**:

```c
static u8 m_client_id[] = "petfinder-01";
static u8 m_username[]  = "petfinder-01";
static u8 m_password[]  = "123qweasd";
```

The RIL layer calls `Ql_strlen()` on these pointers when building the AT command — a real `NULL` is a null-deref, not "anonymous". For anonymous, use `""`.

---

## Timer IDs — must be > 0xFF

`ql_timer.h` defines `TIMER_ID_USER_START` as `0x100` and requires all IDs above `0xFF`:

```c
#define TIMER_ID_STATE_MACHINE   (TIMER_ID_USER_START + 1)   /* 0x101 */
#define TIMER_ID_PUBLISH         (TIMER_ID_USER_START + 2)   /* 0x102 */
```

Decimal `101`/`102` are `0x65`/`0x66` — below the floor. `Ql_Timer_Register` rejects them and **nothing reports the error at runtime**; the app just stalls after "RIL ready". Both registrations now log their return codes for exactly this reason.

Max 10 stack timers per task. We use 2.

---

## Payload Format

```json
{"clientId":"petfinder-01","lat":35.123456,"lng":51.123456,"src":"gps"}
```

`src` is `"gps"` or `"cell"`.

**No floats are ever printed.** Coordinates are carried as signed micro-degrees in `s32` and formatted digit-by-digit. `%f` and zero-padded `%06d` are not reliably implemented across OpenCPU `sprintf` builds, and a silently mangled payload is very hard to diagnose in the field. Float *arithmetic* is fine — only formatting is avoided.

---

## NMEA Parsing

RMC field layout:

```
$GPRMC,time,status,lat,N/S,lon,E/W,...
   0     1     2     3   4   5   6
```

Parsed with a hand-written comma tokenizer, **not `Ql_sscanf`**. `Ql_sscanf` exists in the SDK, but `%[^,]` cannot match a zero-length field — and a no-fix sentence is exactly that:

```
$GPRMC,,V,,,,,,,,,,N*53
```

The parser also rewinds to `$` before field 0, since the modem may prefix the sentence with `+QGNSSRD: `.

Conversion is integer-only: `ddmm.mmmm` → micro-degrees, degree-digit count inferred from the decimal point position so it handles both lat (2 digits) and lon (3 digits).

---

## Debug Output

**UART_PORT1 @ 115200 8N1**, via the `APP_DEBUG` macro.

`Ql_Debug_Trace` alone writes to the *debug* UART — a different physical port. UART1 is registered and opened at the top of `proc_main_task` so bring-up is visible on the port you're already wired to.

Healthy boot:

```
<-- Pet Finder starting -->
[BOOT] state timer register, ret=0
[BOOT] publish timer register, ret=0
[BOOT] MQTT recv callback registered, ret=0
[BOOT] RIL ready
[BOOT] state machine timer start, ret=0
[URC] SIM state: 1
[SM] GSM registered
[SM] GPRS registered
[SM] activating PDP context (APN=mtnirancell)
[SM] PDP context active
[SM] MQTT cfg done (v3.1.1)
[SM] opening MQTT socket to 45.67.139.65:1883
[URC] +QMTOPEN ok
[SM] sending MQTT CONNECT (id=petfinder-01)
[URC] +QMTCONN ok, starting publish timer
```

Any non-zero `ret` in the first four lines → stop there, don't debug the network layer.

---

## Broker Setup (EMQX)

Dashboard admin password (port 18083) and MQTT client credentials are **separate things**. Changing the dashboard login does nothing for the device.

In EMQX 5.x there is no `allow_anonymous` — all clients connect freely until you enable an authenticator. Ours lives under **Access Control → Authentication → Built-in Database**, with `user_id_type` set to **username** (not clientid).

Verify from a PC before touching the module:

```bash
mosquitto_sub -h 45.67.139.65 -p 1883 -t 'petfinder/loc' -u petfinder-01 -P 123qweasd -v
```

Also confirm the 1883 listener is bound to `0.0.0.0` (not `127.0.0.1`) and TCP 1883 is open in the host firewall and any cloud security group.

**Client ID must be unique.** Two devices sharing `petfinder-01`, or a test client left connected, will cause the broker to kick the older session — presenting as a confusing reconnect loop.

---

## Known Limits / TODO

- **No TLS.** Port 1883 is plaintext; credentials cross the public internet in the clear. Move to 8883 + TLS before this leaves the bench.
- **Blocking calls in a timer callback.** `RIL_NW_OpenPDPContext()` and `RIL_GetLocation_Ex()` are synchronous and can block for tens of seconds, during which URCs aren't processed. An `m_busy` re-entrancy guard stops the next tick re-entering the state machine. Quectel's own examples do the same, but unexplained ~30 s stalls trace back to here.
- **QoS 1 without PUBACK tracking.** `+QMTPUB` is handled, but there's no per-message retry queue — a failed publish is counted, not resent.
- **First fix is slow.** Expect `[LOC] GPS no fix yet` for several minutes on a cold start outdoors. Cell fallback covers the gap.
- **GNSS mode unconfirmed.** *(pending supervisor)* The MC60 supports two wiring modes — check which the board uses:
  - **All-in-one** — GSM core controls GNSS internally. Uses `RIL_GPS_Open` / `RIL_GPS_Read`. *(current default)*
  - **Stand-alone** — GNSS has its own UART and power. Requires `Ql_GNSS_*` APIs.