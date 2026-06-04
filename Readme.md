# Pet Finder

A GPS pet tracker built on the **Quectel MC60** cellular module using its **OpenCPU** capability — custom C firmware runs directly on the module's ARM7 core, no external microcontroller needed.

---

## What It Does

Reads the pet's GPS coordinates and publishes them to an MQTT broker over a GPRS (cellular data) connection every few seconds. If GPS has no fix (pet is indoors), it falls back to cell-tower location via QuecLocator.

---

## Boot Flow

```
1. Module boots
       │
       ▼
2. RIL ready
   └─ The modem's AT command layer is up
   └─ Call Ql_RIL_Initialize() to enable all RIL functions

       │
       ▼
3. SIM ready
   └─ SIM card is inserted, unlocked, and carrier is known
   └─ Start the state machine timer

       │
       ▼
4. GSM registered
   └─ Module found a base station and has signal
   └─ Start reading GPS in the background

       │
       ▼
5. GPRS registered
   └─ Cellular data network is available
   └─ Activate PDP context (the data "dial-up" session)

       │
       ▼
6. MQTT connect
   └─ Open TCP socket to broker
   └─ Send MQTT CONNECT with client credentials
   └─ Subscribe to command topic

       │
       ▼
7. Publishing  ← normal running state
   └─ Every N seconds: read GPS → publish location to broker
   └─ Any disconnect or failure → auto-reconnect
```

---

## Auto-Reconnect

The firmware never enters a dead state. Every failure point has a recovery path:

- GPRS drops → wait for re-registration → re-activate PDP → re-connect MQTT
- MQTT link error → disconnect → close socket → re-open
- Too many publish failures → reconnect from scratch
- 24-hour uptime → automatic module reset for long-term stability

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

All settings are at the top of `main2.c`:

| Define | Description |
|---|---|
| `APN` | Carrier APN for your SIM |
| `MQTT_HOST` | Broker address |
| `MQTT_PORT` | Broker port (default 1883) |
| `MQTT_CLIENT_ID` | Unique device name |
| `MQTT_PUB_TOPIC` | Topic to publish location to |
| `PUBLISH_INTERVAL_MS` | How often to publish (ms) |

---

## GNSS Mode Note(idk my own mode btw imma ask ma supervisor)

The MC60 supports two GNSS wiring modes. Check with your hardware provider which one the board uses, then update the GPS section in `main2.c` accordingly:

- **All-in-one** — GSM core controls GNSS internally. Uses `RIL_GPS_Open` / `RIL_GPS_Read`. *(current default)*
- **Stand-alone** — GNSS has its own UART and power. Requires `Ql_GNSS_*` APIs.