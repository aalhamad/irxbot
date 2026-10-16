/*
 * CAN_VESC — ESP32 DevKit (WROOM) CAN bus validation against 4x VESC 75100
 *
 * Purpose: a known-good second CAN node. Confirms whether the SN65HVD230
 * module, harness and bus ground are healthy, independent of the STM32.
 *
 * Wiring (SN65HVD230 module — 3.3 V, no level shifting needed):
 *   ESP32 GPIO21 ── CTX
 *   ESP32 GPIO22 ── CRX
 *   ESP32 GND    ── GND   <- must ALSO reach the VESC CAN connector ground
 *   ESP32 3V3    ── 3V3
 *   module CANH  ── bus CANH
 *   module CANL  ── bus CANL
 *
 * Uses the built-in TWAI driver (ESP-IDF) — no library install required.
 *
 * Serial monitor @ 115200. Commands:
 *   r<rpm>  set RPM on all 4 VESCs, e.g. "r3000"
 *   s       stop (RPM 0)
 */

#include <driver/twai.h>

/* SELFTEST 1 = no-ACK self-test mode. Transmits without requiring any other
 * node to acknowledge, and asks the controller to receive its own frames back.
 * This exercises TX -> transceiver -> CANH/CANL -> transceiver -> RX using only
 * the module's own 120 ohm terminator, so it proves the module good or bad with
 * the robot bus physically unplugged. Set back to 0 to talk to the VESCs. */
#define SELFTEST 0

/* LISTEN_ONLY 1 = pure receiver. The controller never transmits and never even
 * sends ACK bits, so it cannot go bus-off and cannot disturb the bus. This
 * isolates the receive path completely: if the VESC is transmitting and the
 * wiring is right, rx MUST climb. Overrides SELFTEST. */
#define LISTEN_ONLY 0

/* Bring-up one VESC at a time. The controller currently on the bus reports as
 * ID 2 (observed: ExtId 0x00000902), so command that one — not ID 1. Widen the
 * range only after each previous controller is proven. */
#define VESC_FIRST_ID 2
#define VESC_LAST_ID  2

/* RAW_DUMP 1 prints every frame seen on the bus (rate limited), not just
 * decoded STATUS packets. Invaluable when you don't yet know what's out there. */
#define RAW_DUMP 1

#define CAN_TX_PIN GPIO_NUM_21
#define CAN_RX_PIN GPIO_NUM_22

#define CAN_PACKET_SET_RPM 3
#define CAN_PACKET_STATUS  9

/* Transmit from boot with RPM 0 — harmless (won't move a motor) but it keeps
 * the TX path exercised so TEC/bus_err are meaningful without any serial input. */
static int32_t  target_rpm   = 0;
static bool     driving      = true;
static uint32_t last_send    = 0;
static uint32_t last_status  = 0;
static uint32_t rx_count     = 0;
static uint32_t tx_ok        = 0;
static uint32_t tx_fail      = 0;

/* Track which VESC IDs we have heard from */
static bool seen[5] = {false};

void setup()
{
  Serial.begin(115200);
  delay(300);
  Serial.println();
  Serial.println("=== CAN_VESC boot ===");

#if LISTEN_ONLY
  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(
      CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_LISTEN_ONLY);
  driving = false;                 /* never transmit in this mode */
  Serial.println("MODE: LISTEN_ONLY (pure receiver, cannot go bus-off)");
  Serial.println("Expect rx to climb if the VESC is transmitting.");
#elif SELFTEST
  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(
      CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NO_ACK);
  Serial.println("MODE: SELFTEST (no-ACK, self-reception)");
  Serial.println("Unplug CANH/CANL from the robot bus for this test.");
#else
  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(
      CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
  Serial.println("MODE: NORMAL");
#endif
  g.rx_queue_len = 32;
  g.tx_queue_len = 16;

  twai_timing_config_t t = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g, &t, &f) != ESP_OK) {
    Serial.println("ERR: twai_driver_install failed");
    while (1) delay(1000);
  }
  if (twai_start() != ESP_OK) {
    Serial.println("ERR: twai_start failed");
    while (1) delay(1000);
  }

  Serial.println("TWAI started @ 500 kbps, accept-all filter");
  Serial.printf("Commanding VESC IDs %d..%d\n", VESC_FIRST_ID, VESC_LAST_ID);
  Serial.println("Listening. Type r3000 to drive, s to stop.");
}

/* ── TX: VESC SET_RPM, extended ID = (packet_id << 8) | vesc_id ───────────── */

static bool send_rpm(uint8_t vesc_id, int32_t rpm)
{
  twai_message_t msg;
  memset(&msg, 0, sizeof(msg));

  msg.extd              = 1;                 /* 29-bit extended ID */
  msg.identifier        = ((uint32_t)CAN_PACKET_SET_RPM << 8) | vesc_id;
  msg.data_length_code  = 4;
#if SELFTEST
  msg.self              = 1;                 /* loop the frame back to our RX */
#endif

  /* VESC expects big-endian int32 */
  msg.data[0] = (rpm >> 24) & 0xFF;
  msg.data[1] = (rpm >> 16) & 0xFF;
  msg.data[2] = (rpm >> 8)  & 0xFF;
  msg.data[3] =  rpm        & 0xFF;

  return twai_transmit(&msg, pdMS_TO_TICKS(20)) == ESP_OK;
}

/* ── Serial command parsing ───────────────────────────────────────────────── */

static char    sbuf[16];
static uint8_t slen      = 0;
static uint32_t last_char = 0;

static void apply_command()
{
  sbuf[slen] = '\0';
  if (sbuf[0] == 'r') {
    target_rpm = atol(sbuf + 1);
    driving    = true;
    Serial.printf(">> drive RPM=%ld\n", target_rpm);
  } else if (sbuf[0] == 's') {
    target_rpm = 0;
    driving    = true;            /* keep sending zeros */
    Serial.println(">> stop");
  } else {
    Serial.println(">> unknown (use r<rpm> or s)");
  }
  slen = 0;
}

static void poll_serial()
{
  while (Serial.available()) {
    char c = Serial.read();
    last_char = millis();
    if (c == '\r' || c == '\n') {
      if (slen) apply_command();
    } else if (slen < sizeof(sbuf) - 1) {
      sbuf[slen++] = c;
    }
  }

  /* Flush on idle so it works with Serial Monitor set to "No line ending" */
  if (slen && (millis() - last_char) > 60) apply_command();
}

/* ── RX: drain and report ─────────────────────────────────────────────────── */

static void poll_can_rx()
{
  twai_message_t msg;
  while (twai_receive(&msg, 0) == ESP_OK) {
    rx_count++;

    uint8_t vesc_id = msg.identifier & 0xFF;
    uint8_t pkt_id  = (msg.identifier >> 8) & 0xFF;

    if (vesc_id >= 1 && vesc_id <= 4 && !seen[vesc_id]) {
      seen[vesc_id] = true;
#if SELFTEST
      /* Our own loopback frame, not a real controller */
      Serial.printf("** loopback frame id=%u **\n", vesc_id);
#else
      Serial.printf("** first frame from VESC %u **\n", vesc_id);
#endif
    }

#if RAW_DUMP
    /* Print every distinct (pkt_id, vesc_id) pair once per second so the
     * console stays readable while still showing everything on the bus. */
    static uint32_t last_raw = 0;
    if (millis() - last_raw >= 1000) {
      last_raw = millis();
      Serial.printf("RAW id=0x%08lX ext=%u dlc=%u data=",
                    (uint32_t)msg.identifier, msg.extd, msg.data_length_code);
      for (int i = 0; i < msg.data_length_code; i++)
        Serial.printf("%02X ", msg.data[i]);
      Serial.printf(" (pkt=%u vesc=%u)\n", pkt_id, vesc_id);
    }
#endif

    /* Decode RPM out of the STATUS packet. Printed once per second per ID —
     * unrated it floods the console at the VESC's status rate. */
    if (pkt_id == CAN_PACKET_STATUS && msg.data_length_code >= 4) {
      int32_t rpm = (int32_t)(((uint32_t)msg.data[0] << 24) |
                              ((uint32_t)msg.data[1] << 16) |
                              ((uint32_t)msg.data[2] << 8)  |
                               (uint32_t)msg.data[3]);
      static uint32_t last_stat[5] = {0};
      if (vesc_id <= 4 && millis() - last_stat[vesc_id] >= 1000) {
        last_stat[vesc_id] = millis();
        Serial.printf("STATUS id=%u rpm=%ld\n", vesc_id, rpm);
      }
    }
  }
}

/* ── Bus health — the ESP32 equivalent of the STM32 ESR dump ──────────────── */

static void print_status()
{
  twai_status_info_t st;
  if (twai_get_status_info(&st) != ESP_OK) return;

  const char *state = "?";
  switch (st.state) {
    case TWAI_STATE_STOPPED:   state = "STOPPED";   break;
    case TWAI_STATE_RUNNING:   state = "RUNNING";   break;
    case TWAI_STATE_BUS_OFF:   state = "BUS_OFF";   break;
    case TWAI_STATE_RECOVERING:state = "RECOVERING";break;
  }

  /* Frames/sec matters: a healthy VESC emits STATUS at a steady tens-of-Hz
   * rate. Thousands per second means frames are being retransmitted because
   * nothing on the bus is acknowledging them. */
  static uint32_t prev_rx = 0;
  uint32_t rx_rate = rx_count - prev_rx;
  prev_rx = rx_count;

  Serial.printf(
    "[%s] rx=%lu (%lu/s) tx_ok=%lu tx_fail=%lu | TEC=%lu REC=%lu "
    "arb_lost=%lu bus_err=%lu rxq=%lu txq=%lu\n",
    state, rx_count, rx_rate, tx_ok, tx_fail,
    (uint32_t)st.tx_error_counter, (uint32_t)st.rx_error_counter,
    (uint32_t)st.arb_lost_count,   (uint32_t)st.bus_error_count,
    (uint32_t)st.msgs_to_rx,       (uint32_t)st.msgs_to_tx);

  /* Auto-recover from bus-off so the test keeps running */
  if (st.state == TWAI_STATE_BUS_OFF) {
    Serial.println("!! BUS_OFF — initiating recovery");
    twai_initiate_recovery();
  } else if (st.state == TWAI_STATE_STOPPED) {
    twai_start();
  }
}

void loop()
{
  uint32_t now = millis();

  poll_serial();
  poll_can_rx();

  /* Resend every 100 ms — VESC command timeout is 1 s */
  if (driving && (now - last_send) >= 100) {
    last_send = now;
    for (uint8_t id = VESC_FIRST_ID; id <= VESC_LAST_ID; id++) {
      if (send_rpm(id, target_rpm)) tx_ok++; else tx_fail++;
    }
  }

  if ((now - last_status) >= 1000) {
    last_status = now;
    print_status();
  }
}
