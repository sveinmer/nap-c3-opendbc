#pragma once

// ============================================
// SAFETY_TESLA_PREAP — Pre-Autopilot Tesla Model S (2012-2014)
// ============================================
//
// Standalone safety mode for Pre-AP Tesla Model S. These cars have NO
// Autopilot ECU, NO harness relay, and a different EPAS/CAN layout than
// HW1+ Teslas. This is the spiritual successor to Tinkla (Boggyver's
// Pre-AP openpilot fork, tesla_unity_betaC3 branch).
//
// WHY check_relay=false AND disable_static_blocking=true:
//
//   Pre-AP has no harness relay hardware. Standard openpilot uses a relay
//   on the harness to switch between stock AP ECU and openpilot — when
//   openpilot is not active, the relay routes CAN to the stock ECU. On
//   Pre-AP there is no AP ECU and no relay; the panda connects directly
//   to the car's CAN bus. Setting check_relay=true would cause the panda
//   to falsely detect a "relay malfunction" and block ALL TX permanently.
//   disable_static_blocking=true is required for the same reason — without
//   a relay, the panda's static blocking logic (which assumes relay state)
//   would incorrectly block messages.
//
//   Tinkla handled this identically via generic_rx_checks(false) in the
//   older panda API, with the comment "PreAP has no relay" (safety_tesla.h
//   line 1071, tesla_unity_betaC3 branch). The modern API added check_relay
//   and disable_static_blocking with restrictive defaults, so we explicitly
//   set them to get the same behavior Tinkla had implicitly.
//
// WHY ignore_checksum=true AND ignore_counter=true on RX:
//
//   Pre-AP EPAS firmware uses a byte-sum checksum, but the exact algorithm
//   has not been fully verified against all firmware versions. A checksum
//   mismatch caused a silent 21-second steering dropout during testing.
//   Tinkla's RX checks also had no checksum/counter validation (frequency
//   set to 0 for all messages). Once the checksum algorithm is verified
//   across all Pre-AP EPAS firmware versions, these can be re-enabled.
//
// ALL ACTUAL SAFETY CHECKS REMAIN FULLY ACTIVE:
//   - Steering angle + rate limits via steer_angle_cmd_checks_vm()
//   - controls_allowed gating on all TX
//   - Disengage on hands-on override (level >= 3)
//   - Disengage on EPAS error codes 6-9
//   - Disengage on door open, gear out of Drive
//   - Disengage on stalk cancel (with 600ms echo filter)
//   - AEB events blocked from openpilot
//   - EPB_epasControl mode validation
//   - Pedal TX gated by PREAP_FLAG_ENABLE_PEDAL + get_longitudinal_allowed()
//
// Completely independent from tesla_legacy.h — has its own hooks struct,
// counter/checksum functions, init, RX/TX/fwd hooks, and GTW emulation.
// Registered as SAFETY_TESLA_PREAP in declarations.h.

#include "opendbc/safety/declarations.h"

// Forward declarations for panda firmware CAN send (defined in can_common.h)
#if defined(STM32H7) || defined(STM32F4)
void can_send(CANPacket_t *to_push, uint8_t bus_number, bool skip_tx_hook);
void can_set_checksum(CANPacket_t *packet);
#endif

// ============================================
// Byte manipulation macros
// ============================================

#define PREAP_GET_BYTES_04(msg) ((msg)->data[0] | ((msg)->data[1] << 8) | ((msg)->data[2] << 16) | ((msg)->data[3] << 24))
#define PREAP_GET_BYTES_48(msg) ((msg)->data[4] | ((msg)->data[5] << 8) | ((msg)->data[6] << 16) | ((msg)->data[7] << 24))
#define PREAP_WORD_TO_BYTES(dst8, src32) 0[dst8] = ((src32) & 0xFFU); 1[dst8] = (((src32) >> 8U) & 0xFFU); 2[dst8] = (((src32) >> 16U) & 0xFFU); 3[dst8] = (((src32) >> 24U) & 0xFFU)

// ============================================
// Safety param flags
// ============================================
// Longitudinal is gated by PREAP_FLAG_ENABLE_PEDAL + get_longitudinal_allowed().
// There is no separate LONG_CONTROL flag — the framework's get_longitudinal_allowed()
// is a derived check (controls_allowed && !gas_pressed_prev), not a settable flag.
// This matches how tesla.h, honda.h, and hyundai.h handle longitudinal gating.

#define PREAP_FLAG_ENABLE_PEDAL         1U
#define PREAP_FLAG_RADAR_EMULATION      2U
#define PREAP_FLAG_RADAR_BEHIND_NOSECONE 4U
#define PREAP_FLAG_IC_INTEGRATION       8U   // Tinkla Buddy IC TX-gate (display-only, risk-tier 3)

// ============================================
// State variables
// ============================================

static bool preap_enable_pedal = false;
static bool preap_radar_emulation = false;
static bool preap_radar_behind_nosecone = false;
static bool preap_has_ic_integration = false;  // gates Buddy IC TX-frames in tx_hook

static int preap_pedal_can = -1;

// Gear and door checks
static int preap_gear = 4;        // init to Drive to avoid false disables on startup
static int preap_gear_prev = 4;
static bool preap_doors_open = false;

// Stalk echo filter
static uint32_t preap_last_stalk_engage_us = 0;
#define PREAP_CANCEL_ECHO_WINDOW_US 600000U  // 600ms

// Radar emulation state
static int preap_radar_status = 0;
static uint32_t preap_last_radar_signal = 0;
static int preap_radar_epas_type = 0;
static int preap_radar_position = 0;

// ============================================
// Checksum and counter (for EPAS validation)
// ============================================

static uint8_t tesla_preap_get_counter(const CANPacket_t *msg) {
  if (msg->addr == 0x370U) {
    return msg->data[6] & 0x0FU;  // EPAS_sysStatusCounter
  }
  return 0U;
}

static uint32_t tesla_preap_get_checksum(const CANPacket_t *msg) {
  if (msg->addr == 0x370U) {
    return msg->data[7];  // EPAS_sysStatusChecksum at byte 7
  }
  if (msg->addr == 0x488U) {
    return msg->data[3];  // DAS_steeringControlChecksum at byte 3
  }
  return 0U;
}

static uint32_t tesla_preap_compute_checksum(const CANPacket_t *msg) {
  // Tesla byte-sum checksum: sum of address bytes + all data bytes except checksum byte
  int checksum_byte = -1;
  if (msg->addr == 0x370U) {
    checksum_byte = 7;
  } else if (msg->addr == 0x488U) {
    checksum_byte = 3;
  }
  if (checksum_byte == -1) {
    return 0U;
  }

  uint8_t chksum = (uint8_t)(msg->addr & 0xFFU) + (uint8_t)((msg->addr >> 8) & 0xFFU);
  int len = GET_LEN(msg);
  for (int i = 0; i < len; i++) {
    if (i != checksum_byte) {
      chksum += msg->data[i];
    }
  }
  return chksum;
}

// CRC-8 lookup table (polynomial 0x1D) for steering angle re-addressing
static const int preap_crc_lookup[256] = {
  0x00, 0x1D, 0x3A, 0x27, 0x74, 0x69, 0x4E, 0x53, 0xE8, 0xF5, 0xD2, 0xCF, 0x9C, 0x81, 0xA6, 0xBB,
  0xCD, 0xD0, 0xF7, 0xEA, 0xB9, 0xA4, 0x83, 0x9E, 0x25, 0x38, 0x1F, 0x02, 0x51, 0x4C, 0x6B, 0x76,
  0x87, 0x9A, 0xBD, 0xA0, 0xF3, 0xEE, 0xC9, 0xD4, 0x6F, 0x72, 0x55, 0x48, 0x1B, 0x06, 0x21, 0x3C,
  0x4A, 0x57, 0x70, 0x6D, 0x3E, 0x23, 0x04, 0x19, 0xA2, 0xBF, 0x98, 0x85, 0xD6, 0xCB, 0xEC, 0xF1,
  0x13, 0x0E, 0x29, 0x34, 0x67, 0x7A, 0x5D, 0x40, 0xFB, 0xE6, 0xC1, 0xDC, 0x8F, 0x92, 0xB5, 0xA8,
  0xDE, 0xC3, 0xE4, 0xF9, 0xAA, 0xB7, 0x90, 0x8D, 0x36, 0x2B, 0x0C, 0x11, 0x42, 0x5F, 0x78, 0x65,
  0x94, 0x89, 0xAE, 0xB3, 0xE0, 0xFD, 0xDA, 0xC7, 0x7C, 0x61, 0x46, 0x5B, 0x08, 0x15, 0x32, 0x2F,
  0x59, 0x44, 0x63, 0x7E, 0x2D, 0x30, 0x17, 0x0A, 0xB1, 0xAC, 0x8B, 0x96, 0xC5, 0xD8, 0xFF, 0xE2,
  0x26, 0x3B, 0x1C, 0x01, 0x52, 0x4F, 0x68, 0x75, 0xCE, 0xD3, 0xF4, 0xE9, 0xBA, 0xA7, 0x80, 0x9D,
  0xEB, 0xF6, 0xD1, 0xCC, 0x9F, 0x82, 0xA5, 0xB8, 0x03, 0x1E, 0x39, 0x24, 0x77, 0x6A, 0x4D, 0x50,
  0xA1, 0xBC, 0x9B, 0x86, 0xD5, 0xC8, 0xEF, 0xF2, 0x49, 0x54, 0x73, 0x6E, 0x3D, 0x20, 0x07, 0x1A,
  0x6C, 0x71, 0x56, 0x4B, 0x18, 0x05, 0x22, 0x3F, 0x84, 0x99, 0xBE, 0xA3, 0xF0, 0xED, 0xCA, 0xD7,
  0x35, 0x28, 0x0F, 0x12, 0x41, 0x5C, 0x7B, 0x66, 0xDD, 0xC0, 0xE7, 0xFA, 0xA9, 0xB4, 0x93, 0x8E,
  0xF8, 0xE5, 0xC2, 0xDF, 0x8C, 0x91, 0xB6, 0xAB, 0x10, 0x0D, 0x2A, 0x37, 0x64, 0x79, 0x5E, 0x43,
  0xB2, 0xAF, 0x88, 0x95, 0xC6, 0xDB, 0xFC, 0xE1, 0x5A, 0x47, 0x60, 0x7D, 0x2E, 0x33, 0x14, 0x09,
  0x7F, 0x62, 0x45, 0x58, 0x0B, 0x16, 0x31, 0x2C, 0x97, 0x8A, 0xAD, 0xB0, 0xE3, 0xFE, 0xD9, 0xC4
};

static int preap_compute_crc8(uint32_t lo, uint32_t hi, int msg_len) {
  int crc = 0xFF;
  for (int x = 0; x < msg_len; x++) {
    int v = (x <= 3) ? ((lo >> (x * 8)) & 0xFF) : ((hi >> ((x - 4) * 8)) & 0xFF);
    crc = preap_crc_lookup[crc ^ v];
  }
  return crc ^ 0xFF;
}

// ============================================
// GTW Emulation helpers
// ============================================

static void preap_radar_readdr(const CANPacket_t *src, uint16_t new_addr) {
#if defined(STM32H7) || defined(STM32F4)
  CANPacket_t pkt;
  pkt.returned = 0U;
  pkt.rejected = 0U;
  pkt.extended = src->extended;
  pkt.bus = 1;
  pkt.addr = new_addr;
  pkt.data_len_code = src->data_len_code;
  for (int i = 0; i < GET_LEN(src); i++) {
    pkt.data[i] = src->data[i];
  }
  can_set_checksum(&pkt);
  can_send(&pkt, 1, true);
#else
  (void)src;
  (void)new_addr;
#endif
}

static uint8_t preap_byte_sum_checksum(const CANPacket_t *pkt) {
  uint8_t chksum = (uint8_t)(pkt->addr & 0xFFU) + (uint8_t)((pkt->addr >> 8) & 0xFFU);
  int len = GET_LEN(pkt);
  for (int i = 0; i < (len - 1); i++) {
    chksum += pkt->data[i];
  }
  return chksum;
}

// ============================================
// GTW Emulation: CAN0 → CAN1 for Bosch radar
// ============================================

// Forward declaration — IC-generator block is defined further down in the file
// (after the TX-hook section). The 0x348 trigger lives in gtw_emulation
// because 0x348 is not in preap_rx_checks (rx_hook only sees whitelisted msgs).
static void preap_ic_send_messages(void);

static void tesla_preap_gtw_emulation(const CANPacket_t *to_fwd) {
  int bus_num = GET_BUS(to_fwd);
  int addr = GET_ADDR(to_fwd);

  // GTW_status (0x348) broadcast by Pre-AP gateway at ~10 Hz drives the
  // Panda-side IC-message-generator. Routed via rx_all because 0x348 is not
  // in preap_rx_checks (the framework only delivers whitelisted msgs to
  // rx_hook). preap_ic_send_messages is internally gated on
  // preap_has_ic_integration → no-op when the flag is off.
  if (bus_num == 0 && addr == 0x348) {
    preap_ic_send_messages();
  }

  if (bus_num == 0 && preap_radar_emulation) {
    // Group A: Simple re-addresses
    switch (addr) {
      case 0x45:   preap_radar_readdr(to_fwd, 0x219); break;  // STW_ACTN_RQ
      case 0x108:  preap_radar_readdr(to_fwd, 0x109); break;  // DI_torque1
      case 0x145:  preap_radar_readdr(to_fwd, 0x149); break;  // ESP_145h
      case 0x20A:  preap_radar_readdr(to_fwd, 0x159); break;  // BrakeMessage -> ESP_C
      case 0x308:  preap_radar_readdr(to_fwd, 0x209); break;  // GTW_odo
      case 0x30A:  preap_radar_readdr(to_fwd, 0x2D9); break;  // BC_status
      case 0x405:  preap_radar_readdr(to_fwd, 0x2B9); break;  // VIP_405HS
      default: break;
    }

    // Group B: GTW_carConfig (0x398) → 0x2A9 with bitfield patching
    if (addr == 0x398) {
      CANPacket_t pkt = {.returned = 0U, .rejected = 0U, .extended = to_fwd->extended,
                         .bus = 1, .addr = 0x2A9, .data_len_code = to_fwd->data_len_code};
      uint32_t lo = PREAP_GET_BYTES_04(to_fwd);
      uint32_t hi = PREAP_GET_BYTES_48(to_fwd);
      lo = (lo & 0xFFFFF33F) | 0x100 | 0x440;  // country=US, radar_type=Bosch
      hi = (hi & 0xCFFF0F0F) | 0x10000000 | (preap_radar_position << 4) | (preap_radar_epas_type << 12);
      PREAP_WORD_TO_BYTES(&pkt.data[0], lo);
      PREAP_WORD_TO_BYTES(&pkt.data[4], hi);
      pkt.data[7] = preap_byte_sum_checksum(&pkt);
#if defined(STM32H7) || defined(STM32F4)
      can_set_checksum(&pkt);
      can_send(&pkt, 1, true);
#endif
    }

    // Group B: STW_ANGLHP_STAT (0x0E) → 0x199 with SNA replacement
    if (addr == 0x0E) {
      CANPacket_t pkt = {.returned = 0U, .rejected = 0U, .extended = to_fwd->extended,
                         .bus = 1, .addr = 0x199, .data_len_code = to_fwd->data_len_code};
      uint32_t lo = PREAP_GET_BYTES_04(to_fwd);
      uint32_t hi = PREAP_GET_BYTES_48(to_fwd);
      if (((lo >> 16) & 0xFF3F) == 0xFF3F) {
        lo = (lo & 0x00C0FFFF) | (0x0020 << 16);
        hi = (hi & 0x00FFFFF0) | 0x00000004;  // force DELPHI sensor ID
        int crc = preap_compute_crc8(lo, hi, 7);
        hi = hi | ((uint32_t)crc << 24);
      }
      PREAP_WORD_TO_BYTES(&pkt.data[0], lo);
      PREAP_WORD_TO_BYTES(&pkt.data[4], hi);
#if defined(STM32H7) || defined(STM32F4)
      can_set_checksum(&pkt);
      can_send(&pkt, 1, true);
#endif
    }

    // Group C: ESP_115h (0x115) → 0x129 + synthetic DI_espControl (0x1A9)
    if (addr == 0x115) {
      preap_radar_readdr(to_fwd, 0x129);
      uint32_t hi_src = PREAP_GET_BYTES_48(to_fwd);
      int counter = ((hi_src & 0xF0) >> 4) & 0x0F;
      uint32_t syn_lo = 0x000C0000U | ((uint32_t)counter << 28);
      int cksm = (0x38 + 0x0C + (counter << 4)) & 0xFF;
      CANPacket_t pkt = {.returned = 0U, .rejected = 0U, .extended = 0,
                         .bus = 1, .addr = 0x1A9, .data_len_code = 5};
      PREAP_WORD_TO_BYTES(&pkt.data[0], syn_lo);
      PREAP_WORD_TO_BYTES(&pkt.data[4], (uint32_t)cksm);
#if defined(STM32H7) || defined(STM32F4)
      can_set_checksum(&pkt);
      can_send(&pkt, 1, true);
#endif
    }

    // Group C: DI_torque2 (0x118) → 0x119 + synthetic ESP_wheelSpeeds (0x169)
    if (addr == 0x118) {
      preap_radar_readdr(to_fwd, 0x119);
      uint32_t lo = PREAP_GET_BYTES_04(to_fwd);
      int ws_counter = PREAP_GET_BYTES_48(to_fwd) & 0x0F;
      int raw_speed = (int)((0xFFF0000U & lo) >> 16);
      int speed;
      if (raw_speed == 0xFFF) {
        speed = 0x1FFF;
      } else {
        int mph_x100 = raw_speed * 5 - 2500;
        int kph_x100 = mph_x100 * 1609 / 1000;
        speed = (kph_x100 < 0) ? 0 : ((kph_x100 / 4) & 0x1FFF);
      }
      uint32_t ws_lo = (uint32_t)(speed | (speed << 13) | (speed << 26));
      uint32_t ws_hi = (uint32_t)((speed >> 6) | (speed << 7) | (ws_counter << 20)) & 0x00FFFFFFU;
      int ws_cksm = 0x76;
      ws_cksm = (ws_cksm + (int)(ws_lo & 0xFF) + (int)((ws_lo >> 8) & 0xFF) + (int)((ws_lo >> 16) & 0xFF) + (int)((ws_lo >> 24) & 0xFF)) & 0xFF;
      ws_cksm = (ws_cksm + (int)(ws_hi & 0xFF) + (int)((ws_hi >> 8) & 0xFF) + (int)((ws_hi >> 16) & 0xFF)) & 0xFF;
      ws_hi = ws_hi | ((uint32_t)ws_cksm << 24);
      CANPacket_t pkt = {.returned = 0U, .rejected = 0U, .extended = 0,
                         .bus = 1, .addr = 0x169, .data_len_code = 8};
      PREAP_WORD_TO_BYTES(&pkt.data[0], ws_lo);
      PREAP_WORD_TO_BYTES(&pkt.data[4], ws_hi);
#if defined(STM32H7) || defined(STM32F4)
      can_set_checksum(&pkt);
      can_send(&pkt, 1, true);
#endif
    }
  }

  // Radar status tracking (CAN1 → informational only)
  if (bus_num == 1 && preap_radar_emulation) {
    if (addr == 0x631 && preap_radar_status == 0) {
      preap_radar_status = 1;
      preap_last_radar_signal = microsecond_timer_get();
    }
    if (addr == 0x300 && preap_radar_status == 1) {
      preap_radar_status = 2;
      preap_last_radar_signal = microsecond_timer_get();
    }
  }
}

// ============================================
// Panda-side IC-message-generator
// ============================================
//
// Re-emits cached Tinkla Buddy IC display frames at Tinkla's multi-rate cadence
// (10Hz/2Hz/1Hz), triggered by GTW_status (0x348) RX. Mirrors Tinkla's
// teslaPreAp_send_IC_messages (safety_tesla.h:805-841) using a tx_hook-capture
// cache instead of fwd-hook capture (we have no AP-ECU on bus 2 to forward from).
//
// All paths are no-op when preap_has_ic_integration is false. The 0x2B9
// DAS_control actuator path and 0x488 DAS_steeringControl actuator path are
// deliberately excluded — they remain native openpilot emissions at full rate.
//
// Sprint: docs/NAP_FIX_PANDA_IC_GENERATOR_SPRINT.md (2026-05-25).

// IC-cache addresses (display-only, all gated by preap_has_ic_integration).
// Indexed by preap_ic_msg_index; KEEP ORDER STABLE — getters use index-mapping.
#define PREAP_IC_CACHE_COUNT 10
static const uint16_t PREAP_IC_CACHE_ADDRS[PREAP_IC_CACHE_COUNT] = {
  0x239,  // DAS_lanes              (10Hz)
  0x3A9,  // DAS_telemetry          (10Hz)
  0x309,  // DAS_object             (10Hz)
  0x3B1,  // DAS_telemetryFurniture (10Hz)
  0x399,  // DAS_status             (2Hz)
  0x389,  // DAS_status2            (2Hz)
  0x3E9,  // DAS_bodyControls       (2Hz)
  0x329,  // DAS_warningMatrix0     (1Hz)
  0x369,  // DAS_warningMatrix1     (1Hz)
  0x349,  // DAS_warningMatrix3     (1Hz)
};

typedef struct {
  uint8_t bytes[8];
  bool valid;
  uint8_t counter;          // current rotation index (mod 16 unless noted)
  uint32_t emission_count;  // monotonic emission count (for test introspection)
} preap_ic_slot_t;

static preap_ic_slot_t preap_ic_cache[PREAP_IC_CACHE_COUNT];
static uint8_t preap_ic_send_counter = 0;  // 0..9, advances per GTW_status RX

static int preap_ic_msg_index(uint32_t addr) {
  for (int i = 0; i < PREAP_IC_CACHE_COUNT; i++) {
    if ((uint32_t)PREAP_IC_CACHE_ADDRS[i] == addr) {
      return i;
    }
  }
  return -1;
}

static void preap_ic_reset_state(void) {
  for (int i = 0; i < PREAP_IC_CACHE_COUNT; i++) {
    for (int j = 0; j < 8; j++) {
      preap_ic_cache[i].bytes[j] = 0;
    }
    preap_ic_cache[i].valid = false;
    preap_ic_cache[i].counter = 0;
    preap_ic_cache[i].emission_count = 0;
  }
  preap_ic_send_counter = 0;
}

// Capture bytes from an allowed IC TX into the cache. Caller (tx_hook) must
// already have verified that tx is allowed AND that preap_has_ic_integration
// is true. Silently ignores non-IC addresses.
static void preap_ic_capture_tx(const CANPacket_t *msg) {
  int idx = preap_ic_msg_index(msg->addr);
  if (idx < 0) return;
  for (int i = 0; i < 8; i++) {
    preap_ic_cache[idx].bytes[i] = msg->data[i];
  }
  preap_ic_cache[idx].valid = true;
}

// Per-address counter bit-mask, matching Tinkla safety_tesla.h:713-738.
//
// data[6] bits 4-7 = upper-nibble of byte 6 = bits 52-55 in the 64-bit payload.
// data[7] bits 4-7 = upper-nibble of byte 7 = bits 60-63.
//
// Returns true if counter applied, false if addr has no counter rotation
// (telemetry/warningMatrix/object/etc.).
static bool preap_ic_apply_counter(uint16_t addr, uint8_t *data, uint8_t counter) {
  // 4-bit counter in upper nibble of byte 7 (DAS_lanesCounter).
  if (addr == 0x239) {
    data[7] = (uint8_t)((data[7] & 0x0FU) | ((counter & 0x0FU) << 4));
    return true;
  }
  // 4-bit counter in upper nibble of byte 6 (DAS_status / status2 / bodyControlsCounter).
  if ((addr == 0x399) || (addr == 0x389) || (addr == 0x3E9)) {
    data[6] = (uint8_t)((data[6] & 0x0FU) | ((counter & 0x0FU) << 4));
    return true;
  }
  // No-counter frames: 0x3A9, 0x309, 0x3B1, 0x329, 0x369, 0x349.
  return false;
}

// Tesla byte-sum checksum stored at byte 7. Applies to 0x399/0x389/0x3E9.
// Other IC addresses do not carry a checksum (Tinkla: tesla_compute_fwd_checksum
// returns valid=true without writing a checksum byte for 0x239/0x329/0x369/0x349
// and the telemetry/object frames).
static bool preap_ic_apply_checksum(uint16_t addr, uint8_t *data) {
  if ((addr == 0x399) || (addr == 0x389) || (addr == 0x3E9)) {
    uint8_t chksum = (uint8_t)(addr & 0xFFU) + (uint8_t)((addr >> 8) & 0xFFU);
    for (int i = 0; i < 7; i++) {
      chksum += data[i];
    }
    data[7] = chksum;
    return true;
  }
  return false;
}

// Emit one IC frame from the cache: load → counter-rotate → checksum → can_send.
// Increments cache.counter and cache.emission_count regardless of can_send
// availability (so test introspection observes scheduling correctly).
static void preap_ic_emit_message(uint16_t addr) {
  int idx = preap_ic_msg_index(addr);
  if (idx < 0) return;
  if (!preap_ic_cache[idx].valid) return;

  uint8_t buf[8];
  for (int i = 0; i < 8; i++) {
    buf[i] = preap_ic_cache[idx].bytes[i];
  }

  if (preap_ic_apply_counter((uint16_t)addr, buf, preap_ic_cache[idx].counter)) {
    preap_ic_cache[idx].counter = (uint8_t)((preap_ic_cache[idx].counter + 1U) & 0x0FU);
  }
  (void)preap_ic_apply_checksum((uint16_t)addr, buf);

#if defined(STM32H7) || defined(STM32F4)
  // 8-byte standard CAN: data_len_code maps 1:1 to length per dlc_to_len[].
  CANPacket_t pkt;
  pkt.returned = 0U;
  pkt.rejected = 0U;
  pkt.extended = 0U;
  pkt.bus = 0;
  pkt.addr = addr;
  pkt.data_len_code = 8U;
  for (int i = 0; i < 8; i++) {
    pkt.data[i] = buf[i];
  }
  can_set_checksum(&pkt);
  can_send(&pkt, 0, true);
#endif

  preap_ic_cache[idx].emission_count++;
}

// Multi-rate IC-dispatcher: called from rx_hook when GTW_status (0x348) arrives
// at ~10 Hz. Mirrors Tinkla's teslaPreAp_send_IC_messages (safety_tesla.h:805).
//
//   @10Hz (every tick):       0x239, 0x3A9, 0x309, 0x3B1
//   @2Hz (counter ∈ {1,6}):   0x399, 0x389, 0x3E9
//   @1Hz (counter == 3):      0x329, 0x369, 0x349
//
// 0x2B9 (DAS_control) and 0x488 (DAS_steeringControl) are NOT in this
// dispatcher — they are actuator paths driven directly by openpilot.
static void preap_ic_send_messages(void) {
  if (!preap_has_ic_integration) return;

  preap_ic_emit_message(0x239);
  preap_ic_emit_message(0x3A9);
  preap_ic_emit_message(0x309);
  preap_ic_emit_message(0x3B1);

  if ((preap_ic_send_counter == 1U) || (preap_ic_send_counter == 6U)) {
    preap_ic_emit_message(0x399);
    preap_ic_emit_message(0x389);
    preap_ic_emit_message(0x3E9);
  }
  if (preap_ic_send_counter == 3U) {
    preap_ic_emit_message(0x329);
    preap_ic_emit_message(0x369);
    preap_ic_emit_message(0x349);
  }

  preap_ic_send_counter = (uint8_t)((preap_ic_send_counter + 1U) % 10U);
}

// ============================================
// Test-only introspection (used by libsafety in opendbc/safety/tests/)
// ============================================
// These are unconditionally compiled — the panda firmware build links them but
// never calls them, so they cost ~50 bytes of flash. Kept simple and read-only.

bool preap_ic_get_cache_valid(uint32_t addr);
uint8_t preap_ic_get_cache_byte(uint32_t addr, uint32_t offset);
uint32_t preap_ic_get_counter(uint32_t addr);
uint32_t preap_ic_get_emission_count(uint32_t addr);

bool preap_ic_get_cache_valid(uint32_t addr) {
  int idx = preap_ic_msg_index(addr);
  return (idx >= 0) ? preap_ic_cache[idx].valid : false;
}

uint8_t preap_ic_get_cache_byte(uint32_t addr, uint32_t offset) {
  int idx = preap_ic_msg_index(addr);
  if (idx < 0) return 0U;
  if (offset >= 8U) return 0U;
  return preap_ic_cache[idx].bytes[offset];
}

uint32_t preap_ic_get_counter(uint32_t addr) {
  int idx = preap_ic_msg_index(addr);
  return (idx >= 0) ? (uint32_t)preap_ic_cache[idx].counter : 0U;
}

uint32_t preap_ic_get_emission_count(uint32_t addr) {
  int idx = preap_ic_msg_index(addr);
  return (idx >= 0) ? preap_ic_cache[idx].emission_count : 0U;
}

// ============================================
// RX Hook
// ============================================

static void tesla_preap_rx_hook(const CANPacket_t *msg) {
  // Pedal interceptor (0x552) — may arrive on bus 0 OR bus 2 depending on wiring.
  // Must be handled BEFORE the bus-0-only bailout below.
  // Whitelisted on both bus 0 and bus 2 in preap_rx_checks; the framework has
  // already verified the message matches one of them, so accept either here.
  //
  // Gas-press threshold: 650 raw, chosen from real Pre-AP drive data:
  //   - At-rest noise (driver not pressing): raw range 424-633, mean 470 (p99.9=602)
  //   - Actual gas press: raw range 441-1246, mean 799 (p10=607, p50=802)
  // The original threshold of 450 was inside the resting noise distribution and
  // caused false gas_pressed readings that blocked pedal TX → pedal wouldn't engage.
  // 650 gives zero false positives on rest noise while still catching the vast
  // majority of real driver presses. Python-layer DI_pedalPos is the primary
  // gas-override detection; the panda threshold here is a safety backstop.
  if (preap_enable_pedal && (msg->addr == 0x552U)) {
    int pedal_val = ((msg->data[0] << 8) | msg->data[1]);
    gas_pressed = (pedal_val > 650);
    if (preap_pedal_can == -1) {
      preap_pedal_can = msg->bus;
    }
    return;
  }

  // All other RX handlers are bus 0 only.
  if (msg->bus != 0U) return;

  // EPAS (0x370): steering angle, hands-on level, disengage detection
  if (msg->addr == 0x370U) {
    const int angle_meas_new = (((msg->data[4] & 0x3FU) << 8) | msg->data[5]) - 8192U;
    update_sample(&angle_meas, angle_meas_new);

    const int hands_on_level = msg->data[4] >> 6;
    const int eac_status = msg->data[6] >> 5;
    const int eac_error_code = msg->data[2] >> 4;

    // Disengage on hands-on override OR EPAS actively rejecting steering commands.
    // Error codes 6/7/8 = EPAS request validators rejected angle/rate, 9 = safety layer.
    // All indicate the EPAS stopped steering — driver must be notified immediately.
    bool epas_rejecting = (eac_status == 0) && (eac_error_code >= 6) && (eac_error_code <= 9);
    steering_disengage = (hands_on_level >= 3) || epas_rejecting;

    // Re-arm fix: force cruise_engaged_prev reset on steering disengage
    // so next stalk pull creates a clean rising edge
    if (steering_disengage && !steering_disengage_prev) {
      pcm_cruise_check(false);
    }
  }

  // Vehicle speed (ESP_B: 0x155) — derive vehicle_moving from actual speed
  if (msg->addr == 0x155U) {
    float speed = (((msg->data[5] << 8) | msg->data[6]) * 0.01f) * KPH_TO_MS;
    UPDATE_VEHICLE_SPEED(speed);
    vehicle_moving = speed > (0.5f * KPH_TO_MS);
  }

  // Gas pressed from DI_torque1 (0x108) — only when pedal interceptor is not active.
  // (The pedal interceptor path is handled above the bus-0-only bailout since it may
  // arrive on bus 0 or bus 2.)
  if (msg->addr == 0x108U) {
    if (!preap_enable_pedal) {
      gas_pressed = msg->data[6] != 0U;
    }
  }

  // Brake (0x20a) — force false so generic_rx_checks doesn't drop controls_allowed.
  // Pre-AP brake → steering-only is handled in the software layer.
  if (msg->addr == 0x20aU) {
    brake_pressed = false;
  }

  // Cruise state (DI_state: 0x368) — vehicle_moving only, engagement via stalk
  if (msg->addr == 0x368U) {
    int cruise_state = (msg->data[1] >> 4) & 0x07U;
    // Backup vehicle_moving from cruise state (standstill detection)
    if (cruise_state == 3) {
      vehicle_moving = false;
    }
  }

  // Gear check (DI_torque2: 0x118) — disable controls on leaving Drive
  if (msg->addr == 0x118U) {
    preap_gear = (msg->data[1] >> 4) & 0x07;
    if ((preap_gear_prev == 4) && (preap_gear != 4)) {
      controls_allowed = false;
    }
    preap_gear_prev = preap_gear;
  }

  // Door check (GTW_carState: 0x318)
  if (msg->addr == 0x318U) {
    int d_fl = (msg->data[1] >> 4) & 0x03;
    int d_fr = (msg->data[1] >> 6) & 0x03;
    int d_rl = (msg->data[2] >> 6) & 0x03;
    int d_rr = (msg->data[3] >> 5) & 0x03;
    int d_ft = (msg->data[6] >> 2) & 0x03;
    int d_tr = (msg->data[5] >> 6) & 0x03;
    preap_doors_open = (d_fl == 1) || (d_fr == 1) || (d_rl == 1) || (d_rr == 1) || (d_ft == 1) || (d_tr == 1);
    if (preap_doors_open) {
      controls_allowed = false;
    }
  }

  // Stalk engagement (STW_ACTN_RQ: 0x45) with echo-filtered cancel
  if (msg->addr == 0x45U) {
    int lever = msg->data[0] & 0x3FU;
    if (lever == 2) {  // RWD = pull toward driver = enable
      if ((preap_gear == 4) && !preap_doors_open) {
        pcm_cruise_check(true);
        preap_last_stalk_engage_us = microsecond_timer_get();
      }
    } else if (lever == 1) {  // FWD = push away = cancel
      uint32_t elapsed = microsecond_timer_get() - preap_last_stalk_engage_us;
      if (elapsed > PREAP_CANCEL_ECHO_WINDOW_US) {
        pcm_cruise_check(false);
      }
    }
  }
}

// ============================================
// TX Hook
// ============================================

static bool tesla_preap_tx_hook(const CANPacket_t *msg) {
  const AngleSteeringLimits PREAP_STEERING_LIMITS = {
    .max_angle = 3600,  // 360 deg, EPAS faults above this
    .angle_deg_to_can = 10,
    .frequency = 50U,
  };

  // Pre-AP Model S is physically the same car as HW1/HW2/HW3 Model S.
  // These values MUST match VehicleModel(TESLA_MODEL_S_HW3) in carcontroller.py.
  // Verified: mass=2100+STD_CARGO_KG, wheelbase=2.960, steerRatio=15.0
  //           → slip_factor = -0.0005666 (calc_slip_factor)
  // Confirmed by Lukas (xnor-tech, former comma employee, Tesla port author).
  const AngleSteeringParams PREAP_STEERING_PARAMS = {
    .slip_factor = -0.0005666,
    .steer_ratio = 15.,
    .wheelbase = 2.96,
  };

  bool tx = true;
  bool violation = false;

  // DAS_steeringControl (0x488)
  if (msg->addr == 0x488U) {
    int raw_angle_can = ((msg->data[0] & 0x7FU) << 8) | msg->data[1];
    int desired_angle = raw_angle_can - 16384;
    int steer_control_type = msg->data[2] >> 6;
    bool steer_control_enabled = steer_control_type == 1;

    if (steer_angle_cmd_checks_vm(desired_angle, steer_control_enabled, PREAP_STEERING_LIMITS, PREAP_STEERING_PARAMS)) {
      violation = true;
    }
    if ((steer_control_type != 0) && (steer_control_type != 1)) {
      violation = true;
    }
  }

  // EPB_epasControl (0x214): only allow valid EAC modes (0=disable, 1=enable)
  if (msg->addr == 0x214U) {
    int epas_control_type = msg->data[0] & 0x07U;  // EPB_epasEACAllow: bits 2:0 of byte 0
    if (epas_control_type > 1) {
      violation = true;
    }
  }

  // DAS_control (0x2B9): no AEB events from openpilot
  if (msg->addr == 0x2B9U) {
    int aeb_event = msg->data[2] & 0x03U;
    if (aeb_event != 0) {
      violation = true;
    }
  }

  // Pedal interceptor (0x551 GAS_COMMAND): parse ENABLE bit and GAS_COMMAND
  // value to distinguish authoritative accel commands from driver-passthrough
  // release commands.
  //   DBC: SG_ ENABLE : 39|1@0+  →  bit 7 of data[4]
  //   DBC: SG_ GAS_COMMAND : 7|16@0+  →  bytes 0-1 big-endian (physical 0 = raw 450)
  //
  //   ENABLE=0: openpilot is releasing control. Comma Pedal ignores GAS_COMMAND
  //   and passes driver's OEM pedal voltage through. NAP's pedal passthrough
  //   feature sends this during driver gas override for a smooth handoff.
  //   Defense-in-depth: we still require the GAS_COMMAND raw value to be at or
  //   below the zero point (raw <= 500, which is ~2.5% physical) so a bugged or
  //   malicious ENABLE=0 + high-value message can't sneak through a potential
  //   Comma Pedal firmware bug.
  //
  //   ENABLE=1: authoritative actuation command. Gated by get_longitudinal_allowed()
  //   (controls_allowed && !gas_pressed_prev).
  if (msg->addr == 0x551U) {
    if (!preap_enable_pedal) {
      violation = true;
    } else {
      bool pedal_enable = (msg->data[4] & 0x80U) != 0U;
      int raw_gas_cmd = (msg->data[0] << 8) | msg->data[1];
      if (pedal_enable) {
        if (!get_longitudinal_allowed()) {
          violation = true;
        }
      } else {
        // ENABLE=0: only allow near-zero GAS_COMMAND values (defense-in-depth).
        // Legitimate passthrough sends physical 0 = raw 450.
        if (raw_gas_cmd > 500) {
          violation = true;
        }
      }
    }
  }

  // Tinkla Buddy IC TX-gate. The 8 display-frames below are emitted only when
  // the NAPTinklaICIntegration param is set (which sets PREAP_FLAG_IC_INTEGRATION
  // → preap_has_ic_integration=true). Default is OFF → these arb-IDs are blocked
  // at the panda even if the Python layer accidentally builds them.
  //
  // 0x3E9 (DAS_bodyControls) and 0x659 (DAS_uds fake-DAS for PreAP) are NOT gated
  // here: Tinkla sends them independently of the IC-toggle (Tinkla paritet,
  // findings §5.3 and §5.4). They remain whitelisted by the TX-allow list above.
  //
  // None of these frames are actuator-paths. They are display-only DAS-* frames
  // sniffed passively by the Buddy EtherCAN MITM and rendered on the Tesla IC.
  // Risk-tier 3: no engage / no vehicle-control effect.
  if (!preap_has_ic_integration) {
    const uint32_t ic_addr = msg->addr;
    if ((ic_addr == 0x239U) || (ic_addr == 0x309U) || (ic_addr == 0x329U) ||
        (ic_addr == 0x349U) || (ic_addr == 0x369U) || (ic_addr == 0x389U) ||
        (ic_addr == 0x399U) || (ic_addr == 0x3A9U) || (ic_addr == 0x3B1U)) {
      tx = false;
    }
  }

  if (violation) {
    tx = false;
  }

  // IC-generator cache capture: only for allowed IC display frames AND only
  // when the IC-integration flag is on. tx==true here means the message has
  // passed all tx_hook validation AND the IC-gate above (which blocks all 8
  // gated addrs when preap_has_ic_integration is false), so capture is safe.
  // 0x2B9 (DAS_control) and 0x488 (DAS_steeringControl) are NOT in the cache
  // address list — actuator paths remain untouched.
  if (tx && preap_has_ic_integration) {
    preap_ic_capture_tx(msg);
  }

  return tx;
}

// ============================================
// Forwarding Hook
// ============================================

static bool tesla_preap_fwd_hook(int bus_num, int addr) {
  (void)bus_num;
  (void)addr;
  // Pre-AP has no AP ECU on bus 2. Block default 0↔2 forwarding to avoid
  // flooding a dead TX queue.
  return true;
}

// ============================================
// Init
// ============================================

static safety_config tesla_preap_init(uint16_t param) {
  preap_enable_pedal = GET_FLAG(param, PREAP_FLAG_ENABLE_PEDAL);
  preap_radar_emulation = GET_FLAG(param, PREAP_FLAG_RADAR_EMULATION);
  preap_radar_behind_nosecone = GET_FLAG(param, PREAP_FLAG_RADAR_BEHIND_NOSECONE);
  preap_has_ic_integration = GET_FLAG(param, PREAP_FLAG_IC_INTEGRATION);

  preap_gear = 4;
  preap_gear_prev = 4;
  preap_doors_open = false;
  preap_pedal_can = -1;
  preap_radar_status = 0;
  preap_last_radar_signal = 0;
  preap_last_stalk_engage_us = 0;
  preap_radar_position = preap_radar_behind_nosecone ? 1 : 0;

  // Reset Panda-side IC-generator state (cache + counters + emission counts).
  preap_ic_reset_state();

  // TX whitelist — no harness relay on Pre-AP
  static const CanMsg PREAP_TX_MSGS[] = {
    {0x488, 0, 4, .check_relay = false, .disable_static_blocking = true},  // DAS_steeringControl
    {0x2B9, 0, 8, .check_relay = false, .disable_static_blocking = true},  // DAS_control
    {0x214, 0, 3, .check_relay = false, .disable_static_blocking = true},  // EPB_epasControl
    {0x551, 0, 6, .check_relay = false, .disable_static_blocking = true},  // Pedal on bus 0
    {0x551, 2, 6, .check_relay = false, .disable_static_blocking = true},  // Pedal on bus 2
    {0x45,  0, 8, .check_relay = false, .disable_static_blocking = true},  // STW_ACTN_RQ (stalk spoof)
    // Tinkla Buddy IC integration frames (gated runtime by preap_has_ic_integration in tx_hook).
    // 0x3E9 and 0x659 are NOT gated by the IC flag (Tinkla-paritet, see findings §5.3/§5.4).
    // All entries use check_relay=false + disable_static_blocking=true for the same reason
    // as the actuator frames above: Pre-AP has no harness relay.
    {0x239, 0, 8, .check_relay = false, .disable_static_blocking = true},  // DAS_lanes
    {0x309, 0, 8, .check_relay = false, .disable_static_blocking = true},  // DAS_object
    {0x329, 0, 8, .check_relay = false, .disable_static_blocking = true},  // DAS_warningMatrix0
    {0x349, 0, 8, .check_relay = false, .disable_static_blocking = true},  // DAS_warningMatrix3
    {0x369, 0, 8, .check_relay = false, .disable_static_blocking = true},  // DAS_warningMatrix1
    {0x389, 0, 8, .check_relay = false, .disable_static_blocking = true},  // DAS_status2
    {0x399, 0, 8, .check_relay = false, .disable_static_blocking = true},  // DAS_status
    {0x3A9, 0, 8, .check_relay = false, .disable_static_blocking = true},  // DAS_telemetry
    {0x3B1, 0, 8, .check_relay = false, .disable_static_blocking = true},  // DAS_telemetryFurniture
    {0x3E9, 0, 8, .check_relay = false, .disable_static_blocking = true},  // DAS_bodyControls (ungated)
    {0x659, 0, 8, .check_relay = false, .disable_static_blocking = true},  // DAS_uds (PreAP fake DAS, ungated)
  };

  // RX checks — disable EPAS counter/checksum until we verify the Pre-AP
  // EPAS firmware's checksum matches our compute_checksum exactly.
  // Mismatched validation caused silent 21s steering dropout.
  static RxCheck preap_rx_checks[] = {
    {.msg = {{0x370, 0, 8, 25U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},   // EPAS_sysStatus
    {.msg = {{0x108, 0, 8, 100U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},  // DI_torque1
    {.msg = {{0x118, 0, 6, 100U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},  // DI_torque2
    {.msg = {{0x20a, 0, 8, 50U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},   // BrakeMessage
    {.msg = {{0x368, 0, 8, 10U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},   // DI_state
    {.msg = {{0x318, 0, 8, 10U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},   // GTW_carState
    {.msg = {{0x45,  0, 8, 10U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},   // STW_ACTN_RQ
    {.msg = {{0x155, 0, 8, 50U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},   // ESP_B
    // Pedal interceptor — must be in rx_checks or the framework won't pass it to the rx hook.
    // Listed on both bus 0 and bus 2 to support either wiring configuration.
    // Frequency=0 because pedal may not be present on all cars.
    {.msg = {{0x552, 0, 6, 0U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true},
             {0x552, 2, 6, 0U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }}},  // GAS_SENSOR (pedal interceptor)
  };

  return BUILD_SAFETY_CFG(preap_rx_checks, PREAP_TX_MSGS);
}

// ============================================
// Hooks struct
// ============================================

const safety_hooks tesla_preap_hooks = {
  .init = tesla_preap_init,
  .rx = tesla_preap_rx_hook,
  .rx_all = tesla_preap_gtw_emulation,  // must see ALL CAN traffic for radar GTW forwarding
  .tx = tesla_preap_tx_hook,
  .fwd = tesla_preap_fwd_hook,
  .get_counter = tesla_preap_get_counter,
  .get_checksum = tesla_preap_get_checksum,
  .compute_checksum = tesla_preap_compute_checksum,
  .get_quality_flag_valid = NULL,
};
