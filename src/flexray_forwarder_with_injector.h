#ifndef FLEXRAY_FORWARDER_WITH_INJECTOR_H
#define FLEXRAY_FORWARDER_WITH_INJECTOR_H

#include <stdint.h>
#include <stdbool.h>
#include "hardware/pio.h"

// Cache a frame's raw bytes (header + payload + CRC) when rules match.
void try_cache_last_target_frame(uint16_t frame_id, uint8_t cycle_count, uint16_t frame_length, uint8_t *captured_bytes);

// On receiving a frame, check triggers; if matched, inject using the cached target frame template.
void try_inject_frame(uint16_t frame_id, uint8_t cycle_count);

void setup_forwarder_with_injector(PIO pio,
    uint rx_pin_from_ecu, uint tx_pin_to_vehicle,
    uint rx_pin_from_vehicle, uint tx_pin_to_ecu);

// Submit a host-provided payload override.
// bytes must contain the DBC-form payload, where byte 0 is the synthetic cycle count
// and the remaining bytes map onto the on-wire FlexRay payload.
// The queued payload is applied to the cached target frame when a matching masked cycle slot is triggered.
bool injector_submit_override(uint16_t id, uint8_t base, uint16_t len, const uint8_t *bytes);

// Enable/disable injection at runtime
void injector_set_enabled(bool enabled);
bool injector_is_enabled(void);


#endif // FLEXRAY_FORWARDER_WITH_INJECTOR_H
