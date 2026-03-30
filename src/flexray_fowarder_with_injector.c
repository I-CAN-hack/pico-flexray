#include <stdint.h>
#include <string.h>
#include "hardware/dma.h"
#include "flexray_frame.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"


#include "flexray_forwarder_with_injector.pio.h"
#include "flexray_forwarder_with_injector.h"
#include "flexray_injector_rules.h"

static PIO pio_forwarder_with_injector;
static uint sm_forwarder_with_injector_to_vehicle;
static uint sm_forwarder_with_injector_to_ecu;

static int dma_inject_chan_to_vehicle = -1;
static int dma_inject_chan_to_ecu = -1;
static dma_channel_config injector_to_vehicle_dc;
static dma_channel_config injector_to_ecu_dc;

// rules now come from flexray_injector_rules.h

typedef struct {
    uint8_t valid;
    uint16_t len;
    uint8_t data[MAX_FRAME_PAYLOAD_BYTES + 8];
} frame_template_t;

static frame_template_t TEMPLATES[NUM_TRIGGER_RULES];

typedef struct {
    uint8_t valid;
    uint16_t id;
    uint8_t mask;
    uint8_t base;
    uint16_t len;
    uint8_t data[MAX_FRAME_PAYLOAD_BYTES + 8];
} host_override_t;

#define HOST_OVERRIDE_CAP 4
static volatile uint32_t host_override_head = 0;
static volatile uint32_t host_override_tail = 0;
static host_override_t host_overrides[HOST_OVERRIDE_CAP];
static volatile bool injector_enabled = true;

static inline bool host_override_push(uint16_t id, uint8_t mask, uint8_t base, uint16_t len, const uint8_t *bytes)
{
    if (len > sizeof(host_overrides[0].data)) {
        return false;
    }

    uint32_t next_head = (host_override_head + 1u) % HOST_OVERRIDE_CAP;
    if (next_head == host_override_tail) {
        // drop oldest on overflow
        host_override_tail = (host_override_tail + 1u) % HOST_OVERRIDE_CAP;
    }
    host_override_t *slot = &host_overrides[host_override_head];
    slot->id = id;
    slot->mask = mask;
    slot->base = base;
    slot->len = len;
    memcpy(slot->data, bytes, len);
    __atomic_store_n(&slot->valid, 1, __ATOMIC_RELEASE);
    host_override_head = next_head;
    return true;
}

static inline bool host_override_try_pop_for(uint16_t id, uint8_t cycle_count, uint8_t *out, uint16_t *out_len)
{
    uint32_t t = host_override_tail;
    while (t != host_override_head) {
        host_override_t *slot = &host_overrides[t];
        uint8_t v = __atomic_load_n(&slot->valid, __ATOMIC_ACQUIRE);
        if (v && slot->id == id && (uint8_t)(cycle_count & slot->mask) == slot->base) {
            memcpy(out, slot->data, slot->len);
            // invalidate and advance tail to this+1
            slot->valid = 0;
            host_override_tail = (t + 1u) % HOST_OVERRIDE_CAP;
            *out_len = slot->len;
            return true;
        }
        t = (t + 1u) % HOST_OVERRIDE_CAP;
    }
    return false;
}

static inline const trigger_rule_t *find_rule_for_id_and_base(uint16_t id, uint8_t base) {
    for (int i = 0; i < (int)NUM_TRIGGER_RULES; i++) {
        if (INJECT_TRIGGERS[i].target_id == id && INJECT_TRIGGERS[i].cycle_base == base) {
            return &INJECT_TRIGGERS[i];
        }
    }
    return NULL;
}

static inline int find_cache_slot_for_id(uint16_t id, uint8_t cycle_count)
{
    for (int i = 0; i < (int)NUM_TRIGGER_RULES; i++) {
        if (INJECT_TRIGGERS[i].target_id == id &&
            (uint8_t)(cycle_count & INJECT_TRIGGERS[i].cycle_mask) == INJECT_TRIGGERS[i].cycle_base) {
            return i;
        }
    }
    return -1;
}

void try_cache_last_target_frame(uint16_t frame_id, uint8_t cycle_count, uint16_t frame_len, uint8_t *captured_bytes)
{
    int slot = find_cache_slot_for_id(frame_id, cycle_count);
    if (slot < 0) {
        return;
    }

    if (frame_len > sizeof(TEMPLATES[slot].data)) {
        return;
    }

    memcpy(TEMPLATES[slot].data, captured_bytes, frame_len);
    TEMPLATES[slot].len = frame_len;
    TEMPLATES[slot].valid = 1;
}

static void fix_cycle_count(uint8_t *full_frame, uint8_t cycle_count)
{
    full_frame[4] = (full_frame[4] & 0xC0u) | (cycle_count & 0x3Fu);
}

static void inject_frame(uint8_t *full_frame, uint16_t injector_payload_length, uint8_t direction)
{
    // first word is length indicator, rest is payload
    // pio y-- need pre-sub 1 from length
    if (direction == INJECT_DIRECTION_TO_VEHICLE) {
    pio_sm_put(pio_forwarder_with_injector, sm_forwarder_with_injector_to_vehicle, injector_payload_length - 1);
    dma_channel_set_read_addr((uint)dma_inject_chan_to_vehicle, (const void *)full_frame, false);
    dma_channel_set_trans_count((uint)dma_inject_chan_to_vehicle, injector_payload_length / 4, true);
    } else if (direction == INJECT_DIRECTION_TO_ECU) {
    pio_sm_put(pio_forwarder_with_injector, sm_forwarder_with_injector_to_ecu, injector_payload_length - 1);
    dma_channel_set_read_addr((uint)dma_inject_chan_to_ecu, (const void *)full_frame, false);
    dma_channel_set_trans_count((uint)dma_inject_chan_to_ecu, injector_payload_length / 4, true);
    } else {
        return;
    }
}

static uint8_t override_payload_bytes[MAX_FRAME_PAYLOAD_BYTES];
void __time_critical_func(try_inject_frame)(uint16_t frame_id, uint8_t cycle_count)
{
    // Find any trigger where current frame is the "previous" id
    for (int i = 0; i < (int)NUM_TRIGGER_RULES; i++) {
        if (INJECT_TRIGGERS[i].trigger_id != frame_id){
            continue;
        }
        if ((uint8_t)(cycle_count & INJECT_TRIGGERS[i].cycle_mask) != INJECT_TRIGGERS[i].cycle_base){
            continue;
        }

        int target_slot = find_cache_slot_for_id(INJECT_TRIGGERS[i].target_id, cycle_count);
        if (target_slot < 0) {
            continue;
        }

        frame_template_t *tpl = &TEMPLATES[target_slot];
        if (!tpl->valid || tpl->len < 8) {
            continue;
        }

        uint16_t override_payload_len = 0;
        bool has_data = host_override_try_pop_for(INJECT_TRIGGERS[i].target_id, cycle_count, override_payload_bytes, &override_payload_len);
        if (!has_data) {
            continue;
        }

        if (override_payload_len != INJECT_TRIGGERS[i].replace_len) {
            continue;
        }

        memcpy(tpl->data + 5 + INJECT_TRIGGERS[i].replace_offset, override_payload_bytes, override_payload_len);
        fix_cycle_count(tpl->data, cycle_count);
        fix_flexray_frame_crc(tpl->data, tpl->len);
        inject_frame(tpl->data, tpl->len, INJECT_TRIGGERS[i].direction);
        break; // fire once per triggering frame
        
    }
}

static void setup_dma(void){
    dma_inject_chan_to_vehicle = (int)dma_claim_unused_channel(true);
    dma_inject_chan_to_ecu = (int)dma_claim_unused_channel(true);
    
    injector_to_vehicle_dc = dma_channel_get_default_config((uint)dma_inject_chan_to_vehicle);
    channel_config_set_transfer_data_size(&injector_to_vehicle_dc, DMA_SIZE_32);
    channel_config_set_bswap(&injector_to_vehicle_dc, true);
    channel_config_set_read_increment(&injector_to_vehicle_dc, true);
    channel_config_set_write_increment(&injector_to_vehicle_dc, false);
    channel_config_set_dreq(&injector_to_vehicle_dc, pio_get_dreq(pio_forwarder_with_injector, sm_forwarder_with_injector_to_vehicle, true)); // TX pacing
    dma_channel_set_config((uint)dma_inject_chan_to_vehicle, &injector_to_vehicle_dc, false);        
    dma_channel_set_write_addr((uint)dma_inject_chan_to_vehicle, (void *)&pio2->txf[sm_forwarder_with_injector_to_vehicle], false);
    
    injector_to_ecu_dc = dma_channel_get_default_config((uint)dma_inject_chan_to_ecu);
    channel_config_set_transfer_data_size(&injector_to_ecu_dc, DMA_SIZE_32);
    channel_config_set_bswap(&injector_to_ecu_dc, true);
    channel_config_set_read_increment(&injector_to_ecu_dc, true);
    channel_config_set_write_increment(&injector_to_ecu_dc, false);
    channel_config_set_dreq(&injector_to_ecu_dc, pio_get_dreq(pio_forwarder_with_injector, sm_forwarder_with_injector_to_ecu, true)); // TX pacing
    dma_channel_set_config((uint)dma_inject_chan_to_ecu, &injector_to_ecu_dc, false);        
    dma_channel_set_write_addr((uint)dma_inject_chan_to_ecu, (void *)&pio2->txf[sm_forwarder_with_injector_to_ecu], false);

}

bool injector_submit_override(uint16_t id, uint8_t base, uint16_t len, const uint8_t *bytes)
{
    if (bytes == NULL) {
        return false;
    }

    if (len < 1 || len > MAX_FRAME_PAYLOAD_BYTES + 1) {
        return false;
    }

    const trigger_rule_t *matched_rule = find_rule_for_id_and_base(id, base);
    if (matched_rule == NULL) {
        return false;
    }

    uint16_t payload_len = (uint16_t)(len - 1u);
    if ((uint8_t)(bytes[0] & matched_rule->cycle_mask) != matched_rule->cycle_base) {
        return false;
    }

    if (payload_len != (uint16_t)(matched_rule->replace_offset + matched_rule->replace_len)) {
        return false;
    }

    return host_override_push(
        id,
        matched_rule->cycle_mask,
        matched_rule->cycle_base,
        matched_rule->replace_len,
        bytes + 1 + matched_rule->replace_offset
    );
}

void injector_set_enabled(bool enabled)
{
    injector_enabled = enabled;
}

bool injector_is_enabled(void)
{
    return injector_enabled;
}

void setup_forwarder_with_injector(PIO pio,
    uint rx_pin_from_ecu, uint tx_pin_to_vehicle,
    uint rx_pin_from_vehicle, uint tx_pin_to_ecu)
{
    pio_forwarder_with_injector = pio;
    uint offset = pio_add_program(pio, &flexray_forwarder_with_injector_program);
    sm_forwarder_with_injector_to_vehicle = pio_claim_unused_sm(pio, true);
    sm_forwarder_with_injector_to_ecu = pio_claim_unused_sm(pio, true);

    flexray_forwarder_with_injector_program_init(pio, sm_forwarder_with_injector_to_vehicle, offset, rx_pin_from_ecu, tx_pin_to_vehicle);
    flexray_forwarder_with_injector_program_init(pio, sm_forwarder_with_injector_to_ecu, offset, rx_pin_from_vehicle, tx_pin_to_ecu);
    setup_dma();
}
