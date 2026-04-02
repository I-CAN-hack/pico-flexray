#ifndef FLEXRAY_INJECTOR_RULES_H
#define FLEXRAY_INJECTOR_RULES_H

#include <stdint.h>

#define INJECT_DIRECTION_TO_VEHICLE 1
#define INJECT_DIRECTION_TO_ECU 0
typedef struct {
	uint16_t trigger_id;    // when this id arrives...
	uint16_t target_id;  // ...inject using the cached template of this id
	uint8_t cycle_mask;
	uint8_t cycle_base;
	uint8_t replace_offset;
	uint8_t replace_len;
	uint8_t direction;
} trigger_rule_t;

static const trigger_rule_t INJECT_TRIGGERS[] = {
	// I connect the ECU side to the Domain Controller, so reverse the direction
	{
		.trigger_id = 0x47,
		.target_id = 0x48,
		.cycle_mask = 0b11,
		.cycle_base = 1,
		.replace_offset = 0,
		.replace_len = 16,
		.direction = INJECT_DIRECTION_TO_ECU,
	},
	{
		.trigger_id = 0x42,
		.target_id = 0x44,
		.cycle_mask = 0b01,
		.cycle_base = 0,
		.replace_offset = 0,
		.replace_len = 16,
		.direction = INJECT_DIRECTION_TO_ECU,
	},
};

#define NUM_TRIGGER_RULES (sizeof(INJECT_TRIGGERS)/sizeof(INJECT_TRIGGERS[0]))

#endif // FLEXRAY_INJECTOR_RULES_H
