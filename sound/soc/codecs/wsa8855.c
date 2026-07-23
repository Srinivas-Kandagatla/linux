// SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause)
// Copyright (c) 2025 Qualcomm Technologies, Inc. All rights reserved.

/*
 * Qualcomm WSA8855 SDCA class-compliant Class-D speaker amplifier driver.
 *
 * WSA8855 is a SoundWire SDCA slave (SimpleAmp function).  This driver owns
 * its own struct sdw_driver/id_table and reuses the generic SDCA class
 * implementation exported by sdca_class.c (sdca_class_probe(),
 * sdca_class_read_prop(), sdca_class_pm helpers) for the SoundWire probe,
 * regmap and PM plumbing.  Device-specific hardware bring-up (supplies,
 * powerdown GPIO), static function-data injection (on non-DisCo platforms),
 * and per-PDE hooks live here.
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pm.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/soundwire/sdw.h>
#include <linux/soundwire/sdw_registers.h>
#include <linux/soundwire/sdw_type.h>
#include <sound/sdca.h>
#include <sound/sdca_function.h>
#include "../sdca/sdca_class.h"

/* Forward declaration of entities array */
static struct sdca_entity wsa8855_entities[];

/*
 * Entity array index map.  Kept in ASL order with Entity 0 (Function) last.
 *
 *  [0]  E001 IT 21     (0x1)   [8]  E009 FU 23    (0x9)
 *  [1]  E002 CS 21     (0x2)   [9]  E00A PDE 23   (0xA)
 *  [2]  E003 PPU 21    (0x3)   [10] E00B OT 23    (0xB)
 *  [3]  E004 FU 21     (0x4)   [11] E00C IT 29    (0xC)
 *  [4]  E005 MFPU 21   (0x5)   [12] E00D XU 24    (0xD)
 *  [5]  E006 XU 22     (0x6)   [13] E00E CS 24    (0xE)
 *  [6]  E007 SAPU 29   (0x7)   [14] E00F OT 24    (0xF)
 *  [7]  E008 UDMPU 23  (0x8)   [15] E000 Function (0x0)
 */
#define WSA_IT21	0
#define WSA_CS21	1
#define WSA_PPU21	2
#define WSA_FU21	3
#define WSA_MFPU21	4
#define WSA_XU22	5
#define WSA_SAPU29	6
#define WSA_UDMPU23	7
#define WSA_FU23	8
#define WSA_PDE23	9
#define WSA_OT23	10
#define WSA_IT29	11
#define WSA_XU24	12
#define WSA_CS24	13
#define WSA_OT24	14

/*
 * Range Data Structures
 */

/* Entity 1 (IT 21) Ranges */
static u32 range_it21_usage_data[] = {
	0x0, 0x19A, 0xBB80, 0x10, 0x0, 0x0, 0x0,	/* 48kHz, 16-bit */
	0x0, 0x19A, 0xBB80, 0x18, 0x0, 0x0, 0x0,	/* 48kHz, 24-bit */
	0x0, 0x19A, 0xBB80, 0x20, 0x0, 0x0, 0x0,	/* 48kHz, 32-bit */
};

static u32 range_it21_cluster_data[] = { 0x1, 0x1 };

static u32 range_it21_dp_data[] = {
	0x1, 0x1, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF
};

/* Entity 2 (CS 21) Ranges */
static u32 range_cs21_sr_data[] = { 0x4, 0xBB80 };

/* Entity 3 (PPU 21) Ranges */
static u32 range_ppu21_posture_data[] = {
	0x0, 0x167, 0x0, 0x167, 0x0, 0x167, 0x0, 0x167, 0x0, 0x1, 0x1
};

/* Entity 4 (FU 21) Ranges */
static u32 range_fu21_vol_data[] = { 0xAC00, 0x0, 0x1E };

/* Entity 8 (UDMPU 23) Ranges */
static u32 range_udmpu23_cluster_data[] = { 0x1, 0x1 };

/* Entity 0xA (PDE 23) Ranges */
static u32 range_pde_req_ps_data[] = { 0x0, 0x3 };

/* Entity 0xB (OT 23) Ranges */
static u32 range_ot23_usage_data[] = {
	0x3, 0x2BC, 0xBB80, 0x10, 0x0, 0x0, 0x0,
	0x3, 0x2BC, 0xBB80, 0x20, 0x0, 0x0, 0x0,
};

/* Entity 0xC (IT 29) Ranges */
static u32 range_it29_cluster_data[] = { 0x1, 0x2 };

/* Entity 0xE (CS 24) Ranges */
static u32 range_cs24_sr_data[] = { 0x3, 0xBB80 };

/* Entity 0xF (OT 24) Ranges */
static u32 range_ot24_usage_data[] = {
	0x3, 0x2BC, 0xBB80, 0x10, 0x0, 0x0, 0x0,
	0x3, 0x2BC, 0xBB80, 0x18, 0x0, 0x0, 0x0,
	0x3, 0x2BC, 0xBB80, 0x20, 0x0, 0x0, 0x0,
};

static u32 range_ot24_dp_data[] = {
	0xFF, 0xFF, 0xFF, 0x3,  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF
};

/*
 * Control Value Arrays
 */

/* Entity 0 (Function) */
static int ctrl_fun_sdca_ver_vals[] = { 0x11 };
static int ctrl_fun_type_vals[] = { 0x02 };	/* SimpleAmp */
static int ctrl_fun_man_id_vals[] = { 0x0217 };
static int ctrl_fun_id_vals[] = { 0x1 };
static int ctrl_fun_ver_vals[] = { 0x0 };
static int ctrl_fun_ext_id_vals[] = { 0x2210 };
static int ctrl_fun_ext_ver_vals[] = { 0x1 };
static int ctrl_dev_man_id_vals[] = { 0x217 };
static int ctrl_dev_part_id_vals[] = { 0x25 };
static int ctrl_dev_ver_vals[] = { 0x20 };
static int ctrl_dev_sdca_ver_vals[] = { 0x11 };

/* Entity 1 (IT 21) */
static int ctrl_it21_latency_vals[] = { 0x0 };
static int ctrl_it21_cluster_vals[] = { 0x1 };
static int ctrl_it21_dp_vals[] = { 0x1 };

/* Entity 2 (CS 21) */
static int ctrl_cs21_sr_vals[] = { 0x4 };

/* Entity 3 (PPU 21) */
static int ctrl_ppu21_posture_vals[] = { 0x1 };

/* Entity 4 (FU 21) */
static int ctrl_fu21_mute_vals[] = { 0x1 };
static int ctrl_fu21_vol_vals[] = { 0x0, 0x0 };

/* Entity 5 (MFPU 21) */
static int ctrl_mfpu21_bypass_vals[] = { 0x1 };

/* Entity 6 (XU 22) */
static int ctrl_xu22_bypass_vals[] = { 0x1 };
static int ctrl_xu22_id_vals[] = { 0x2210 };
static int ctrl_xu22_ver_vals[] = { 0x1 };

/* Entity 8 (UDMPU 23) */
static int ctrl_udmpu23_cluster_vals[] = { 0x1 };

/* Entity 0xC (IT 29) */
static int ctrl_it29_cluster_vals[] = { 0x1 };

/* Entity 0xD (XU 24) */
static int ctrl_xu24_bypass_vals[] = { 0x1 };
static int ctrl_xu24_id_vals[] = { 0x2210 };
static int ctrl_xu24_ver_vals[] = { 0x1 };

/* Entity 0xF (OT 24) */
static int ctrl_ot24_latency_vals[] = { 0x0 };
static int ctrl_ot24_dp_vals[] = { 0x3 };

/* Entity 0 (Function) Controls */
static struct sdca_control entity0_controls[] = {
	{ .sel = 0x1,  .mode = SDCA_ACCESS_MODE_RW, .layers = 0x4, .cn_list = 0x1,
	  .label = SDCA_CTL_COMMIT_GROUP_MASK_NAME },
	{ .sel = 0x4,  .mode = SDCA_ACCESS_MODE_DC, .layers = 0x4, .cn_list = 0x1,
	  .values = ctrl_fun_sdca_ver_vals, .has_fixed = true,
	  .label = SDCA_CTL_FUNCTION_SDCA_VERSION_NAME },
	{ .sel = 0x5,  .mode = SDCA_ACCESS_MODE_DC, .layers = 0x4, .cn_list = 0x1,
	  .values = ctrl_fun_type_vals, .has_fixed = true,
	  .label = SDCA_CTL_FUNCTION_TYPE_NAME },
	{ .sel = 0x6,  .mode = SDCA_ACCESS_MODE_DC, .layers = 0x4, .cn_list = 0x1,
	  .values = ctrl_fun_man_id_vals, .has_fixed = true,
	  .label = SDCA_CTL_FUNCTION_MANUFACTURER_ID_NAME },
	{ .sel = 0x7,  .mode = SDCA_ACCESS_MODE_DC, .layers = 0x4, .cn_list = 0x1,
	  .values = ctrl_fun_id_vals, .has_fixed = true,
	  .label = SDCA_CTL_FUNCTION_ID_NAME },
	{ .sel = 0x8,  .mode = SDCA_ACCESS_MODE_DC, .layers = 0x4, .cn_list = 0x1,
	  .values = ctrl_fun_ver_vals, .has_fixed = true,
	  .label = SDCA_CTL_FUNCTION_VERSION_NAME },
	{ .sel = 0x9,  .mode = SDCA_ACCESS_MODE_DC, .layers = 0x4, .cn_list = 0x1,
	  .values = ctrl_fun_ext_id_vals, .has_fixed = true,
	  .label = SDCA_CTL_FUNCTION_EXTENSION_ID_NAME },
	{ .sel = 0xA,  .mode = SDCA_ACCESS_MODE_DC, .layers = 0x4, .cn_list = 0x1,
	  .values = ctrl_fun_ext_ver_vals, .has_fixed = true,
	  .label = SDCA_CTL_FUNCTION_EXTENSION_VERSION_NAME },
	{ .sel = 0x10, .mode = SDCA_ACCESS_MODE_RW, .layers = 0x4, .cn_list = 0x1,
	  .label = SDCA_CTL_FUNCTION_STATUS_NAME },
	{ .sel = 0x11, .mode = SDCA_ACCESS_MODE_RW, .layers = 0x4, .cn_list = 0x1,
	  .label = SDCA_CTL_FUNCTION_ACTION_NAME },
	{ .sel = 0x2C, .mode = SDCA_ACCESS_MODE_DC, .layers = 0x4, .cn_list = 0x1,
	  .values = ctrl_dev_man_id_vals, .has_fixed = true,
	  .label = SDCA_CTL_DEVICE_MANUFACTURER_ID_NAME },
	{ .sel = 0x2D, .mode = SDCA_ACCESS_MODE_DC, .layers = 0x4, .cn_list = 0x1,
	  .values = ctrl_dev_part_id_vals, .has_fixed = true,
	  .label = SDCA_CTL_DEVICE_PART_ID_NAME },
	{ .sel = 0x2E, .mode = SDCA_ACCESS_MODE_DC, .layers = 0x4, .cn_list = 0x1,
	  .values = ctrl_dev_ver_vals, .has_fixed = true,
	  .label = SDCA_CTL_DEVICE_VERSION_NAME },
	{ .sel = 0x2F, .mode = SDCA_ACCESS_MODE_DC, .layers = 0x4, .cn_list = 0x1,
	  .values = ctrl_dev_sdca_ver_vals, .has_fixed = true,
	  .label = SDCA_CTL_DEVICE_SDCA_VERSION_NAME },
};

/* Entity 1 (IT 21) Controls */
static struct sdca_control entity_it21_controls[] = {
	{ .sel = 0x4,  .mode = SDCA_ACCESS_MODE_RW, .layers = 0x4, .cn_list = 0x1,
	  .range = { .cols = 0x7, .rows = 0x3, .data = range_it21_usage_data },
	  .label = SDCA_CTL_USAGE_NAME },
	{ .sel = 0x8,  .mode = SDCA_ACCESS_MODE_DC, .layers = 0x4, .cn_list = 0x1,
	  .values = ctrl_it21_latency_vals, .has_fixed = true,
	  .label = SDCA_CTL_LATENCY_NAME },
	{ .sel = 0x10, .mode = SDCA_ACCESS_MODE_RW, .layers = 0x4, .cn_list = 0x1,
	  .values = ctrl_it21_cluster_vals, .has_fixed = true,
	  .range = { .cols = 0x2, .rows = 0x1, .data = range_it21_cluster_data },
	  .label = SDCA_CTL_CLUSTERINDEX_NAME },
	{ .sel = 0x11, .mode = SDCA_ACCESS_MODE_DC, .layers = 0x4, .cn_list = 0x1,
	  .values = ctrl_it21_dp_vals, .has_fixed = true,
	  .range = { .cols = 0x10, .rows = 0x4, .data = range_it21_dp_data },
	  .label = SDCA_CTL_DATAPORT_SELECTOR_NAME },
};

/* Entity 2 (CS 21) Controls */
static struct sdca_control entity_cs21_controls[] = {
	{ .sel = 0x2,  .mode = SDCA_ACCESS_MODE_RO, .layers = 0x4, .cn_list = 0x1,
	  .is_volatile = true, .label = SDCA_CTL_CLOCK_VALID_NAME },
	{ .sel = 0x10, .mode = SDCA_ACCESS_MODE_DC, .layers = 0x4, .cn_list = 0x1,
	  .values = ctrl_cs21_sr_vals, .has_fixed = true,
	  .range = { .cols = 0x2, .rows = 0x1, .data = range_cs21_sr_data },
	  .label = SDCA_CTL_SAMPLERATEINDEX_NAME },
};

/* Entity 3 (PPU 21) Controls */
static struct sdca_entity *entity_ppu21_sources[] = { &wsa8855_entities[WSA_IT21] };

static struct sdca_control entity_ppu21_controls[] = {
	{ .sel = 0x10, .mode = SDCA_ACCESS_MODE_DC, .layers = 0x4, .cn_list = 0x1,
	  .values = ctrl_ppu21_posture_vals, .has_fixed = true,
	  .range = { .cols = 0xB, .rows = 0x1, .data = range_ppu21_posture_data },
	  .label = SDCA_CTL_POSTURENUMBER_NAME },
};

/* Entity 4 (FU 21) Controls */
static struct sdca_entity *entity_fu21_sources[] = { &wsa8855_entities[WSA_PPU21] };

static struct sdca_control entity_fu21_controls[] = {
	{ .sel = 0x1, .mode = SDCA_ACCESS_MODE_DUAL, .layers = 0x3, .cn_list = 0x1,
	  .values = ctrl_fu21_mute_vals, .has_fixed = true, .label = SDCA_CTL_MUTE_NAME },
	{ .sel = 0x2, .mode = SDCA_ACCESS_MODE_DUAL, .layers = 0x3, .cn_list = 0x6,
	  .values = ctrl_fu21_vol_vals, .has_fixed = true, .nbits = 16,
	  .range = { .cols = 0x3, .rows = 0x1, .data = range_fu21_vol_data },
	  .label = SDCA_CTL_CHANNEL_VOLUME_NAME },
};

/* Entity 5 (MFPU 21) Controls */
static struct sdca_entity *entity_mfpu21_sources[] = { &wsa8855_entities[WSA_FU21] };

static struct sdca_control entity_mfpu21_controls[] = {
	{ .sel = 0x1, .mode = SDCA_ACCESS_MODE_DC, .layers = 0x4, .cn_list = 0x1,
	  .values = ctrl_mfpu21_bypass_vals, .has_fixed = true,
	  .label = SDCA_CTL_BYPASS_NAME },
};

/* Entity 6 (XU 22) Controls */
static struct sdca_entity *entity_xu22_sources[] = { &wsa8855_entities[WSA_MFPU21] };

static struct sdca_control entity_xu22_controls[] = {
	{ .sel = 0x1, .mode = SDCA_ACCESS_MODE_DC, .layers = 0x4, .cn_list = 0x1,
	  .values = ctrl_xu22_bypass_vals, .has_fixed = true,
	  .label = SDCA_CTL_BYPASS_NAME },
	{ .sel = 0x7, .mode = SDCA_ACCESS_MODE_DC, .layers = 0x4, .cn_list = 0x1,
	  .values = ctrl_xu22_id_vals, .has_fixed = true, .label = SDCA_CTL_XU_ID_NAME },
	{ .sel = 0x8, .mode = SDCA_ACCESS_MODE_DC, .layers = 0x4, .cn_list = 0x1,
	  .values = ctrl_xu22_ver_vals, .has_fixed = true, .label = SDCA_CTL_XU_VERSION_NAME },
};

/* Entity 7 (SAPU 29) - no controls */
static struct sdca_entity *entity_sapu29_sources[] = {
	&wsa8855_entities[WSA_XU22],
	&wsa8855_entities[WSA_IT29],
};

/* Entity 8 (UDMPU 23) Controls */
static struct sdca_entity *entity_udmpu23_sources[] = { &wsa8855_entities[WSA_SAPU29] };

static struct sdca_control entity_udmpu23_controls[] = {
	{ .sel = 0x10, .mode = SDCA_ACCESS_MODE_DC, .layers = 0x4, .cn_list = 0x1,
	  .values = ctrl_udmpu23_cluster_vals, .has_fixed = true,
	  .range = { .cols = 0x2, .rows = 0x1, .data = range_udmpu23_cluster_data },
	  .label = SDCA_CTL_CLUSTERINDEX_NAME },
};

/* Entity 9 (FU 23) - no controls */
static struct sdca_entity *entity_fu23_sources[] = { &wsa8855_entities[WSA_UDMPU23] };

/* Entity 0xA (PDE 23) - manages OT 23 (speaker output) */
static struct sdca_entity *entity_pde23_managed[] = { &wsa8855_entities[WSA_OT23] };

static struct sdca_pde_delay pde23_delays[] = {
	{ .from_ps = 3, .to_ps = 0, .us = 3000 },
	{ .from_ps = 0, .to_ps = 3, .us = 10000 },
};

static struct sdca_control entity_pde23_controls[] = {
	{ .sel = 0x1,  .mode = SDCA_ACCESS_MODE_RW, .layers = SDCA_ACCESS_LAYER_CLASS,
	  .cn_list = 0x1,
	  .range = { .cols = 0x1, .rows = 0x2, .data = range_pde_req_ps_data },
	  .label = SDCA_CTL_REQUESTED_PS_NAME },
	{ .sel = 0x10, .mode = SDCA_ACCESS_MODE_RO, .layers = SDCA_ACCESS_LAYER_CLASS,
	  .cn_list = 0x1, .is_volatile = true, .label = SDCA_CTL_ACTUAL_PS_NAME },
};

/* Entity 0xB (OT 23) Controls */
static struct sdca_entity *entity_ot23_sources[] = { &wsa8855_entities[WSA_FU23] };

static struct sdca_control entity_ot23_controls[] = {
	{ .sel = 0x4, .mode = SDCA_ACCESS_MODE_RW, .layers = 0x4, .cn_list = 0x1,
	  .range = { .cols = 0x7, .rows = 0x2, .data = range_ot23_usage_data },
	  .label = SDCA_CTL_USAGE_NAME },
};

/* Entity 0xC (IT 29) Controls */
static struct sdca_control entity_it29_controls[] = {
	{ .sel = 0x10, .mode = SDCA_ACCESS_MODE_DC, .layers = 0x4, .cn_list = 0x1,
	  .values = ctrl_it29_cluster_vals, .has_fixed = true,
	  .range = { .cols = 0x2, .rows = 0x1, .data = range_it29_cluster_data },
	  .label = SDCA_CTL_CLUSTERINDEX_NAME },
};

/* Entity 0xD (XU 24) Controls */
static struct sdca_entity *entity_xu24_sources[] = { &wsa8855_entities[WSA_IT29] };

static struct sdca_control entity_xu24_controls[] = {
	{ .sel = 0x1, .mode = SDCA_ACCESS_MODE_DC, .layers = 0x4, .cn_list = 0x1,
	  .values = ctrl_xu24_bypass_vals, .has_fixed = true,
	  .label = SDCA_CTL_BYPASS_NAME },
	{ .sel = 0x7, .mode = SDCA_ACCESS_MODE_DC, .layers = 0x4, .cn_list = 0x1,
	  .values = ctrl_xu24_id_vals, .has_fixed = true, .label = SDCA_CTL_XU_ID_NAME },
	{ .sel = 0x8, .mode = SDCA_ACCESS_MODE_DC, .layers = 0x4, .cn_list = 0x1,
	  .values = ctrl_xu24_ver_vals, .has_fixed = true, .label = SDCA_CTL_XU_VERSION_NAME },
};

/* Entity 0xE (CS 24) Controls */
static struct sdca_control entity_cs24_controls[] = {
	{ .sel = 0x2,  .mode = SDCA_ACCESS_MODE_RO, .layers = 0x4, .cn_list = 0x1,
	  .is_volatile = true, .label = SDCA_CTL_CLOCK_VALID_NAME },
	{ .sel = 0x10, .mode = SDCA_ACCESS_MODE_RW, .layers = 0x4, .cn_list = 0x1,
	  .range = { .cols = 0x2, .rows = 0x1, .data = range_cs24_sr_data },
	  .label = SDCA_CTL_SAMPLERATEINDEX_NAME },
};

/* Entity 0xF (OT 24) Controls */
static struct sdca_entity *entity_ot24_sources[] = { &wsa8855_entities[WSA_XU24] };

static struct sdca_control entity_ot24_controls[] = {
	{ .sel = 0x4,  .mode = SDCA_ACCESS_MODE_RW, .layers = 0x4, .cn_list = 0x1,
	  .range = { .cols = 0x7, .rows = 0x3, .data = range_ot24_usage_data },
	  .label = SDCA_CTL_USAGE_NAME },
	{ .sel = 0x8,  .mode = SDCA_ACCESS_MODE_DC, .layers = 0x4, .cn_list = 0x1,
	  .values = ctrl_ot24_latency_vals, .has_fixed = true,
	  .label = SDCA_CTL_LATENCY_NAME },
	{ .sel = 0x11, .mode = SDCA_ACCESS_MODE_DC, .layers = 0x4, .cn_list = 0x1,
	  .values = ctrl_ot24_dp_vals, .has_fixed = true,
	  .range = { .cols = 0x10, .rows = 0x4, .data = range_ot24_dp_data },
	  .label = SDCA_CTL_DATAPORT_SELECTOR_NAME },
};

/* Entities Array (see index map at top of file) */
static struct sdca_entity wsa8855_entities[] = {
	/* [0] E001: IT 21 - PCM render stream input */
	{ .id = 0x1, .label = "IT 21", .type = SDCA_ENTITY_TYPE_IT,
	  .iot = { .type = 0x0101, .is_dataport = true,
		   .clock = &wsa8855_entities[WSA_CS21] },
	  .num_controls = ARRAY_SIZE(entity_it21_controls),
	  .controls = entity_it21_controls },
	/* [1] E002: CS 21 */
	{ .id = 0x2, .label = "CS 21", .type = SDCA_ENTITY_TYPE_CS,
	  .cs = { .type = 0x0 },
	  .num_controls = ARRAY_SIZE(entity_cs21_controls),
	  .controls = entity_cs21_controls },
	/* [2] E003: PPU 21 */
	{ .id = 0x3, .label = "PPU 21", .type = SDCA_ENTITY_TYPE_PPU,
	  .num_controls = ARRAY_SIZE(entity_ppu21_controls),
	  .controls = entity_ppu21_controls,
	  .num_sources = ARRAY_SIZE(entity_ppu21_sources),
	  .sources = entity_ppu21_sources },
	/* [3] E004: FU 21 */
	{ .id = 0x4, .label = "FU 21", .type = SDCA_ENTITY_TYPE_FU,
	  .num_controls = ARRAY_SIZE(entity_fu21_controls),
	  .controls = entity_fu21_controls,
	  .num_sources = ARRAY_SIZE(entity_fu21_sources),
	  .sources = entity_fu21_sources },
	/* [4] E005: MFPU 21 */
	{ .id = 0x5, .label = "MFPU 21", .type = SDCA_ENTITY_TYPE_MFPU,
	  .num_controls = ARRAY_SIZE(entity_mfpu21_controls),
	  .controls = entity_mfpu21_controls,
	  .num_sources = ARRAY_SIZE(entity_mfpu21_sources),
	  .sources = entity_mfpu21_sources },
	/* [5] E006: XU 22 */
	{ .id = 0x6, .label = "XU 22", .type = SDCA_ENTITY_TYPE_XU,
	  .num_controls = ARRAY_SIZE(entity_xu22_controls),
	  .controls = entity_xu22_controls,
	  .num_sources = ARRAY_SIZE(entity_xu22_sources),
	  .sources = entity_xu22_sources },
	/* [6] E007: SAPU 29 */
	{ .id = 0x7, .label = "SAPU 29", .type = SDCA_ENTITY_TYPE_SAPU,
	  .num_sources = ARRAY_SIZE(entity_sapu29_sources),
	  .sources = entity_sapu29_sources },
	/* [7] E008: UDMPU 23 */
	{ .id = 0x8, .label = "UDMPU 23", .type = SDCA_ENTITY_TYPE_UDMPU,
	  .num_controls = ARRAY_SIZE(entity_udmpu23_controls),
	  .controls = entity_udmpu23_controls,
	  .num_sources = ARRAY_SIZE(entity_udmpu23_sources),
	  .sources = entity_udmpu23_sources },
	/* [8] E009: FU 23 */
	{ .id = 0x9, .label = "FU 23", .type = SDCA_ENTITY_TYPE_FU,
	  .num_sources = ARRAY_SIZE(entity_fu23_sources),
	  .sources = entity_fu23_sources },
	/* [9] E00A: PDE 23 - speaker power domain */
	{ .id = 0xA, .label = "PDE 23", .type = SDCA_ENTITY_TYPE_PDE,
	  .pde = { .num_managed = ARRAY_SIZE(entity_pde23_managed),
		   .managed = entity_pde23_managed,
		   .num_max_delay = ARRAY_SIZE(pde23_delays),
		   .max_delay = pde23_delays },
	  .num_controls = ARRAY_SIZE(entity_pde23_controls),
	  .controls = entity_pde23_controls },
	/* [10] E00B: OT 23 - speaker output terminal */
	{ .id = 0xB, .label = "OT 23", .type = SDCA_ENTITY_TYPE_OT,
	  .iot = { .type = 0x380, .num_transducer = 0x2 },
	  .num_controls = ARRAY_SIZE(entity_ot23_controls),
	  .controls = entity_ot23_controls,
	  .num_sources = ARRAY_SIZE(entity_ot23_sources),
	  .sources = entity_ot23_sources },
	/* [11] E00C: IT 29 - IV-sense input */
	{ .id = 0xC, .label = "IT 29", .type = SDCA_ENTITY_TYPE_IT,
	  .iot = { .type = 0x280 },
	  .num_controls = ARRAY_SIZE(entity_it29_controls),
	  .controls = entity_it29_controls },
	/* [12] E00D: XU 24 */
	{ .id = 0xD, .label = "XU 24", .type = SDCA_ENTITY_TYPE_XU,
	  .num_controls = ARRAY_SIZE(entity_xu24_controls),
	  .controls = entity_xu24_controls,
	  .num_sources = ARRAY_SIZE(entity_xu24_sources),
	  .sources = entity_xu24_sources },
	/* [13] E00E: CS 24 */
	{ .id = 0xE, .label = "CS 24", .type = SDCA_ENTITY_TYPE_CS,
	  .cs = { .type = 0x0 },
	  .num_controls = ARRAY_SIZE(entity_cs24_controls),
	  .controls = entity_cs24_controls },
	/* [14] E00F: OT 24 - IV-sense output terminal */
	{ .id = 0xF, .label = "OT 24", .type = SDCA_ENTITY_TYPE_OT,
	  .iot = { .type = 0x189, .is_dataport = true,
		   .clock = &wsa8855_entities[WSA_CS24] },
	  .num_controls = ARRAY_SIZE(entity_ot24_controls),
	  .controls = entity_ot24_controls,
	  .num_sources = ARRAY_SIZE(entity_ot24_sources),
	  .sources = entity_ot24_sources },
	/* Entity 0 (Function) */
	{ .id = 0x0, .label = "entity0",
	  .num_controls = ARRAY_SIZE(entity0_controls), .controls = entity0_controls },
};

/* Clusters */
static struct sdca_channel cl1_channels[] = {
	{ .id = 0x1, .purpose = 0x1, .relationship = 0x2 },
	{ .id = 0x2, .purpose = 0x1, .relationship = 0x3 },
};

static struct sdca_channel cl2_channels[] = {
	{ .id = 0xFF, .purpose = 0x12, .relationship = 0x58 },
	{ .id = 0xFF, .purpose = 0x12, .relationship = 0x59 },
	{ .id = 0xFF, .purpose = 0x9,  .relationship = 0x58 },
	{ .id = 0xFF, .purpose = 0x9,  .relationship = 0x59 },
};

static struct sdca_cluster wsa8855_clusters[] = {
	{ .id = 0x1, .num_channels = ARRAY_SIZE(cl1_channels), .channels = cl1_channels },
	{ .id = 0x2, .num_channels = ARRAY_SIZE(cl2_channels), .channels = cl2_channels },
};

/*
 * Init Table
 *
 * Hardware requires toggling the IT21 enable register (0x40480000) once
 * after the entire init table has been written to latch all configuration.
 */
static struct sdca_init_write wsa8855_init_table[] = {
	{ .addr = 0x40580606, .val = 0x24 }, /* CDC_RX0_RX_PATH_CTL */
	{ .addr = 0x40580626, .val = 0x24 }, /* CDC_RX1_RX_PATH_CTL */
	{ .addr = 0x40580613, .val = 0x1  }, /* RX0_RX_PATH_DSMDEM_CTL */
	{ .addr = 0x40580633, .val = 0x1  }, /* RX1_RX_PATH_DSMDEM_CTL */
	{ .addr = 0x40580640, .val = 0x1  }, /* CDC_COMPANDER0_CTL0 */
	{ .addr = 0x40580660, .val = 0x1  }, /* CDC_COMPANDER1_CTL0 */
	{ .addr = 0x405806A1, .val = 0x14 }, /* CDC_VSENSE0_SPKR_PROT_PATH_CTL */
	{ .addr = 0x405806B1, .val = 0x14 }, /* CDC_VSENSE1_SPKR_PROT_PATH_CTL */
	{ .addr = 0x405806A9, .val = 0x14 }, /* CDC_ISENSE0_SPKR_PROT_PATH_CTL */
	{ .addr = 0x405806B9, .val = 0x14 }, /* CDC_ISENSE1_SPKR_PROT_PATH_CTL */
	{ .addr = 0x4058041C, .val = 0xF  }, /* DIG_CTRL0_CDC_CLK_CTL SYS_CLOCK_EN */
	{ .addr = 0x4058041C, .val = 0xCF }, /* DIG_CTRL0_CDC_CLK_CTL FSM_INTP_CG_DISABLE */
	{ .addr = 0x40580470, .val = 0x2  }, /* DIG_CTRL0_CDC_RXTX_FSCNT_CTL FS_CNT_CLR */
	{ .addr = 0x40580470, .val = 0x0  }, /* DIG_CTRL0_CDC_RXTX_FSCNT_CTL CLR release */
	{ .addr = 0x40580470, .val = 0x1  }, /* DIG_CTRL0_CDC_RXTX_FSCNT_CTL FS_CNT_EN */
	{ .addr = 0x40400008, .val = 0x2  }, /* SMP_AMP_CTRL_STEREO_CMT_GRP_MASK */
	{ .addr = 0x40580602, .val = 0x60 }, /* CDC_RX0_RX_PATH_CFG1 HPF_EN */
	{ .addr = 0x40580622, .val = 0x60 }, /* CDC_RX1_RX_PATH_CFG1 HPF_EN */
	{ .addr = 0x40580108, .val = 0xA5 }, /* SPK_TOP_PWRSTG_CH1_CTRL3 */
	{ .addr = 0x4058010E, .val = 0xA5 }, /* SPK_TOP_PWRSTG_CH2_CTRL3 */
	{ .addr = 0x405800CA, .val = 0x85 }, /* IVSENSE_ADC_MODE_CTL2 */
	{ .addr = 0x405800CB, .val = 0xC  }, /* IVSENSE_ADC_MODE_CTL3 set */
	{ .addr = 0x405800CB, .val = 0xE  }, /* IVSENSE_ADC_MODE_CTL3 commit */
	{ .addr = 0x405800CC, .val = 0xC  }, /* IVSENSE_ADC_REF_CTL */
	{ .addr = 0x405804B4, .val = 0x1  }, /* GAIN_RAMP0_CTL1 ANA_BYPASS */
	{ .addr = 0x405804B7, .val = 0x1  }, /* GAIN_RAMP1_CTL1 ANA_BYPASS */
	{ .addr = 0x40580601, .val = 0x88 }, /* CDC_RX0_RX_PATH_CFG0 DLY_ZN_EN */
	{ .addr = 0x40580601, .val = 0x89 }, /* CDC_RX0_RX_PATH_CFG0 PH_EQ_EN */
	{ .addr = 0x40580621, .val = 0x88 }, /* CDC_RX1_RX_PATH_CFG0 DLY_ZN_EN */
	{ .addr = 0x40580621, .val = 0x89 }, /* CDC_RX1_RX_PATH_CFG0 PH_EQ_EN */
	{ .addr = 0x4058005B, .val = 0x82 }, /* BOOST_STB_CTRL2 */
	{ .addr = 0x4058005C, .val = 0x34 }, /* BOOST_STB_CTRL3 */
	{ .addr = 0x40580065, .val = 0x41 }, /* BOOST_PWRSTAGE_CTRL2 */
	{ .addr = 0x40580067, .val = 0x7F }, /* BOOST_PWRSTAGE_CTRL4 */
	{ .addr = 0x405806CD, .val = 0x50 }, /* CDC_CLSH_V1P8_BP_CTL1 */
	{ .addr = 0x405806CC, .val = 0x6C }, /* CDC_CLSH_V1P8_BP_CTL0 */
	{ .addr = 0x405806C7, .val = 0xD  }, /* CDC_CLSH_CLSH_SIG_DP_CTL0 */
	{ .addr = 0x405806C3, .val = 0x3  }, /* CDC_CLSH_CLSH_V_HD_PA */
	{ .addr = 0x40580423, .val = 0x5  }, /* POWER_FSM_CTL0 */
	{ .addr = 0x4058010B, .val = 0x45 }, /* SPK_TOP_PWRSTG_CH1_TUNE3 */
	{ .addr = 0x40580111, .val = 0x45 }, /* SPK_TOP_PWRSTG_CH2_TUNE3 */
	{ .addr = 0x405806CE, .val = 0x5  }, /* CDC_CLSH_V1P8_BP_CTL2 */
	{ .addr = 0x40580024, .val = 0x35 }, /* BG_TVP_UVLO1_PROG */
	{ .addr = 0x40580025, .val = 0x21 }, /* BG_TVP_UVLO2_PROG */
	{ .addr = 0x4058005E, .val = 0xC7 }, /* BOOST_BYP_CTRL2 */
	{ .addr = 0x4058005F, .val = 0x11 }, /* BOOST_BYP_CTRL3 */
	{ .addr = 0x405800D0, .val = 0x80 }, /* IVSENSE_ADC_CDAC_CAL_CTL2 */
	{ .addr = 0x4058013C, .val = 0x08 }, /* SPK_TOP_SPARE3 */
	{ .addr = 0x40580103, .val = 0x03 }, /* SPK_TOP_COMMON_TUNE1 */
	{ .addr = 0x4058000D, .val = 0x20 }, /* PON_CKSK_CTL_0 */
	{ .addr = 0x4058042B, .val = 0x47 }, /* PA0_FSM_CTL1 */
	{ .addr = 0x40580435, .val = 0x47 }, /* PA1_FSM_CTL1 */
	{ .addr = 0x40580647, .val = 0x34 }, /* CDC_COMPANDER0_CTL7 */
	{ .addr = 0x40580667, .val = 0x34 }, /* CDC_COMPANDER1_CTL7 */
	{ .addr = 0x40580458, .val = 0x79 }, /* VBAT_THRM_FLT_CTL */
	/* Toggle IT21 enable to latch all configuration */
	{ .addr = 0x40480000, .val = 0x20 },
	{ .addr = 0x40480000, .val = 0xff },
};

/* Function Descriptor */
static struct sdca_function_desc wsa8855_desc = {
	.adr  = 0x1,
	.type = SDCA_FUNCTION_TYPE_SIMPLE_AMP,
	.name = SDCA_FUNCTION_TYPE_SIMPLE_AMP_NAME,
};

/* Main Function Data */
static struct sdca_function_data wsa8855_data = {
	.desc           = &wsa8855_desc,
	.num_entities   = ARRAY_SIZE(wsa8855_entities),
	.entities       = wsa8855_entities,
	.num_clusters   = ARRAY_SIZE(wsa8855_clusters),
	.clusters       = wsa8855_clusters,
	.num_init_table = ARRAY_SIZE(wsa8855_init_table),
	.init_table     = wsa8855_init_table,
};

/*
 * FU21 vendor Feature Unit — CH1/CH2 mute.  The codec re-mutes FU21 on
 * every PDE23 PS3->PS0 cycle, so the unmute has to be re-applied from
 * pde_post_pmu with an SCP_COMMIT trigger.
 */
#define WSA8855_FU21_MUTE_CH1_ADDR	0x40404209
#define WSA8855_FU21_MUTE_CH2_ADDR	0x4040420a
#define WSA8855_FU21_MUTE_UNMUTE	0x00

/* PDE23 (entity_id=0xA) manages the OT23 speaker output power domain. */
#define WSA8855_PDE23_ENTITY_ID		0x0A

static const char * const wsa8855_supplies[] = {
	"vdd-1p8",
	"vdd-io",
};

struct wsa8855_priv {
	struct sdca_class_drv class;
};

static int wsa8855_sdw_read_prop(struct sdw_slave *slave)
{
	struct sdw_slave_prop *prop = &slave->prop;
	struct device *dev = &slave->dev;
	struct sdw_dpn_prop *sink;
	int ret;

	ret = sdca_class_read_prop(slave);
	if (ret)
		return ret;

	prop->simple_clk_stop_capable = true;
	prop->paging_support = true;
	prop->clock_reg_supported = false;
	prop->sink_ports = BIT(1);

	sink = devm_kcalloc(dev, 1, sizeof(*sink), GFP_KERNEL);
	if (!sink)
		return -ENOMEM;

	sink[0].num = 1;
	sink[0].type = SDW_DPN_FULL;
	sink[0].simple_ch_prep_sm = true;
	sink[0].ch_prep_timeout = 10;
	sink[0].max_ch = 2;
	sink[0].min_ch = 1;

	prop->sink_dpn_prop = sink;

	if (device_property_read_u32(dev, "qcom,port-mapping",
				     &slave->m_port_map[1]) == 0)
		dev_dbg(dev, "using port mapping %u\n", slave->m_port_map[1]);

	return 0;
}

static int wsa8855_hw_init(struct sdw_slave *slave)
{
	struct device *dev = &slave->dev;
	struct gpio_desc *powerdown;
	int ret;

	ret = devm_regulator_bulk_get_enable(dev,
					     ARRAY_SIZE(wsa8855_supplies),
					     wsa8855_supplies);
	if (ret)
		return dev_err_probe(dev, ret, "failed to enable supplies\n");

	powerdown = devm_gpiod_get_optional(dev, "powerdown", GPIOD_OUT_HIGH);
	if (IS_ERR(powerdown))
		return dev_err_probe(dev, PTR_ERR(powerdown),
				     "failed to get powerdown GPIO\n");

	if (powerdown) {
		gpiod_set_value(powerdown, 0);
		usleep_range(5000, 5010);
	}

	return 0;
}

static int wsa8855_populate_function(struct sdw_slave *slave,
				     struct sdca_function_data *function)
{
	/*
	 * The framework has already set @function->desc to the per-instance
	 * descriptor; leave it alone and only copy the templated payload.
	 * WSA8855 has a single SimpleAmp function.
	 */
	if (function->desc->type != wsa8855_desc.type)
		return -ENOENT;

	function->num_entities   = wsa8855_data.num_entities;
	function->entities       = wsa8855_data.entities;
	function->num_clusters   = wsa8855_data.num_clusters;
	function->clusters       = wsa8855_data.clusters;
	function->num_init_table = wsa8855_data.num_init_table;
	function->init_table     = wsa8855_data.init_table;

	return 0;
}

static int wsa8855_pde_post_pmu(struct sdw_slave *slave,
				struct regmap *regmap,
				unsigned int function_id,
				unsigned int entity_id)
{
	struct wsa8855_priv *priv = dev_get_drvdata(&slave->dev);
	struct sdca_class_drv *core = &priv->class;
	int ret;

	if (entity_id != WSA8855_PDE23_ENTITY_ID)
		return 0;

	if (!core->dev_regmap)
		return 0;

	/*
	 * Unmute FU21 CH1/CH2 and commit via SCP_COMMIT.  Both writes target
	 * the vendor slave regmap (FU21 mute aliases sit outside the SDCA
	 * class regmap's routing) and are grouped by the CommitGroupMask=2
	 * programmed in the init_table (SMP_AMP_CTRL 0x40400008 = 0x2).
	 */
	ret = regmap_write(core->dev_regmap, WSA8855_FU21_MUTE_CH1_ADDR,
			   WSA8855_FU21_MUTE_UNMUTE);
	if (ret)
		dev_err(&slave->dev, "FU21 MUTE_CH1: %d\n", ret);

	ret = regmap_write(core->dev_regmap, WSA8855_FU21_MUTE_CH2_ADDR,
			   WSA8855_FU21_MUTE_UNMUTE);
	if (ret)
		dev_err(&slave->dev, "FU21 MUTE_CH2: %d\n", ret);

	ret = sdw_write_no_pm(slave, SDW_SCP_COMMIT, 0x02);
	if (ret)
		dev_err(&slave->dev, "FU21 SCP_COMMIT: %d\n", ret);

	return 0;
}

static const struct sdca_class_hw_ops wsa8855_hw_ops = {
	.hw_init           = wsa8855_hw_init,
	.populate_function = wsa8855_populate_function,
	.pde_post_pmu      = wsa8855_pde_post_pmu,
};

static int wsa8855_sdw_probe(struct sdw_slave *slave,
			     const struct sdw_device_id *id)
{
	const struct sdca_class_hw_ops *hw_ops = (const void *)id->driver_data;
	struct device *dev = &slave->dev;
	struct sdca_device_data *data = &slave->sdca_data;
	struct wsa8855_priv *priv;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	dev_set_drvdata(dev, priv);

	/*
	 * DT/ARM: no DisCo/ACPI enumeration -- seed the short function
	 * descriptor from the static topology so sdca_dev_register_functions()
	 * creates the SimpleAmp auxdev.  Payload is filled by
	 * wsa8855_populate_function() via hw_ops.
	 */
	if (!data->num_functions) {
		data->function[0].type = wsa8855_desc.type;
		data->function[0].adr  = wsa8855_desc.adr;
		data->function[0].name = wsa8855_desc.name;
		data->num_functions    = 1;
	}

	return sdca_class_probe(slave, &priv->class, hw_ops);
}

static void wsa8855_sdw_remove(struct sdw_slave *slave)
{
	struct wsa8855_priv *priv = dev_get_drvdata(&slave->dev);

	sdca_class_remove(&priv->class);
}

static int wsa8855_runtime_suspend(struct device *dev)
{
	struct wsa8855_priv *priv = dev_get_drvdata(dev);

	return sdca_class_runtime_suspend(&priv->class);
}

static int wsa8855_runtime_resume(struct device *dev)
{
	struct wsa8855_priv *priv = dev_get_drvdata(dev);

	return sdca_class_runtime_resume(&priv->class);
}

static int wsa8855_system_suspend(struct device *dev)
{
	struct wsa8855_priv *priv = dev_get_drvdata(dev);

	return sdca_class_system_suspend(&priv->class);
}

static int wsa8855_system_resume(struct device *dev)
{
	struct wsa8855_priv *priv = dev_get_drvdata(dev);

	return sdca_class_system_resume(&priv->class);
}

static const struct dev_pm_ops wsa8855_pm_ops = {
	SYSTEM_SLEEP_PM_OPS(wsa8855_system_suspend, wsa8855_system_resume)
	RUNTIME_PM_OPS(wsa8855_runtime_suspend, wsa8855_runtime_resume, NULL)
};

static const struct sdw_slave_ops wsa8855_sdw_ops = {
	.read_prop	= wsa8855_sdw_read_prop,
};

static const struct sdw_device_id wsa8855_sdw_id[] = {
	SDW_SLAVE_ENTRY_EXT(0x0217, 0x0205, 0x2, 0x1, &wsa8855_hw_ops),
	{}
};
MODULE_DEVICE_TABLE(sdw, wsa8855_sdw_id);

static struct sdw_driver wsa8855_sdw_driver = {
	.driver = {
		.name	= "wsa8855",
		.pm	= pm_ptr(&wsa8855_pm_ops),
	},
	.probe		= wsa8855_sdw_probe,
	.remove		= wsa8855_sdw_remove,
	.id_table	= wsa8855_sdw_id,
	.ops		= &wsa8855_sdw_ops,
};
module_sdw_driver(wsa8855_sdw_driver);

MODULE_DESCRIPTION("Qualcomm WSA8855 SDCA SoundWire Class-D speaker amplifier");
MODULE_AUTHOR("Qualcomm Technologies, Inc.");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_IMPORT_NS("SND_SOC_SDCA_CLASS");
