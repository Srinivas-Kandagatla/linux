/* SPDX-License-Identifier: GPL-2.0 */
/*
 * The MIPI SDCA specification is available for public downloads at
 * https://www.mipi.org/mipi-sdca-v1-0-download
 *
 * Copyright (C) 2025 Cirrus Logic, Inc. and
 *                    Cirrus Logic International Semiconductor Ltd.
 */

#ifndef __SDCA_CLASS_H__
#define __SDCA_CLASS_H__

#include <linux/completion.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>

struct device;
struct dev_pm_ops;
struct regmap;
struct sdw_slave;
struct sdca_function_data;

/**
 * struct sdca_class_hw_ops - optional device-specific hardware callbacks
 * @hw_init: called during probe to enable supplies, toggle reset GPIO, etc.
 * @populate_function: called from the SDCA class function driver when no
 *             DisCo/ACPI firmware node is available (e.g. DT/ARM platforms),
 *             as a substitute for sdca_parse_function().  The callback must
 *             fill in @function (entities, clusters, init_table, delays, ...)
 *             from its own static tables selected by @function->desc->type
 *             and must leave @function->desc alone -- the framework owns the
 *             per-instance descriptor.  Return 0 on success or a negative
 *             errno if no matching template is known.  May be NULL.
 * @pde_pre_pmu: called before DAPM writes REQUESTED_PS=PS0; use to
 *             prepare device state that must be valid before the PDE
 *             sequencer runs (e.g. IT_USAGE); may be NULL
 * @pde_post_pmu: called after DAPM writes REQUESTED_PS=PS0 and before
 *             ACTUAL_PS polling begins; use to commit pending register
 *             writes (e.g. FUNCTION_ACTION) on devices that require an
 *             explicit commit trigger for PDE power-up; may be NULL
 *
 * Codec-specific SoundWire drivers pass a pointer to this struct to
 * sdca_class_probe() from their sdw_driver.probe.  Codec-specific
 * SoundWire slave property overrides live directly in the codec's
 * sdw_slave_ops.read_prop, which should call sdca_class_read_prop() to
 * fill the SDCA-common bits first.
 */
struct sdca_class_hw_ops {
	int  (*hw_init)(struct sdw_slave *slave);
	int  (*populate_function)(struct sdw_slave *slave,
				  struct sdca_function_data *function);
	int  (*pde_pre_pmu)(struct sdw_slave *slave, struct regmap *regmap,
			    unsigned int function_id, unsigned int entity_id);
	int  (*pde_post_pmu)(struct sdw_slave *slave, struct regmap *regmap,
			     unsigned int function_id, unsigned int entity_id);
};

struct sdca_class_drv {
	struct device *dev;
	struct regmap *dev_regmap;
	struct sdw_slave *sdw;

	struct sdca_interrupt_info *irq_info;

	const struct sdca_class_hw_ops *hw_ops;

	struct mutex regmap_lock;
	/* Serialise function initialisations */
	struct mutex init_lock;
	struct work_struct boot_work;
};

/* Library helpers used by codec-specific SDCA SoundWire drivers. */
int sdca_class_read_prop(struct sdw_slave *sdw);
int sdca_class_probe(struct sdw_slave *sdw,
		     struct sdca_class_drv *drv,
		     const struct sdca_class_hw_ops *hw_ops);
void sdca_class_remove(struct sdca_class_drv *drv);

/*
 * PM helpers.  Codec drivers embed sdca_class_drv in their own priv,
 * own dev_set_drvdata(), and compose these into their own dev_pm_ops:
 *
 *	static int wcd_runtime_suspend(struct device *dev) {
 *		struct wcd_priv *priv = dev_get_drvdata(dev);
 *		return sdca_class_runtime_suspend(&priv->class);
 *	}
 *
 * The built-in class_sdw_driver in sdca_class.c uses sdca_class_pm_ops
 * directly because it stashes the sdca_class_drv in drvdata itself.
 */
int sdca_class_runtime_suspend(struct sdca_class_drv *drv);
int sdca_class_runtime_resume(struct sdca_class_drv *drv);
int sdca_class_system_suspend(struct sdca_class_drv *drv);
int sdca_class_system_resume(struct sdca_class_drv *drv);
extern const struct dev_pm_ops sdca_class_pm_ops;

#endif /* __SDCA_CLASS_H__ */
