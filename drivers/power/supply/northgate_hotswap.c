// SPDX-License-Identifier: GPL-2.0
/*
 * northgate-hotswap.c - skeleton power-supply driver with threaded GPIO IRQ, I2C 0x18, PM, and sysfs bit-level attributes
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/regmap.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/power_supply.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/property.h>

/* --- Registers --- */
#define REG_PATTERN_ID 0xE6
#define NORTHGATE_PATTERN_ID 0x05
#define REG_LATCHED_FAULTS 0xF1
#define REG_REALTIME_STATUS 0xF2
#define REG_I2C_VIRTUAL_IN 0xF4
#define REG_FET_STATUS 0xF5
#define REG_REALTIME_STATUS2 0xF7

/* --- Bit macros --- */
/* LATCHED_FAULTS (RO) */
#define LT_HW_BATT_CONTROL BIT(3)
#define LT_CHARGER_INT_LATCHED BIT(4)
#define LT_USE_INT_BATT BIT(5)
#define LT_PACK_STATE_CHANGED BIT(6)
#define LT_INTERNAL_UVLO BIT(7)

/* REALTIME_STATUS (RO) */
#define RT_PACK_CONNECTED BIT(2)
#define RT_CROSS_CHG_VINT_FROM_VREM BIT(6)
#define RT_CROSS_CHG_VREM_FROM_VINT BIT(7)

/* I2C_VIRTUAL_IN (RW) */
#define VI_USE_INTERNAL_BATTERY BIT(0)
#define VI_CHARGE_FROM_INT BIT(1)
#define VI_ENABLE_CROSS_CHARGER BIT(2)
#define VI_CLEAR_BATT_INTERRUPT BIT(4)

/* FET_STATUS (RO) */
#define FET_INT_CHG BIT(0)
#define FET_INT_DIS BIT(1)
#define FET_REM_CHG BIT(2)
#define FET_REM_DSG BIT(3)
#define FET_INT_ACT BIT(5)
#define FET_REM_ACT BIT(6)

/* REALTIME_STATUS2 (RO) */
#define RT2_REMOVABLE_V_OK BIT(1)
#define RT2_INTERNAL_V_OK BIT(2)

#define REMOVABLE_PACK_SLAVE_ADDR 0x76
#define REMOVABLE_PACK_NVM_ADDR 0x4B
#define REMOVABLE_PACK_RSENSE 25

#define CROSS_CHARGER_SLAVE_ADDR 0x69

#define REMOVABLE_PACK_ATTACH_DELAY_MS 500

#define REMOVABLE_PACK_ATTACH_RETRY_MS 1000
#define REMOVABLE_PACK_ATTACH_RETRIES 10

// The Silego, by default, autonomously selects which battery is connected to Vsys. 
// When the removable battery voltage crosses valid (6.25 V) / invalid (5.85 V) thresholds it 
// automatically connects or disconnects the battery via hardware and raises no interrupt
// The driver therefore must poll the FET and voltage status so userspace can see the selection change.
//
// The poll runs whether or not the pack is attached, because it is also the safety net for a missed
// pack interrupt (e.g. the aDSP serviced the Silego during suspend, or the edge was lost). If the
// Silego's live PACK_CONNECTED bit disagrees with the driver's state for PACK_RESYNC_POLLS polls in
// a row, the poll services the event the same way the interrupt handler would.
#define STATUS_POLL_MS 1000
#define PACK_RESYNC_POLLS 2
#define FET_SELECTED_BATTERY_MASK (FET_INT_ACT | FET_REM_ACT)

/* --- Driver state --- */
struct northgate_hotswap_psy {
	struct device *dev;
	struct regmap *regmap;
	struct i2c_client *client;
	struct power_supply *psy;

	// Cached Last Interrupt State
	struct mutex fault_mutex;
	u8 pack_removed;
	u8 pack_installed;
	u8 charger_fault;
	u8 vint_not_valid;

	// Removable Pack Handling
	int removable_pack_connected;
	struct i2c_client *battery_client;  /* max17320, dynamically created */
	struct delayed_work hotswap_work;
	unsigned int attach_retries;
	unsigned int pack_mismatch_polls;

	// Cross Charger
	struct i2c_client *charger_client;  /* mp2652, dynamically created */
	bool cross_charger_enabled;
	u8 cross_charge_selected_battery;

	// Selected-battery / voltage status poll (see STATUS_POLL_MS)
	struct delayed_work status_poll_work;
	u8 last_fet_status;
	u8 last_realtime_status;
	u8 last_realtime_status2;
};

static int northgate_set_cross_charger(struct northgate_hotswap_psy *northgate, bool enable);
static void northgate_hotswap_notify(struct northgate_hotswap_psy *northgate);
static int northgate_hotswap_cache_and_clear_faults(struct northgate_hotswap_psy *northgate, int *pack_connected);
static bool northgate_hotswap_set_pack_state(struct northgate_hotswap_psy *northgate, int pack_connected);

/* --- Sysfs helpers with DEVICE_ATTR integrated --- */
#define NORTHGATE_HOTSWAP_BIT_SHOW(name, reg, bit_macro)                                         \
	static ssize_t name##_show(struct device *dev, struct device_attribute *attr, char *buf) \
	{                                                                                        \
		struct power_supply *psy = dev_get_drvdata(dev);                                 \
		struct northgate_hotswap_psy *northgate = power_supply_get_drvdata(psy);         \
		unsigned int val;                                                                \
		int ret;                                                                         \
                                                                                                 \
		ret = regmap_read(northgate->regmap, reg, &val);                                 \
		if (ret < 0)                                                                     \
			return ret;                                                              \
		return sysfs_emit(buf, "%d\n", !!(val & (bit_macro)));                           \
	}

#define NORTHGATE_HOTSWAP_BIT_STORE(name, reg, bit_macro)                                                             \
	static ssize_t name##_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) \
	{                                                                                                             \
		struct power_supply *psy = dev_get_drvdata(dev);                                                      \
		struct northgate_hotswap_psy *northgate = power_supply_get_drvdata(psy);                              \
		unsigned long v;                                                                                      \
		int ret;                                                                                              \
                                                                                                                      \
		if (kstrtoul(buf, 0, &v))                                                                             \
			return -EINVAL;                                                                               \
                                                                                                                      \
		ret = regmap_update_bits(northgate->regmap, reg, bit_macro, (v ? bit_macro : 0));                     \
		if (ret < 0)                                                                                          \
			return ret;                                                                                   \
		return count;                                                                                         \
	}

/* --- Define attributes --- */

/* F1 RO bits */
NORTHGATE_HOTSWAP_BIT_SHOW(realtime_hw_batt_control, REG_LATCHED_FAULTS, LT_HW_BATT_CONTROL);
static DEVICE_ATTR_RO(realtime_hw_batt_control);
NORTHGATE_HOTSWAP_BIT_SHOW(realtime_use_int_batt, REG_LATCHED_FAULTS, LT_USE_INT_BATT);
static DEVICE_ATTR_RO(realtime_use_int_batt);

/* F2 RO bits */
NORTHGATE_HOTSWAP_BIT_SHOW(realtime_pack_connected, REG_REALTIME_STATUS, RT_PACK_CONNECTED);
static DEVICE_ATTR_RO(realtime_pack_connected);
NORTHGATE_HOTSWAP_BIT_SHOW(realtime_cross_chg_vint_from_vrem, REG_REALTIME_STATUS, RT_CROSS_CHG_VINT_FROM_VREM);
static DEVICE_ATTR_RO(realtime_cross_chg_vint_from_vrem);
NORTHGATE_HOTSWAP_BIT_SHOW(realtime_cross_chg_vrem_from_vint, REG_REALTIME_STATUS, RT_CROSS_CHG_VREM_FROM_VINT);
static DEVICE_ATTR_RO(realtime_cross_chg_vrem_from_vint);

/* F4 RW bits */
NORTHGATE_HOTSWAP_BIT_SHOW(use_internal_battery, REG_I2C_VIRTUAL_IN, VI_USE_INTERNAL_BATTERY);
NORTHGATE_HOTSWAP_BIT_STORE(use_internal_battery, REG_I2C_VIRTUAL_IN, VI_USE_INTERNAL_BATTERY);
static DEVICE_ATTR_RW(use_internal_battery);

NORTHGATE_HOTSWAP_BIT_SHOW(charge_from_int, REG_I2C_VIRTUAL_IN, VI_CHARGE_FROM_INT);
NORTHGATE_HOTSWAP_BIT_STORE(charge_from_int, REG_I2C_VIRTUAL_IN, VI_CHARGE_FROM_INT);
static DEVICE_ATTR_RW(charge_from_int);

NORTHGATE_HOTSWAP_BIT_SHOW(enable_cross_charger, REG_I2C_VIRTUAL_IN, VI_ENABLE_CROSS_CHARGER);
static ssize_t enable_cross_charger_store(struct device *dev, struct device_attribute *attr,
					  const char *buf, size_t count)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct northgate_hotswap_psy *northgate = power_supply_get_drvdata(psy);
	unsigned long v;
	int ret;

	if (kstrtoul(buf, 0, &v))
		return -EINVAL;

	mutex_lock(&northgate->fault_mutex);
	if (v) {
		unsigned int status = 0, fet = 0;

		ret = regmap_read(northgate->regmap, REG_REALTIME_STATUS, &status);
		if (ret < 0)
			goto out_unlock;
		if (!(status & RT_PACK_CONNECTED)) {
			ret = -ENODEV;
			goto out_unlock;
		}

		// Exactly one battery must be connected to Vsys 
		ret = regmap_read(northgate->regmap, REG_FET_STATUS, &fet);
		if (ret < 0)
			goto out_unlock;
		if (!(fet & FET_INT_ACT) == !(fet & FET_REM_ACT)) {
			ret = -EBUSY;
			goto out_unlock;
		}
	}

	ret = northgate_set_cross_charger(northgate, !!v);
out_unlock:
	mutex_unlock(&northgate->fault_mutex);
	return ret < 0 ? ret : count;
}
static DEVICE_ATTR_RW(enable_cross_charger);

/* F5 RO bits */
NORTHGATE_HOTSWAP_BIT_SHOW(fet_internal_charge, REG_FET_STATUS, FET_INT_CHG);
static DEVICE_ATTR_RO(fet_internal_charge);

NORTHGATE_HOTSWAP_BIT_SHOW(fet_internal_discharge, REG_FET_STATUS, FET_INT_DIS);
static DEVICE_ATTR_RO(fet_internal_discharge);

NORTHGATE_HOTSWAP_BIT_SHOW(fet_removable_charge, REG_FET_STATUS, FET_REM_CHG);
static DEVICE_ATTR_RO(fet_removable_charge);

NORTHGATE_HOTSWAP_BIT_SHOW(fet_removable_discharge, REG_FET_STATUS, FET_REM_DSG);
static DEVICE_ATTR_RO(fet_removable_discharge);

NORTHGATE_HOTSWAP_BIT_SHOW(internal_active, REG_FET_STATUS, FET_INT_ACT);
static DEVICE_ATTR_RO(internal_active);

NORTHGATE_HOTSWAP_BIT_SHOW(removable_active, REG_FET_STATUS, FET_REM_ACT);
static DEVICE_ATTR_RO(removable_active);

/* F7 RO bits */
NORTHGATE_HOTSWAP_BIT_SHOW(removable_voltage_ok, REG_REALTIME_STATUS2, RT2_REMOVABLE_V_OK);
static DEVICE_ATTR_RO(removable_voltage_ok);
NORTHGATE_HOTSWAP_BIT_SHOW(internal_voltage_ok, REG_REALTIME_STATUS2, RT2_INTERNAL_V_OK);
static DEVICE_ATTR_RO(internal_voltage_ok);

static ssize_t interrupt_clear_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct northgate_hotswap_psy *northgate = power_supply_get_drvdata(psy);
	int ret;

	ret = mutex_lock_interruptible(&northgate->fault_mutex);
	if (ret)
		return ret;

	northgate->pack_installed = 0;
	northgate->pack_removed = 0;
	northgate->charger_fault = 0;
	northgate->vint_not_valid = 0;
	mutex_unlock(&northgate->fault_mutex);

	northgate_hotswap_notify(northgate);

	return count;
}
static DEVICE_ATTR_WO(interrupt_clear);

static ssize_t interrupt_pack_installed_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct northgate_hotswap_psy *northgate = power_supply_get_drvdata(psy);

	return sysfs_emit(buf, "%d\n", northgate->pack_installed);
}
static DEVICE_ATTR_RO(interrupt_pack_installed);

static ssize_t interrupt_pack_removed_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct northgate_hotswap_psy *northgate = power_supply_get_drvdata(psy);

	return sysfs_emit(buf, "%d\n", northgate->pack_removed);
}
static DEVICE_ATTR_RO(interrupt_pack_removed);

static ssize_t interrupt_charger_fault_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct northgate_hotswap_psy *northgate = power_supply_get_drvdata(psy);

	return sysfs_emit(buf, "%d\n", northgate->charger_fault);
}
static DEVICE_ATTR_RO(interrupt_charger_fault);

static ssize_t interrupt_vint_not_valid_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct northgate_hotswap_psy *northgate = power_supply_get_drvdata(psy);

	return sysfs_emit(buf, "%d\n", northgate->vint_not_valid);
}
static DEVICE_ATTR_RO(interrupt_vint_not_valid);

// clang-format off
static struct attribute *northgate_hotswap_attrs[] = {
	&dev_attr_realtime_hw_batt_control.attr,
	&dev_attr_realtime_use_int_batt.attr,
	&dev_attr_realtime_pack_connected.attr,
	&dev_attr_realtime_cross_chg_vint_from_vrem.attr,
	&dev_attr_realtime_cross_chg_vrem_from_vint.attr,
	&dev_attr_use_internal_battery.attr,
	&dev_attr_charge_from_int.attr,
	&dev_attr_enable_cross_charger.attr,
	&dev_attr_fet_internal_charge.attr,
	&dev_attr_fet_internal_discharge.attr,
	&dev_attr_fet_removable_charge.attr,
	&dev_attr_fet_removable_discharge.attr,
	&dev_attr_internal_active.attr,
	&dev_attr_removable_active.attr,
	&dev_attr_removable_voltage_ok.attr,
	&dev_attr_internal_voltage_ok.attr,
	&dev_attr_interrupt_clear.attr,
	&dev_attr_interrupt_pack_installed.attr,
	&dev_attr_interrupt_pack_removed.attr,
	&dev_attr_interrupt_charger_fault.attr,
	&dev_attr_interrupt_vint_not_valid.attr,
	NULL };
// clang-format on
ATTRIBUTE_GROUPS(northgate_hotswap);

static int northgate_hotswap_get_property(struct power_supply *psy, enum power_supply_property psp,
					  union power_supply_propval *val)
{
	struct northgate_hotswap_psy *northgate = power_supply_get_drvdata(psy);
	unsigned int reg_val;
	int ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		ret = regmap_read(northgate->regmap, REG_REALTIME_STATUS, &reg_val);
		if (ret < 0)
			return ret;
		val->intval = (reg_val & (RT_CROSS_CHG_VINT_FROM_VREM | RT_CROSS_CHG_VREM_FROM_VINT)) ?
			POWER_SUPPLY_STATUS_CHARGING : POWER_SUPPLY_STATUS_NOT_CHARGING;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		if (!northgate->removable_pack_connected)
			val->intval = POWER_SUPPLY_HEALTH_NO_BATTERY;
		else if (northgate->vint_not_valid)
			val->intval = POWER_SUPPLY_HEALTH_UNDERVOLTAGE;
		else if (northgate->charger_fault)
			val->intval = POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
		else
			val->intval = POWER_SUPPLY_HEALTH_GOOD;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = northgate->removable_pack_connected;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = 1;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static enum power_supply_property northgate_hotswap_psy_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
};

static const struct power_supply_desc northgate_hotswap_psy_desc = {
	.name = "northgate-hotswap",
	.type = POWER_SUPPLY_TYPE_UNKNOWN,
	.properties = northgate_hotswap_psy_props,
	.num_properties = ARRAY_SIZE(northgate_hotswap_psy_props),
	.get_property = northgate_hotswap_get_property,
	.no_thermal = true,
};

static int northgate_hotswap_removable_pack_connected(struct northgate_hotswap_psy *northgate)
{
	unsigned int val = 0;
	int ret;

	ret = regmap_read(northgate->regmap, REG_REALTIME_STATUS, &val);
	if (ret < 0)
		return ret;

	return !!(val & RT_PACK_CONNECTED);
}

static const struct property_entry mp2652_swnode_props[] = {
	PROPERTY_ENTRY_U32("constant-charge-current-max-microamp", 2700000),
	PROPERTY_ENTRY_U32("constant-charge-voltage-max-microvolt", 8900000),
	{}
};

static const struct software_node mp2652_swnode = {
	.name = "mp2652-charger",
	.properties = mp2652_swnode_props,
};

static int northgate_set_cross_charger(struct northgate_hotswap_psy *northgate, bool enable)
{
	int ret;

	if (enable) {
		unsigned int fet = 0;

		// Record which battery is selected. If the Silego selects a different battery while
		// cross charging is enabled, northgate_hotswap_check_status() disables the cross charger
		// (EN_CROSS_CHARGER cleared, mp2652 device removed) to prevent two chargers hitting the same
		// battery
		ret = regmap_read(northgate->regmap, REG_FET_STATUS, &fet);
		if (ret < 0)
			return ret;
		northgate->cross_charge_selected_battery = fet & FET_SELECTED_BATTERY_MASK;


		// The MP2652 is only powered while the path is enabled, so the bit must be set before its
		// I2C device can be probed. If creating the device fails, undo the enable
		ret = regmap_update_bits(northgate->regmap, REG_I2C_VIRTUAL_IN,
					 VI_ENABLE_CROSS_CHARGER, VI_ENABLE_CROSS_CHARGER);
		if (ret < 0)
			return ret;

		if (!northgate->charger_client) {
			struct i2c_board_info info = {
				I2C_BOARD_INFO("mp2652", CROSS_CHARGER_SLAVE_ADDR),
				.swnode = &mp2652_swnode,
			};
			struct i2c_client *charger =
				i2c_new_client_device(northgate->client->adapter, &info);

			if (IS_ERR(charger)) {
				int err;

				ret = PTR_ERR(charger);
				dev_err(northgate->dev, "northgate-hotswap failed to create mp2652 client: %d\n", ret);
				err = regmap_update_bits(northgate->regmap, REG_I2C_VIRTUAL_IN,
							 VI_ENABLE_CROSS_CHARGER, 0);
				if (err < 0)
					dev_err(northgate->dev, "failed to update EN_CROSS_CHARGER: %d\n", err);
				return ret;
			}
			northgate->charger_client = charger;

			dev_info(&northgate->client->dev, "mp2652 charger attached\n");
		}
		northgate->cross_charger_enabled = true;
	} else {
		if (northgate->charger_client) {
			i2c_unregister_device(northgate->charger_client);
			northgate->charger_client = NULL;

			dev_info(&northgate->client->dev, "mp2652 charger removed\n");
		}
		// Clear EN_CROSS_CHARGER first, which opens the cross charger's input and output MUX FETs, then 
		// return CHG_VREM_FROM_VINT to its power-on default of 0 (charge internal from removable). 
		// Restoring the default here means the next enable always starts from a known state.
		ret = regmap_update_bits(northgate->regmap, REG_I2C_VIRTUAL_IN,
					 VI_ENABLE_CROSS_CHARGER, 0);
		if (ret < 0) {
			// The path may still be enabled in hardware. Leave cross_charger_enabled set so a
			// later disable (e.g. the next status poll's battery-change check) retries the clear.
			dev_err(northgate->dev, "failed to clear EN_CROSS_CHARGER: %d\n", ret);
			return ret;
		}
		northgate->cross_charger_enabled = false;

		ret = regmap_update_bits(northgate->regmap, REG_I2C_VIRTUAL_IN,
					 VI_CHARGE_FROM_INT, 0);
		if (ret < 0) {
			dev_err(northgate->dev, "failed to update CHG_VREM_FROM_VINT: %d\n", ret);
			return ret;
		}
	}

	return 0;
}

static void northgate_hotswap_notify(struct northgate_hotswap_psy *northgate)
{
	static const char * const attrs[] = {
		"realtime_hw_batt_control", "realtime_use_int_batt", "realtime_pack_connected",
		"realtime_cross_chg_vint_from_vrem", "realtime_cross_chg_vrem_from_vint",
		"use_internal_battery", "charge_from_int", "enable_cross_charger",
		"internal_active", "removable_active",
		"removable_voltage_ok", "internal_voltage_ok",
		"interrupt_pack_installed", "interrupt_pack_removed",
		"interrupt_charger_fault", "interrupt_vint_not_valid",
	};
	struct kobject *kobj;
	size_t i;

	if (!northgate->psy)
		return;

	kobj = &northgate->psy->dev.kobj;
	for (i = 0; i < ARRAY_SIZE(attrs); i++)
		sysfs_notify(kobj, NULL, attrs[i]);
}

// Read the selected battery and voltage status.  Stop cross charging if the selected
// battery changed, and report.
// Returns 1 if any status changed, 0 if not, or a negative errno if read failed
static int northgate_hotswap_check_status(struct northgate_hotswap_psy *northgate)
{
	unsigned int fet = 0, rt = 0, rt2 = 0;
	bool changed;
	int ret;

	ret = regmap_read(northgate->regmap, REG_FET_STATUS, &fet);
	if (!ret)
		ret = regmap_read(northgate->regmap, REG_REALTIME_STATUS, &rt);
	if (!ret)
		ret = regmap_read(northgate->regmap, REG_REALTIME_STATUS2, &rt2);
	if (ret < 0) {
		dev_warn_ratelimited(northgate->dev, "status read failed: %d\n", ret);
		return ret;
	}

	if (northgate->cross_charger_enabled &&
	    ((fet & FET_SELECTED_BATTERY_MASK) != northgate->cross_charge_selected_battery)) {
		/*
		 * The Silego selected a different battery under an active cross-charge session. The
		 * selected battery is the one USB charges, so leaving the cross charger on risks both
		 * chargers on the same battery. Userspace re-evaluates and re-enables if still appropriate.
		 * If the disable fails, cross_charger_enabled stays set and the next check retries.
		 */
		dev_warn(northgate->dev, "selected battery changed (0x%02x -> 0x%02lx) while cross charging, disabling cross charger\n",
			 northgate->cross_charge_selected_battery, fet & FET_SELECTED_BATTERY_MASK);
		northgate_set_cross_charger(northgate, false);

		ret = regmap_read(northgate->regmap, REG_REALTIME_STATUS, &rt);
		if (ret < 0) {
			dev_warn_ratelimited(northgate->dev, "status read failed: %d\n", ret);
			return ret;
		}
	}

	changed = (fet != northgate->last_fet_status) ||
		  (rt != northgate->last_realtime_status) ||
		  (rt2 != northgate->last_realtime_status2);
	northgate->last_fet_status = fet;
	northgate->last_realtime_status = rt;
	northgate->last_realtime_status2 = rt2;

	return changed;
}

static void northgate_hotswap_status_poll(struct work_struct *work)
{
	struct northgate_hotswap_psy *northgate =
		container_of(to_delayed_work(work), struct northgate_hotswap_psy, status_poll_work);
	bool changed = false;
	int pack_connected;
	int ret;

	mutex_lock(&northgate->fault_mutex);

	ret = northgate_hotswap_check_status(northgate);
	if (ret > 0)
		changed = true;

	if (ret >= 0) {
		pack_connected = !!(northgate->last_realtime_status & RT_PACK_CONNECTED);

		if (pack_connected == northgate->removable_pack_connected) {
			northgate->pack_mismatch_polls = 0;
		} else if (++northgate->pack_mismatch_polls >= PACK_RESYNC_POLLS) {
			// The interrupt handler normally gets here first. Reaching this point means a pack
			// event was never delivered, so service it here. Clearing the latch also releases
			// the alert line so the edge-triggered interrupt can fire again.
			dev_warn(northgate->dev, "pack %s without an interrupt, resyncing\n",
				 pack_connected ? "connected" : "disconnected");

			northgate_hotswap_cache_and_clear_faults(northgate, &pack_connected);
			northgate_hotswap_set_pack_state(northgate, pack_connected);
			northgate_hotswap_check_status(northgate);
			northgate->pack_mismatch_polls = 0;
			changed = true;
		}
	}

	mutex_unlock(&northgate->fault_mutex);

	if (changed) {
		northgate_hotswap_notify(northgate);
		power_supply_changed(northgate->psy);
	}

	schedule_delayed_work(&northgate->status_poll_work, msecs_to_jiffies(STATUS_POLL_MS));
}

static const struct property_entry max1720x_swnode_props[] = { PROPERTY_ENTRY_U8("nvm_reg", REMOVABLE_PACK_NVM_ADDR),
							       PROPERTY_ENTRY_U32("rsense", REMOVABLE_PACK_RSENSE),
							       {} };

static const struct software_node max1720x_swnode = {
	.name = "max17320-battery",
	.properties = max1720x_swnode_props,
};

static void northgate_hotswap_work(struct work_struct *work)
{
	struct northgate_hotswap_psy *northgate = container_of(to_delayed_work(work), struct northgate_hotswap_psy, hotswap_work);
	bool retry = false;

	mutex_lock(&northgate->fault_mutex);

	if (northgate->removable_pack_connected && !northgate->battery_client) {
		/* Attach Pack */
		struct i2c_board_info info = {
			I2C_BOARD_INFO("max17320", REMOVABLE_PACK_SLAVE_ADDR),
			.swnode = &max1720x_swnode,
		};
		struct i2c_client *battery = i2c_new_client_device(northgate->client->adapter, &info);

		if (IS_ERR(battery)) {
			if (northgate->attach_retries < REMOVABLE_PACK_ATTACH_RETRIES) {
				northgate->attach_retries++;
				retry = true;
				dev_warn(&northgate->client->dev, "northgate-hotswap failed to create max17320 client (%ld), retry %u/%u\n",
					 PTR_ERR(battery), northgate->attach_retries, REMOVABLE_PACK_ATTACH_RETRIES);
			} else {
				dev_err(&northgate->client->dev, "northgate-hotswap failed to create max17320 client (%ld), giving up until the pack is reinserted\n",
					PTR_ERR(battery));
			}
		} else {
			northgate->battery_client = battery;
			northgate->attach_retries = 0;
			dev_info(&northgate->client->dev, "northgate-hotswap attached\n");
		}

	} else if (!northgate->removable_pack_connected && northgate->battery_client) {
		/* Remove Pack */
		dev_info(&northgate->client->dev, "northgate-hotswap removed\n");

		i2c_unregister_device(northgate->battery_client);
		northgate->battery_client = NULL;
		northgate->attach_retries = 0;
	}

	mutex_unlock(&northgate->fault_mutex);

	if (retry)
		schedule_delayed_work(&northgate->hotswap_work, msecs_to_jiffies(REMOVABLE_PACK_ATTACH_RETRY_MS));
}

static int northgate_hotswap_cache_and_clear_faults(struct northgate_hotswap_psy *northgate, int *pack_connected)
{
	// NOTE Mutex Lock is held by caller
	unsigned int val;
	int ret = 0, err;

	// Check if Pack is Connected
	*pack_connected = northgate_hotswap_removable_pack_connected(northgate);
	if (*pack_connected < 0) {
		dev_err(northgate->dev, "pack status read failed: %d\n", *pack_connected);
		ret = *pack_connected;
	}

	// Read Latched Faults
	err = regmap_read(northgate->regmap, REG_LATCHED_FAULTS, &val);
	if (err < 0) {
		dev_err(northgate->dev, "latched faults read failed: %d\n", err);
		if (!ret)
			ret = err;
	} else {
		// Cached Last Interrupt State
		if ((val & LT_PACK_STATE_CHANGED) && *pack_connected >= 0) {
			if (*pack_connected) {
				// Pack is now connected
				northgate->pack_installed |= 1;
			} else {
				// Pack isn't connected
				northgate->pack_removed |= 1;
			}
		}
		northgate->charger_fault |= !!(val & LT_CHARGER_INT_LATCHED);
		northgate->vint_not_valid |= !!(val & LT_INTERNAL_UVLO);
	}

	/*
	 * Silego requires a double-write to clear the interrupt latch.
	 * nCLEAR INTERRUPT I2C has start-up state "1", so we toggle
	 * the bit twice: 1→0 then 0→1.
	 */
	err = regmap_update_bits(northgate->regmap, REG_I2C_VIRTUAL_IN, VI_CLEAR_BATT_INTERRUPT, 0);
	if (!err)
		err = regmap_update_bits(northgate->regmap, REG_I2C_VIRTUAL_IN, VI_CLEAR_BATT_INTERRUPT, VI_CLEAR_BATT_INTERRUPT);
	if (err < 0) {
		dev_err(northgate->dev, "interrupt latch clear failed: %d\n", err);
		if (!ret)
			ret = err;
	}

	return ret;
}

// Apply a new pack-connected state: schedule the max17320 attach or removal, and stop cross
// charging on removal. A negative pack_connected (failed read) is ignored.
// Returns true if the state changed. Caller holds fault_mutex.
static bool northgate_hotswap_set_pack_state(struct northgate_hotswap_psy *northgate, int pack_connected)
{
	if (pack_connected < 0 || pack_connected == northgate->removable_pack_connected)
		return false;

	cancel_delayed_work(&northgate->hotswap_work);
	northgate->attach_retries = 0;
	if (pack_connected) {
		schedule_delayed_work(&northgate->hotswap_work, msecs_to_jiffies(REMOVABLE_PACK_ATTACH_DELAY_MS));
	} else {
		northgate_set_cross_charger(northgate, false);
		schedule_delayed_work(&northgate->hotswap_work, 0);
	}
	northgate->removable_pack_connected = pack_connected;
	northgate->pack_mismatch_polls = 0;

	return true;
}

static irqreturn_t northgate_hotswap_irq_thread(int irq, void *dev_id)
{
	struct northgate_hotswap_psy *northgate = dev_id;
	int pack_connected;

	mutex_lock(&northgate->fault_mutex);

	northgate_hotswap_cache_and_clear_faults(northgate, &pack_connected);
	northgate_hotswap_set_pack_state(northgate, pack_connected);

	// Internal UVLO or a pack event may force a selected battery change. Check immediately
	northgate_hotswap_check_status(northgate);

	mutex_unlock(&northgate->fault_mutex);

	northgate_hotswap_notify(northgate);
	power_supply_changed(northgate->psy);

	return IRQ_HANDLED;
}

static const struct regmap_config northgate_hotswap_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.val_format_endian = REGMAP_ENDIAN_NATIVE,
	.cache_type = REGCACHE_NONE, // We could cache two values if the aDSP wasn't modifying them during suspend
};
// TODO add cache and use regcache_mark_dirty()
// TODO Check this for the charger as well

/* --- Probe / Remove --- */
static int northgate_hotswap_probe(struct i2c_client *client)
{
	struct northgate_hotswap_psy *northgate;
	struct power_supply_config psy_cfg = {};
	struct device *dev = &client->dev;
	unsigned int val = 0;
	int pack_connected;
	int ret;

	northgate = devm_kzalloc(dev, sizeof(*northgate), GFP_KERNEL);
	if (!northgate)
		return -ENOMEM;

	mutex_init(&northgate->fault_mutex);

	northgate->dev = dev;
	northgate->client = client;
	i2c_set_clientdata(client, northgate);

	northgate->regmap = devm_regmap_init_i2c(client, &northgate_hotswap_regmap);
	if (IS_ERR(northgate->regmap))
		return PTR_ERR(northgate->regmap);

	// Check for the Device, if its not there, just bail
	val = -1;
	ret = regmap_read(northgate->regmap, REG_PATTERN_ID, &val);
	if (ret < 0) {
		dev_err(dev, "northgate-hotswap pattern id read failed: %d\n", ret);
		return -ENODEV;
	}

	if (val != NORTHGATE_PATTERN_ID) {
		dev_err(dev, "northgate-hotswap unknown pattern id 0x%02x (expected 0x%02x)\n", val, NORTHGATE_PATTERN_ID);
		return -ENODEV;
	}

	INIT_DELAYED_WORK(&northgate->hotswap_work, northgate_hotswap_work);
	INIT_DELAYED_WORK(&northgate->status_poll_work, northgate_hotswap_status_poll);

	mutex_lock(&northgate->fault_mutex);
	ret = northgate_set_cross_charger(northgate, false);
	if (!ret)
		ret = northgate_hotswap_cache_and_clear_faults(northgate, &pack_connected);
	if (!ret) {
		northgate->removable_pack_connected = pack_connected;
		ret = northgate_hotswap_check_status(northgate);
	}
	mutex_unlock(&northgate->fault_mutex);
	if (ret < 0)
		return dev_err_probe(dev, ret, "northgate-hotswap initial hardware setup failed\n");

	// Register the power supply before the IRQ
	psy_cfg.drv_data = northgate;
	psy_cfg.fwnode = dev_fwnode(dev);
	psy_cfg.attr_grp = northgate_hotswap_groups;
	northgate->psy = devm_power_supply_register(dev, &northgate_hotswap_psy_desc, &psy_cfg);
	if (IS_ERR(northgate->psy))
		return PTR_ERR(northgate->psy);

	// setup of GPIO alert failed: -1
	if (client->irq) {
		ret = devm_request_threaded_irq(&client->dev, client->irq, NULL, northgate_hotswap_irq_thread,
						IRQF_TRIGGER_FALLING | IRQF_ONESHOT, "northgate-hotswap-alert",
						northgate);
		if (ret) {
			dev_err(&client->dev, "Unable to register IRQ %d err %d\n", client->irq, ret);
			return ret;
		}
	} else {
		dev_warn(dev, "northgate-hotswap has no IRQ, relying on the status poll\n");
	}

	// Determine Initial State
	schedule_delayed_work(&northgate->hotswap_work, 0);
	schedule_delayed_work(&northgate->status_poll_work, msecs_to_jiffies(STATUS_POLL_MS));

	dev_info(dev, "northgate-hotswap loaded\n");
	return 0;
}

static void northgate_hotswap_remove(struct i2c_client *client)
{
	struct northgate_hotswap_psy *northgate = i2c_get_clientdata(client);

	if (client->irq)
		devm_free_irq(&client->dev, client->irq, northgate);

	cancel_delayed_work_sync(&northgate->status_poll_work);
	cancel_delayed_work_sync(&northgate->hotswap_work);

	mutex_lock(&northgate->fault_mutex);
	northgate_set_cross_charger(northgate, false);
	mutex_unlock(&northgate->fault_mutex);

	if (northgate->battery_client) {
		i2c_unregister_device(northgate->battery_client);
		northgate->battery_client = NULL;
	}
}

static const struct i2c_device_id northgate_hotswap_id_table[] = { { "slg4ad48669m" }, {} };
MODULE_DEVICE_TABLE(i2c, northgate_hotswap_id_table);

#ifdef CONFIG_OF
static const struct of_device_id northgate_hotswap_of_match[] = {
	{ .compatible = "silego,northgate_hotswap" },
	{},
};
MODULE_DEVICE_TABLE(of, northgate_hotswap_of_match);
#endif

static struct i2c_driver northgate_hotswap_i2c_driver = {
	.probe = northgate_hotswap_probe,
	.remove = northgate_hotswap_remove,
	.id_table = northgate_hotswap_id_table,
	.driver = {
		.name = "northgate-hotswap",
		.of_match_table = of_match_ptr(northgate_hotswap_of_match),
	},
};
module_i2c_driver(northgate_hotswap_i2c_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Todd Goyen <todd@tgoyen.com>");
MODULE_DESCRIPTION("Steam Frame Hotswap Driver");