// SPDX-License-Identifier: GPL-2.0

#include "qcom_pm8550b_charger.h"

#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/power_supply.h>

#define UPPER_8_BITS(data) (((data) >> 8) & 0xFF)
#define LOWER_8_BITS(data) ((data) & 0xFF)

struct pm8550b_charger_port_irq_params {
	int virq;
	char *irq_name;
};

struct pm8550b_charger1_resources {
	unsigned int nr_irqs;
	struct pm8550b_charger_port_irq_params irq_params[PM8550B_CHARGER_MAX_IRQS];
};

struct pm8550b_charger2_resources {
	unsigned int nr_irqs;
	struct pm8550b_charger_port_irq_params irq_params[PM8550B_CHARGER_MAX_IRQS];
};

struct pm8550b_misc_resources {
	unsigned int nr_irqs;
	struct pm8550b_charger_port_irq_params irq_params[PM8550B_CHARGER_MAX_IRQS];
};

struct pm8550b_charger_resources {
	struct pm8550b_charger1_resources *charger1_res;
	struct pm8550b_charger2_resources *charger2_res;
	struct pm8550b_misc_resources *misc_res;
};

struct pm8550b_charger_irq_data {
	int virq;
	int irq;
	struct pm8550b_charger *charger;
};

enum base_index {
	BASE_CHG1 = 0,
	BASE_CHG2 = 1,
	BASE_MISC = 2,
    BASE_BATIF = 3,
	BASE_COUNT = 4,
};

struct pm8550b_charger {
	struct device *dev;
	struct regmap *regmap;
	u32 base[BASE_COUNT];

	struct power_supply *psy;

	unsigned int nr_irqs;
	struct pm8550b_charger_irq_data *irq_data;

	unsigned int qph1_charge_current_max;
	unsigned int qph2_charge_current_max;

	bool aicl_complete;

	/* Last CHGR_CHARGER_STATUS read from the charger IRQ, exposed via sysfs */
	u8 charger_status;

	// spinlock_t lock; /* Register atomicity */
};

struct pm8550b_charger_port {
	struct device *dev;
	struct tcpm_port *tcpm_port;
};

static int pm8550b_charger_get_charger_state(struct pm8550b_charger *chg)
{
	unsigned int reg;
	enum chgr_charge_status status;
	int state;
	int valid_power;

	if (!chg->regmap)
		return POWER_SUPPLY_STATUS_UNKNOWN;

	regmap_read(chg->regmap, chg->base[BASE_CHG1] + CHGR_CHARGER_STATUS_ADDR, &reg);
	status = (enum chgr_charge_status)(reg & CHGR_CHARGER_STATUS_MASK);
	valid_power = (reg & CHGR_CHARGER_STATUS_VALID_POWER_MASK);

	if (!valid_power) {
		return POWER_SUPPLY_STATUS_DISCHARGING;
	}

	switch (status) {
	case CHGR_CHARGER_STATUS_CHARGING_DISABLED:
	case CHGR_CHARGER_STATUS_PAUSE:
	case CHGR_CHARGER_STATUS_INHIBIT:
		if (valid_power) {
			state = POWER_SUPPLY_STATUS_NOT_CHARGING;
		} else {
			state = POWER_SUPPLY_STATUS_DISCHARGING;
		}
		break;
	case CHGR_CHARGER_STATUS_TRICKLE:
	case CHGR_CHARGER_STATUS_PRECHARGE:
	case CHGR_CHARGER_STATUS_FULLON:
	case CHGR_CHARGER_STATUS_TAPER:
		state = POWER_SUPPLY_STATUS_CHARGING;
		break;
	case CHGR_CHARGER_STATUS_TERMINATION:
		state = POWER_SUPPLY_STATUS_FULL;
		break;
	default:
		state = POWER_SUPPLY_STATUS_UNKNOWN;
	}

	return state;
}

static int pm8550b_charger_get_charge_type(struct pm8550b_charger *chg)
{
	unsigned int reg;
	enum chgr_charge_status status;
	int state;
	int valid_power;

	if (!chg->regmap)
		return POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;

	regmap_read(chg->regmap, chg->base[BASE_CHG1] + CHGR_CHARGER_STATUS_ADDR, &reg);
	status = (enum chgr_charge_status)(reg & CHGR_CHARGER_STATUS_MASK);
	valid_power = (reg & CHGR_CHARGER_STATUS_VALID_POWER_MASK);

	if (!valid_power) {
		return POWER_SUPPLY_CHARGE_TYPE_NONE;
	}

	switch (status) {
	case CHGR_CHARGER_STATUS_CHARGING_DISABLED:
	case CHGR_CHARGER_STATUS_PAUSE:
	case CHGR_CHARGER_STATUS_INHIBIT:
		if (valid_power) {
			state = POWER_SUPPLY_CHARGE_TYPE_BYPASS;
		} else {
			state = POWER_SUPPLY_CHARGE_TYPE_NONE;
		}
		break;
	case CHGR_CHARGER_STATUS_TRICKLE:
	case CHGR_CHARGER_STATUS_PRECHARGE:
		state = POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
		break;
	case CHGR_CHARGER_STATUS_FULLON:
	case CHGR_CHARGER_STATUS_TAPER:
		state = POWER_SUPPLY_CHARGE_TYPE_STANDARD;
		break;
	case CHGR_CHARGER_STATUS_TERMINATION:
		state = POWER_SUPPLY_CHARGE_TYPE_BYPASS;
		break;
	default:
		state = POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;
	}

	return state;
}

static int pm8550b_charger_get_charge_current(struct pm8550b_charger *chg)
{
	unsigned int reg;

	regmap_read(chg->regmap, chg->base[BASE_CHG1] + CHGR_ICHG_CFG_ADDR, &reg);

	/* uA Fast Charge Current */
	return (reg * CHGR_ICHG_CFG_MA_PER_UNIT * 1000);
}

static int pm8550b_charger_get_charge_current_max(struct pm8550b_charger *chg)
{
	unsigned int reg;

	regmap_read(chg->regmap, chg->base[BASE_CHG1] + CHGR_ICHG_MAX_ADDR, &reg);

	/* uA Fast Charge Current Limit */
	return (reg * CHGR_ICHG_CFG_MA_PER_UNIT * 1000);
}

static int pm8550b_charger_set_charge_current(struct pm8550b_charger *chg, int val)
{
	unsigned int reg = 0;
	bool changed = false;

	/* Charge Current to High */
	if (val > pm8550b_charger_get_charge_current_max(chg)) {
		return -EINVAL;
	}

	reg = (val / 1000) / CHGR_ICHG_CFG_MA_PER_UNIT;
	regmap_update_bits_check(chg->regmap, chg->base[BASE_CHG1] + CHGR_ICHG_CFG_ADDR, 0xFF, reg, &changed);
	if (changed) {
		power_supply_changed(chg->psy);
	}

	return 0;
}

static int pm8550b_charger_set_charge_current_max(struct pm8550b_charger *chg, int val)
{
	unsigned int reg = 0;
	bool changed = false;

	reg = (val / 1000) / CHGR_ICHG_CFG_MA_PER_UNIT;
	regmap_update_bits_check(chg->regmap, chg->base[BASE_CHG1] + CHGR_ICHG_MAX_ADDR, 0xFF, reg, &changed);
	if (changed) {
		power_supply_changed(chg->psy);
	}

	return 0;
}

static int pm8550b_charger_get_float_voltage(struct pm8550b_charger *chg)
{
	unsigned int reg = 0;

	regmap_read(chg->regmap, chg->base[BASE_CHG1] + CHGR_VFLT_CFG_ADDR, &reg);

	/* uV Float Voltage */
	return ((reg * CHGR_VFLT_CFG_MV_PER_UNIT + CHGR_VFLT_CFG_MV_OFFSET) * 1000);
}

// TODO Set this value at startup from value in dts file
static int pm8550b_charger_get_float_voltage_max(void)
{
	/* Max uV Float Voltage */
	return (CHGR_VFLT_CFG_MV_MAX * 1000);
}

static int pm8550b_charger_set_float_voltage(struct pm8550b_charger *chg, int val)
{
	unsigned int reg = 0;
	bool changed = false;

	/* Float Voltage to High */
	if (val > pm8550b_charger_get_float_voltage_max()) {
		return -EINVAL;
	}

	/* Float Voltage to Low */
	if (val / 1000 < CHGR_VFLT_CFG_MV_OFFSET) {
		return -EINVAL;
	}

	reg = (val / 1000 - CHGR_VFLT_CFG_MV_OFFSET) / CHGR_VFLT_CFG_MV_PER_UNIT;
	regmap_update_bits_check(chg->regmap, chg->base[BASE_CHG1] + CHGR_VFLT_CFG_ADDR, 0xFF, reg, &changed);
	if (changed) {
		power_supply_changed(chg->psy);
	}

	return 0;
}

static int pm8550b_charger_get_online(struct pm8550b_charger *chg)
{
	unsigned int reg = 0;

	if (!chg->regmap)
		return 0;

	regmap_read(chg->regmap, chg->base[BASE_CHG1] + CHGR_CHARGER_STATUS_ADDR, &reg);

	return !!(reg & CHGR_CHARGER_STATUS_VALID_POWER_MASK);
}

static int pm8550b_charger_set_precharge_current(struct pm8550b_charger *chg, int val)
{
	const int precharge_currents[] = { 50, 75, 100, 125, 150, 200, 250, 300 };
	unsigned int reg = 0;

	/* Scale to mA */
	val /= 1000;

	/* Find the closest current without going over */
	while (reg < (ARRAY_SIZE(precharge_currents) - 1)) {
		if (val < precharge_currents[reg]) {
			break;
		}
		reg++;
	}

	regmap_update_bits(chg->regmap, chg->base[BASE_CHG1] + CHGR_IPRE_CFG_ADDR, CHGR_IPRE_CFG_MASK, reg);

	return 0;
}

static int pm8550b_charger_get_precharge_current(struct pm8550b_charger *chg)
{
	unsigned int reg = 0;
	const int precharge_currents[] = { 50, 75, 100, 125, 150, 200, 250, 300 };

	if (!chg->regmap)
		return 0;

	regmap_read(chg->regmap, chg->base[BASE_CHG1] + CHGR_IPRE_CFG_ADDR, &reg);

	return (precharge_currents[reg & CHGR_IPRE_CFG_MASK] * 1000);
}

// TODO There is a race condition here with pm8550b_charger_get_termination_current()
static int pm8550b_charger_set_termination_current(struct pm8550b_charger *chg, int val)
{
	s16 iadc_val = 0;

	/* Scale to mA */
	val /= 1000;

	/* Invert and scale to set IADC */
	iadc_val = val * -273 / 100;

	// Set to use IADC
	regmap_update_bits(chg->regmap, chg->base[BASE_CHG1] + CHGR_TERM_CFG_ADDR,
			   CHGR_TERM_CFG_USE_ANALOG_TERMINATION, 0);

	regmap_write(chg->regmap, chg->base[BASE_CHG1] + CHGR_ADC_ITERM_UP_MSB_ADDR,
		UPPER_8_BITS(iadc_val));

	regmap_write(chg->regmap, chg->base[BASE_CHG1] + CHGR_ADC_ITERM_UP_LSB_ADDR,
		LOWER_8_BITS(iadc_val));
			   
	return 0;
}

// TODO There is a race condition here with pm8550b_charger_set_termination_current()
static int pm8550b_charger_get_termination_current(struct pm8550b_charger *chg)
{
	unsigned int reg;
	s16 iadc_val = 0;
	int term_ua = 0;
	const int termination_currents[] = { 120, 180, 240, 300, 360, 420, 480, 540 };

	if (!chg->regmap)
		return 0;

	regmap_read(chg->regmap, chg->base[BASE_CHG1] + CHGR_TERM_CFG_ADDR, &reg);

	if (!(reg & CHGR_TERM_CFG_I_TERM)) {
		return (0);
	} else if (reg & CHGR_TERM_CFG_USE_ANALOG_TERMINATION) {
		return (termination_currents[reg & CHGR_TERM_CFG_ANALOG_CHARGE_CURRENT_TERMINATION] * 1000);
	} else {
		regmap_read(chg->regmap, chg->base[BASE_CHG1] + CHGR_ADC_ITERM_UP_MSB_ADDR, &reg);
		iadc_val = ((reg & 0xFF) << 8);
		regmap_read(chg->regmap, chg->base[BASE_CHG1] + CHGR_ADC_ITERM_UP_LSB_ADDR, &reg);
		iadc_val |= (reg & 0xFF);

		term_ua = iadc_val * 100 * 1000 / -273;

		return term_ua;
	}
}

static int pm8550b_charger_get_property(struct power_supply *psy, enum power_supply_property psp,
					union power_supply_propval *val)
{
	struct pm8550b_charger *chg = power_supply_get_drvdata(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = pm8550b_charger_get_charger_state(chg);
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		val->intval = pm8550b_charger_get_charge_type(chg);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		val->intval = pm8550b_charger_get_charge_current(chg);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		val->intval = pm8550b_charger_get_charge_current_max(chg);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		val->intval = pm8550b_charger_get_float_voltage(chg);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
		val->intval = pm8550b_charger_get_float_voltage_max();
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = pm8550b_charger_get_online(chg);
		break;
	case POWER_SUPPLY_PROP_PRECHARGE_CURRENT:
		val->intval = pm8550b_charger_get_precharge_current(chg);
		break;
	case POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT:
		val->intval = pm8550b_charger_get_termination_current(chg);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int pm8550b_charger_set_property(struct power_supply *psy, enum power_supply_property psp,
					const union power_supply_propval *val)
{
	struct pm8550b_charger *chg = power_supply_get_drvdata(psy);
	int ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		ret = pm8550b_charger_set_charge_current(chg, val->intval);
		if (ret < 0) {
			return (ret);
		}
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		ret = pm8550b_charger_set_float_voltage(chg, val->intval);
		if (ret < 0) {
			return (ret);
		}
		break;
	case POWER_SUPPLY_PROP_STATUS:
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
	case POWER_SUPPLY_PROP_ONLINE:
	case POWER_SUPPLY_PROP_PRECHARGE_CURRENT:
	case POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT:
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int pm8550b_charger_is_writeable(struct power_supply *psy, enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		return 1;
		break;
	case POWER_SUPPLY_PROP_STATUS:
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
	case POWER_SUPPLY_PROP_ONLINE:
	case POWER_SUPPLY_PROP_PRECHARGE_CURRENT:
	case POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT:
	default:
		break;
	}

	return 0;
}

static enum power_supply_property pm8550b_charger_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_PRECHARGE_CURRENT,
	POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT,
};

static ssize_t aicl_status_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct pm8550b_charger *chg = power_supply_get_drvdata(psy);

	// NOTE This should work, but the Hardware clears it, so we cache the value from the interrupt
	//unsigned int reg = 0;
	//regmap_read(chg->regmap, chg->base[BASE_MISC] + MISC_AICL_STATUS, &reg);

	return sysfs_emit(buf, "%d\n", chg->aicl_complete);
}

static ssize_t aicl_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct pm8550b_charger *chg = power_supply_get_drvdata(psy);
	unsigned int reg = 0;

	if (!chg->regmap)
		return 0;

	regmap_read(chg->regmap, chg->base[BASE_MISC] + MISC_AICL_RERUN_TIME_CFG, &reg);

	return sysfs_emit(buf, "%d\n", !!(reg & MISC_AICL_RERUN_TIME_ENABLE_MASK));
}

static ssize_t aicl_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct pm8550b_charger *chg = power_supply_get_drvdata(psy);
	bool enable = false;
	int ret;

	if (!chg->regmap)
		return 0;

	ret = kstrtobool(buf, &enable);
	if (ret != 0) {
		return ret;
	}

	// NOTE Technically we should en/disable QC AICL, but we don't use QC
	// 0x72968 & ~0x80

	regmap_update_bits(chg->regmap, chg->base[BASE_MISC] + MISC_AICL_RERUN_TIME_CFG,
			   MISC_AICL_RERUN_TIME_ENABLE_MASK, (enable ? MISC_AICL_RERUN_TIME_ENABLE_MASK : 0));

	return count;
}

static ssize_t aicl_rerun_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct pm8550b_charger *chg = power_supply_get_drvdata(psy);
	bool rerun = false;
	int ret;

	if (!chg->regmap)
		return 0;

	ret = kstrtobool(buf, &rerun);
	if (ret != 0) {
		return ret;
	}

	if (rerun)
		regmap_write(chg->regmap, chg->base[BASE_MISC] + MISC_AICL_CMD, MISC_AICL_RESTART_MASK);

	return count;
}

static bool charging_pause_get(struct pm8550b_charger *chg)
{
	unsigned int reg;

	regmap_read(chg->regmap, chg->base[BASE_CHG1] + CHGR_CHG_EN_ADDR, &reg);

	return (!!(reg & CHGR_CHG_EN_CHARGING_PAUSE_CMD));
}

static void charging_pause_set(struct pm8550b_charger *chg, bool pause)
{
	bool paused = false;

	paused = charging_pause_get(chg);
	if (paused != pause) {
		regmap_update_bits(chg->regmap, chg->base[BASE_CHG1] + CHGR_CHG_EN_ADDR, CHGR_CHG_EN_CHARGING_PAUSE_CMD,
				   (pause ? CHGR_CHG_EN_CHARGING_PAUSE_CMD : 0));
	}
}

static int charging_ship_mode_set(struct pm8550b_charger *chg)
{
	return regmap_write(chg->regmap, chg->base[BASE_BATIF] + BATIF_SHIP_MODE, 1);
}

static ssize_t charging_pause_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct pm8550b_charger *chg = power_supply_get_drvdata(psy);

	if (!chg->regmap)
		return 0;

	return sysfs_emit(buf, "%d\n", charging_pause_get(chg));
}

static ssize_t charging_pause_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct pm8550b_charger *chg = power_supply_get_drvdata(psy);
	bool pause = false;
	int ret;

	if (!chg->regmap)
		return 0;

	ret = kstrtobool(buf, &pause);
	if (ret != 0) {
		return ret;
	}

	charging_pause_set(chg, pause);

	return count;
}

static bool charging_enable_get(struct pm8550b_charger *chg)
{
	unsigned int reg;

	regmap_read(chg->regmap, chg->base[BASE_CHG1] + CHGR_CHG_EN_ADDR, &reg);

	return (!!(reg & CHGR_CHG_EN_CHARGING_ENABLE_CMD));
}

// TODO There should be locking here, but likely OK since we only have one writer
static void charging_enable_set(struct pm8550b_charger *chg, bool enable)
{
	enum chgr_charge_status status;
	bool enabled = false;
	unsigned int reg;

	regmap_read(chg->regmap, chg->base[BASE_CHG1] + CHGR_CHARGER_STATUS_ADDR, &reg);
	status = (enum chgr_charge_status)(reg & CHGR_CHARGER_STATUS_MASK);

	if ((status == CHGR_CHARGER_STATUS_TERMINATION || status == CHGR_CHARGER_STATUS_INHIBIT ||
	     status == CHGR_CHARGER_STATUS_CHARGING_DISABLED) &&
	    enable) {
		// To exit termination, inhibit, or error (disabled), you have to fully toggle charging
		regmap_update_bits(chg->regmap, chg->base[BASE_CHG1] + CHGR_CHG_EN_ADDR,
				   CHGR_CHG_EN_CHARGING_ENABLE_CMD, 0);

		regmap_update_bits(chg->regmap, chg->base[BASE_CHG1] + CHGR_CHG_EN_ADDR,
				   CHGR_CHG_EN_CHARGING_ENABLE_CMD, CHGR_CHG_EN_CHARGING_ENABLE_CMD);

	} else {
		// Else only toggle if its a real change
		enabled = charging_enable_get(chg);
		if (enabled != enable) {
			regmap_update_bits(chg->regmap, chg->base[BASE_CHG1] + CHGR_CHG_EN_ADDR,
					   CHGR_CHG_EN_CHARGING_ENABLE_CMD,
					   (enable ? CHGR_CHG_EN_CHARGING_ENABLE_CMD : 0));
		}
	}
}

static ssize_t charging_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct pm8550b_charger *chg = power_supply_get_drvdata(psy);

	if (!chg->regmap)
		return 0;

	return sysfs_emit(buf, "%d\n", charging_enable_get(chg));
}

static ssize_t charging_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct pm8550b_charger *chg = power_supply_get_drvdata(psy);
	bool enable = false;
	int ret;

	if (!chg->regmap)
		return 0;

	ret = kstrtobool(buf, &enable);
	if (ret != 0) {
		return ret;
	}

	charging_enable_set(chg, enable);

	return count;
}

static ssize_t status_detailed_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct pm8550b_charger *chg = power_supply_get_drvdata(psy);
	unsigned int reg;
	int at = 0;

	if (!chg->regmap)
		return 0;

	/* Charger QG Status */
	regmap_read(chg->regmap, chg->base[BASE_CHG1] + CHGR_QG_CHG_STS_ADDR, &reg);
	at += sysfs_emit_at(buf, at, "JEITA: %d\n", !!(reg & CHGR_QG_CHG_STS_JEITA_WARM_COOL_MASK));
	at += sysfs_emit_at(buf, at, "CV: %d\n", !!(reg & CHGR_QG_CHG_STS_CV_CHARGING_MASK));
	at += sysfs_emit_at(buf, at, "CC: %d\n", !!(reg & CHGR_QG_CHG_STS_CC_CHARGING_MASK));
	at += sysfs_emit_at(buf, at, "ICL: %d\n", !!(reg & CHGR_QG_CHG_STS_INPUT_CURRENT_LIMIT_MASK));
	at += sysfs_emit_at(buf, at, "Boost: %d\n", !!(reg & CHGR_QG_CHG_STS_BOOST_SWITCHING_MASK));
	at += sysfs_emit_at(buf, at, "Buck: %d\n", !!(reg & CHGR_QG_CHG_STS_BUCK_SWITCHING_MASK));
	at += sysfs_emit_at(buf, at, "USB: %d\n", !!(reg & CHGR_QG_CHG_STS_USB_PRESENT_MASK));

	/* Charger Status */
	regmap_read(chg->regmap, chg->base[BASE_CHG1] + CHGR_CHG_RT_STS, &reg);
	at += sysfs_emit_at(buf, at, "Input OK: %d\n", !!(reg & CHGR_CHG_CHG_STS_INPUT_OK_MASK));
	at += sysfs_emit_at(buf, at, "Buck OK: %d\n", !!(reg & CHGR_CHG_CHG_STS_BUCK_OK_MASK));
	at += sysfs_emit_at(buf, at, "Temp OK: %d\n", !!(reg & CHGR_CHG_CHG_STS_TEMP_OK_MASK));
	at += sysfs_emit_at(buf, at, "BATT OK: %d\n", !!(reg & CHGR_CHG_CHG_STS_BATT_OK_MASK));
	at += sysfs_emit_at(buf, at, "CHG Ctrl OK: %d\n", !!(reg & CHGR_CHG_CHG_STS_CHG_CTRL_OK_MASK));

	/* USB Status */
	regmap_read(chg->regmap, chg->base[BASE_CHG1] + CHGR_USB_RT_STS, &reg);
	at += sysfs_emit_at(buf, at, "USB UV: %d\n", !!(reg & CHGR_USB_RT_STS_USB_UV_MASK));
	at += sysfs_emit_at(buf, at, "USB OV: %d\n", !!(reg & CHGR_USB_RT_STS_USB_OV_MASK));
	at += sysfs_emit_at(buf, at, "USB Suspend: %d\n", !!(reg & CHGR_USB_RT_STS_USB_SUSPEND_MASK));
	at += sysfs_emit_at(buf, at, "IMP Trigger: %d\n", !!(reg & CHGR_USB_RT_STS_IMP_TRIGGER_MASK));

	/* Buck Status */
	regmap_read(chg->regmap, chg->base[BASE_CHG1] + CHGR_BUCK_RT_STS, &reg);
	at += sysfs_emit_at(buf, at, "Buck SS Done: %d\n", !(reg & CHGR_BUCK_RT_STS_SS_NOT_DONE_MASK));
	at += sysfs_emit_at(buf, at, "Buck OC: %d\n", !!(reg & CHGR_BUCK_RT_STS_OC_MASK));
	at += sysfs_emit_at(buf, at, "VPH OV: %d\n", !!(reg & CHGR_BUCK_RT_STS_VPH_OV_MASK));

	/* Temp Status */
	regmap_read(chg->regmap, chg->base[BASE_CHG1] + CHGR_TEMP_RT_STS, &reg);
	at += sysfs_emit_at(buf, at, "Die Temp Bad: %d\n", !!(reg & CHGR_TEMP_RT_STS_CHGR_TEMP_MASK));

	/* Battery Status */
	regmap_read(chg->regmap, chg->base[BASE_CHG1] + CHGR_BATT_RT_STS, &reg);
	at += sysfs_emit_at(buf, at, "JEITA Cold: %d\n", !!(reg & CHGR_BATT_RT_STS_JEITA_COLD_MASK));
	at += sysfs_emit_at(buf, at, "JEITA Hot: %d\n", !!(reg & CHGR_BATT_RT_STS_JEITA_HOT_MASK));

	/* QPH Status */
	regmap_read(chg->regmap, chg->base[BASE_CHG2] + CHGR2_QPH2_RT_STS, &reg);
	at += sysfs_emit_at(buf, at, "QPH VBAT1 OK: %d\n", !!(reg & CHGR2_QPH2_RT_STS_VBAT1_HI_MASK));
	at += sysfs_emit_at(buf, at, "QPH VIN OK: %d\n", !!(reg & CHGR2_QPH2_RT_STS_VIN_HI_MASK));
	at += sysfs_emit_at(buf, at, "QPH2: %d\n", !!(reg & CHGR2_QPH2_RT_STS_QPH2_ON_MASK));

	return (at);
}

static void termination_enable_set(struct pm8550b_charger *chg, bool enable)
{
	regmap_update_bits(chg->regmap, chg->base[BASE_CHG1] + CHGR_TERM_CFG_ADDR, CHGR_TERM_CFG_I_TERM,
			   (enable ? CHGR_TERM_CFG_I_TERM : 0));
}

static ssize_t termination_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct pm8550b_charger *chg = power_supply_get_drvdata(psy);
	unsigned int reg;

	if (!chg->regmap)
		return 0;

	regmap_read(chg->regmap, chg->base[BASE_CHG1] + CHGR_TERM_CFG_ADDR, &reg);

	return sysfs_emit(buf, "%d\n", !!(reg & CHGR_TERM_CFG_I_TERM));
}

static ssize_t termination_enable_store(struct device *dev, struct device_attribute *attr, const char *buf,
					size_t count)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct pm8550b_charger *chg = power_supply_get_drvdata(psy);
	bool enable = false;
	int ret;

	if (!chg->regmap)
		return 0;

	ret = kstrtobool(buf, &enable);
	if (ret != 0) {
		return ret;
	}

	termination_enable_set(chg, enable);

	return (count);
}

static ssize_t qph2_allow_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct pm8550b_charger *chg = power_supply_get_drvdata(psy);
	unsigned int reg;

	if (!chg->regmap)
		return 0;

	regmap_read(chg->regmap, chg->base[BASE_CHG2] + CHGR2_QPH2_CHG_CTRL, &reg);

	return sysfs_emit(buf, "%d\n", !!(reg & CHGR2_QPH2_CHG_CTRL_ALLOW_QPH2_MASK));
}

// TODO Add Locking
static ssize_t qph2_allow_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct pm8550b_charger *chg = power_supply_get_drvdata(psy);
	bool enable = false;
	int ret;

	if (!chg->regmap)
		return 0;

	ret = kstrtobool(buf, &enable);
	if (ret != 0) {
		return ret;
	}

	if (enable) {
		/* Set Input Current Limit Min dring QPH1 <-> QPH2 Transition */
		regmap_write(chg->regmap, chg->base[BASE_CHG2] + CHGR2_QPH2_CHG_TRANS_ICL_MIN, 0x02);
	}

	regmap_update_bits(chg->regmap, chg->base[BASE_CHG2] + CHGR2_QPH2_CHG_CTRL, CHGR2_QPH2_CHG_CTRL_ALLOW_QPH2_MASK,
			   (enable ? CHGR2_QPH2_CHG_CTRL_ALLOW_QPH2_MASK : 0));

	return (count);
}

static ssize_t qph2_resume_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct pm8550b_charger *chg = power_supply_get_drvdata(psy);
	bool enable = false;
	int ret;

	if (!chg->regmap)
		return 0;

	ret = kstrtobool(buf, &enable);
	if (ret != 0) {
		return ret;
	}

	if (enable) {
		/* Resume QPH2 */
		regmap_write(chg->regmap, chg->base[BASE_CHG2] + CHGR2_QPH2_CHG_RESUME, CHGR2_QPH2_CHG_RESUME_MASK);
	}

	return (count);
}

static ssize_t qph2_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct pm8550b_charger *chg = power_supply_get_drvdata(psy);
	unsigned int reg;

	if (!chg->regmap)
		return 0;

	regmap_read(chg->regmap, chg->base[BASE_CHG2] + CHGR2_INT_RT_STS, &reg);

	return sysfs_emit(buf, "%d\n", !!(reg & CHGR2_INT_CHG_PATH_STS_MASK));
}

static ssize_t charger_status_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct pm8550b_charger *chg = power_supply_get_drvdata(psy);

	return sysfs_emit(buf, "%u\n", READ_ONCE(chg->charger_status));
}

/**
 * pm8550b_temp_sts_pack() - Pack temperature related STS bits into one byte
 * @chg_sts: CHG_RT_STS or CHG_LT_STS value
 * @temp_sts: TEMP_RT_STS or TEMP_LT_STS value
 * @batt_sts: BATT_RT_STS or BATT_LT_STS value
 *
 * Return: PM8550B_TEMP_STS_* bits 6:0
 */
static unsigned int pm8550b_temp_sts_pack(unsigned int chg_sts, unsigned int temp_sts,
					  unsigned int batt_sts)
{
	unsigned int val = 0;

	if (chg_sts & CHGR_CHG_CHG_STS_TEMP_OK_MASK)
		val |= PM8550B_TEMP_STS_TEMP_OK;
	if (temp_sts & CHGR_TEMP_RT_STS_CHGR_TEMP_MASK)
		val |= PM8550B_TEMP_STS_CHG_TEMP_SHDN;
	if (temp_sts & CHGR_TEMP_RT_STS_SMB_TEMP_MASK)
		val |= PM8550B_TEMP_STS_SMB_TEMP_SHDN;
	if (temp_sts & CHGR_TEMP_RT_STS_WLS_THERM_MASK)
		val |= PM8550B_TEMP_STS_WLS_THERM_SHDN;
	if (temp_sts & CHGR_TEMP_RT_STS_USB_THERM_MASK)
		val |= PM8550B_TEMP_STS_USB_THERM_SHDN;
	if (batt_sts & CHGR_BATT_RT_STS_JEITA_COLD_MASK)
		val |= PM8550B_TEMP_STS_JEITA_COLD;
	if (batt_sts & CHGR_BATT_RT_STS_JEITA_HOT_MASK)
		val |= PM8550B_TEMP_STS_JEITA_HOT;

	return val;
}

/*
 * Combined temperature status as a single decimal bitmask, see
 * PM8550B_TEMP_STS_*. Live flags are in bits 7:0 and latched flags in bits
 * 15:8; the latched copy holds the state from the last time charging
 * stopped, until it is cleared through CHG_LT_CLR.
 */
static ssize_t charger_temp_status_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct pm8550b_charger *chg = power_supply_get_drvdata(psy);
	unsigned int qg_sts, chg_rt, chg_lt, temp_rt, temp_lt, batt_rt, batt_lt;
	unsigned int val;
	u32 base;
	int ret;

	if (!chg->regmap)
		return 0;

	base = chg->base[BASE_CHG1];

	ret = regmap_read(chg->regmap, base + CHGR_QG_CHG_STS_ADDR, &qg_sts);
	if (!ret)
		ret = regmap_read(chg->regmap, base + CHGR_CHG_RT_STS, &chg_rt);
	if (!ret)
		ret = regmap_read(chg->regmap, base + CHGR_CHG_LT_STS, &chg_lt);
	if (!ret)
		ret = regmap_read(chg->regmap, base + CHGR_TEMP_RT_STS, &temp_rt);
	if (!ret)
		ret = regmap_read(chg->regmap, base + CHGR_TEMP_LT_STS, &temp_lt);
	if (!ret)
		ret = regmap_read(chg->regmap, base + CHGR_BATT_RT_STS, &batt_rt);
	if (!ret)
		ret = regmap_read(chg->regmap, base + CHGR_BATT_LT_STS, &batt_lt);
	if (ret)
		return ret;

	val = pm8550b_temp_sts_pack(chg_rt, temp_rt, batt_rt);
	val |= pm8550b_temp_sts_pack(chg_lt, temp_lt, batt_lt) << PM8550B_TEMP_STS_LATCHED_SHIFT;

	/* Warm/cool zone has no latched copy */
	if (qg_sts & CHGR_QG_CHG_STS_JEITA_WARM_COOL_MASK)
		val |= PM8550B_TEMP_STS_JEITA_WARM_COOL;

	return sysfs_emit(buf, "%u\n", val);
}

static ssize_t ship_mode_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct pm8550b_charger *chg = power_supply_get_drvdata(psy);
	bool enable = false;
	int ret;

	if (!chg->regmap)
		return 0;

	ret = kstrtobool(buf, &enable);
	if (ret != 0) {
		return ret;
	}

	if (enable)
		charging_ship_mode_set(chg);

	return count;
}

// TODO
// auto_recharge_threshold - TODO Should this be in userspace?
// auto recharge enabled?

static DEVICE_ATTR_RO(aicl_status);
static DEVICE_ATTR_RW(aicl_enable);
static DEVICE_ATTR_WO(aicl_rerun);
static DEVICE_ATTR_RW(charging_pause);
static DEVICE_ATTR_RW(charging_enable);
static DEVICE_ATTR_RO(status_detailed);
static DEVICE_ATTR_RW(termination_enable);
static DEVICE_ATTR_RW(qph2_allow);
static DEVICE_ATTR_WO(qph2_resume);
static DEVICE_ATTR_RO(qph2);
static DEVICE_ATTR_WO(ship_mode);
static DEVICE_ATTR_RO(charger_status);
static DEVICE_ATTR_RO(charger_temp_status);

static struct attribute *pm8550b_battery_attrs[] = {
	&dev_attr_aicl_status.attr,
	&dev_attr_aicl_enable.attr,
	&dev_attr_aicl_rerun.attr,
	&dev_attr_charging_pause.attr,
	&dev_attr_charging_enable.attr,
	&dev_attr_status_detailed.attr,
	&dev_attr_termination_enable.attr,
	&dev_attr_qph2_allow.attr,
	&dev_attr_qph2_resume.attr,
	&dev_attr_qph2.attr,
	&dev_attr_ship_mode.attr,
	&dev_attr_charger_status.attr,
	&dev_attr_charger_temp_status.attr,
	NULL,
};
ATTRIBUTE_GROUPS(pm8550b_battery);

static const struct power_supply_desc pm8550b_charger_desc = {
	.name = "pm8550b-charger",
	.properties = pm8550b_charger_props,
	.num_properties = ARRAY_SIZE(pm8550b_charger_props),
	.get_property = pm8550b_charger_get_property,
	.set_property = pm8550b_charger_set_property,
	.property_is_writeable = pm8550b_charger_is_writeable,
};

static const char * const pm8550b_charger_status_strings[] = {
	[CHGR_CHARGER_STATUS_INHIBIT] = "STATUS_INHIBIT",
	[CHGR_CHARGER_STATUS_TRICKLE] = "STATUS_TRICKLE",
	[CHGR_CHARGER_STATUS_PRECHARGE] = "STATUS_PRECHARGE",
	[CHGR_CHARGER_STATUS_FULLON] = "STATUS_FULLON",
	[CHGR_CHARGER_STATUS_TAPER] = "STATUS_TAPER",
	[CHGR_CHARGER_STATUS_TERMINATION] = "STATUS_TERMINATION",
	[CHGR_CHARGER_STATUS_PAUSE] = "STATUS_CHARGING_PAUSE",
	[CHGR_CHARGER_STATUS_CHARGING_DISABLED] = "STATUS_CHARGING_DISABLED",
};

static const char *pm8550b_charger_status_str(unsigned int status)
{
	if (status >= ARRAY_SIZE(pm8550b_charger_status_strings) || !pm8550b_charger_status_strings[status])
		return "STATUS_UNKNOWN";

	return pm8550b_charger_status_strings[status];
}

/**
 * pm8550b_charger_update_status() - Cache the charger status register
 * @chg: charger instance
 *
 * Reads CHGR_CHARGER_STATUS, stores it in @chg->charger_status and notifies
 * pollers of the charger_status sysfs attribute. Pollers are notified on
 * every call, so a repeated error with an unchanged value still wakes them.
 *
 * Return: the raw register value.
 */
static unsigned int pm8550b_charger_update_status(struct pm8550b_charger *chg)
{
	unsigned int reg = 0;

	regmap_read(chg->regmap, chg->base[BASE_CHG1] + CHGR_CHARGER_STATUS_ADDR, &reg);

	WRITE_ONCE(chg->charger_status, (u8)reg);
	sysfs_notify(&chg->psy->dev.kobj, NULL, dev_attr_charger_status.attr.name);

	return reg;
}

static irqreturn_t qcom_pm8550b_charger_isr(int irq, void *dev_id)
{
	struct pm8550b_charger_irq_data *irq_data = dev_id;
	struct pm8550b_charger *chg = irq_data->charger;
	unsigned int reg = 0;
	bool overcurrent, overvoltage, safety_timer;

	dev_dbg(&chg->psy->dev, "8550B IRQ: %i\n", irq_data->virq);

	switch (irq_data->virq) {
	case PM8550B_CHARGER_STATE_CHANGE_IRQ:
		reg = pm8550b_charger_update_status(chg);

		overcurrent = !!(reg & BIT(3));
		overvoltage = !!(reg & BIT(4));
		safety_timer = !!(reg & BIT(5));
		dev_dbg(&chg->psy->dev, "8550B REG: %u: %s, overcurrent: %i, overvoltage: %i, safety_timer: %i\n",
			reg, pm8550b_charger_status_str(reg & CHGR_CHARGER_STATUS_MASK),
			overcurrent, overvoltage, safety_timer);

		if (!(reg & CHGR_CHARGER_STATUS_VALID_POWER_MASK)) {
			// If power is disconnected, clear AICL... Best I have got at the moment
			chg->aicl_complete = false;
		}

		power_supply_changed(chg->psy);
		break;
	case PM8550B_CHARGER_CHARGING_ERROR_IRQ:
		/* Safety Timer Expiration, Battery Overvoltage, or Battery Missing/Detected */
		reg = pm8550b_charger_update_status(chg);
		power_supply_changed(chg->psy);
		dev_warn(&chg->psy->dev, "Charging error IRQ, status: %u\n", reg);
		break;
	}

	return IRQ_HANDLED;
}

static void qcom_pm8550b_set_qph_charge_current(struct pm8550b_charger *chg)
{
	unsigned int reg = 0;
	regmap_read(chg->regmap, chg->base[BASE_CHG2] + CHGR2_INT_RT_STS, &reg);

	/* Change Current Limit */
	if (!!(reg & CHGR2_INT_CHG_PATH_STS_MASK))
		pm8550b_charger_set_charge_current_max(chg, chg->qph2_charge_current_max);
	else
		pm8550b_charger_set_charge_current_max(chg, chg->qph1_charge_current_max);

	sysfs_notify(&chg->psy->dev.kobj, NULL, dev_attr_qph2.attr.name);
}

static irqreturn_t qcom_pm8550b_charger2_isr(int irq, void *dev_id)
{
	struct pm8550b_charger_irq_data *irq_data = dev_id;
	struct pm8550b_charger *chg = irq_data->charger;

	if (!chg->regmap)
		return IRQ_HANDLED;

	switch (irq_data->virq) {
	case PM8550B_CHARGER2_CHARGE_PATH_CHANGE_IRQ:
		qcom_pm8550b_set_qph_charge_current(chg);
		break;
	case PM8550B_CHARGER2_FULL_DUTY_CHARGE_IRQ:
		// TODO This should be off, maybe drop the IRQ
		power_supply_changed(chg->psy);
		break;
	}

	return IRQ_HANDLED;
}

static irqreturn_t qcom_pm8550b_misc_isr(int irq, void *dev_id)
{
	struct pm8550b_charger_irq_data *irq_data = dev_id;
	struct pm8550b_charger *chg = irq_data->charger;

	if (!chg->regmap)
		return IRQ_HANDLED;

	// spin_lock_irq(&chg->lock);
	dev_dbg(&chg->psy->dev, "8550B MISC IRQ: %i\n", irq_data->virq);

	switch (irq_data->virq) {
	case PM8550B_MISC_AICL_FAIL_IRQ:
		// TODO We care about this, but not sure how to handle it...
		dev_dbg(&chg->psy->dev, "AICL failed\n");

		break;
	case PM8550B_MISC_AICL_DONE_IRQ:
		// NOTE The register is fleeting and you cannot read it here and expect it to still be set.
		// The Interrupt implies that it happened.
		chg->aicl_complete = true;

		sysfs_notify(&chg->psy->dev.kobj, NULL, dev_attr_aicl_status.attr.name);
		break;
	}

	// spin_unlock_irq(&chg->lock);

	return IRQ_HANDLED;
}

static void qcom_pm8550b_charger_setup_jeita(struct pm8550b_charger *chg)
{
	// Nominal Charging Window is 5 to 45 C
	// Extended Charging Window is 0 to 60 C. We limit this to 46, since the battery shouldn't get that hot.
	// Discharging Window is -20 to 65 C
	// In Extended Charging Window Current is limited to 0.2 C
	// At High End,

	// TODO Test this, because at 2S voltages are typically still 1S levels in PMIC
	// 8.2 Volts is 700 mV below
	regmap_write(chg->regmap, chg->base[BASE_BATIF] + BATIF_JEITA_FV_WARM, (8900 - 8200) / 10);
	regmap_write(chg->regmap, chg->base[BASE_BATIF] + BATIF_JEITA_FV_COOL, 0);
	// TODO These need to be dynamic based on FCC and are an offset from it
	// TODO Verify the scaling here is really 50 mA and not 60 mA
	regmap_write(chg->regmap, chg->base[BASE_BATIF] + BATIF_JEITA_CC_WARM, 43);
	regmap_write(chg->regmap, chg->base[BASE_BATIF] + BATIF_JEITA_CC_COOL, 43);

	// These are from Beta Equation with 100k thermistor, 100k pullup, Beta=4250, and Full Scale ADC of 0x4000
	// const u16 temp_p75 = 0x0751;
	const u16 temp_p65 = 0x0A00;
	// const u16 temp_p60 = 0x0BB2;
	// const u16 temp_p50 = 0x0DAC;
	const u16 temp_p46 = 0x1201;
	const u16 temp_p45 = 0x128C;
	// const u16 temp_p44 = 0x131C;
	// const u16 temp_p25 = 0x2000;
	// const u16 temp_p10 = 0x2B8A;
	// const u16 temp_p6  = 0x2E69;
	const u16 temp_p5 = 0x2F19;
	const u16 temp_p4 = 0x2FC6;
	// const u16 temp_0   = 0x3257;
	const u16 temp_n20 = 0x3B4B;

	// Too Hot to Charge - 46 C
	regmap_write(chg->regmap, chg->base[BASE_BATIF] + BATIF_JEITA_THOT_MSB, UPPER_8_BITS(temp_p46));
	regmap_write(chg->regmap, chg->base[BASE_BATIF] + BATIF_JEITA_THOT_LSB, LOWER_8_BITS(temp_p46));

	// Too Warm for Full Charging - 45 C
	regmap_write(chg->regmap, chg->base[BASE_BATIF] + BATIF_JEITA_WARM_MSB, UPPER_8_BITS(temp_p45));
	regmap_write(chg->regmap, chg->base[BASE_BATIF] + BATIF_JEITA_WARM_LSB, LOWER_8_BITS(temp_p45));

	// Too Cool for Full Charging - 5 C
	regmap_write(chg->regmap, chg->base[BASE_BATIF] + BATIF_JEITA_COOL_MSB, UPPER_8_BITS(temp_p5));
	regmap_write(chg->regmap, chg->base[BASE_BATIF] + BATIF_JEITA_COOL_LSB, LOWER_8_BITS(temp_p5));

	// Too Cold to Charge - 4 C
	regmap_write(chg->regmap, chg->base[BASE_BATIF] + BATIF_JEITA_COLD_MSB, UPPER_8_BITS(temp_p4));
	regmap_write(chg->regmap, chg->base[BASE_BATIF] + BATIF_JEITA_COLD_LSB, LOWER_8_BITS(temp_p4));

	// Too Hot to Run - 65 C
	regmap_write(chg->regmap, chg->base[BASE_BATIF] + BATIF_JEITA_THOT_AFP_MSB, UPPER_8_BITS(temp_p65));
	regmap_write(chg->regmap, chg->base[BASE_BATIF] + BATIF_JEITA_THOT_AFP_LSB, LOWER_8_BITS(temp_p65));

	// Too Cold to Run - -20 C
	regmap_write(chg->regmap, chg->base[BASE_BATIF] + BATIF_JEITA_COLD_AFP_MSB, UPPER_8_BITS(temp_n20));
	regmap_write(chg->regmap, chg->base[BASE_BATIF] + BATIF_JEITA_COLD_AFP_LSB, LOWER_8_BITS(temp_n20));

	// Enable JEITA Window (Everything but Float Voltage Compenstation in Constant Voltage Mode)
	// This is the same thing XBL sets up.
	regmap_write(chg->regmap, chg->base[BASE_BATIF] + BATIF_JEITA_EN, 0xEF);
}

static int qcom_pm8550b_charger_probe(struct platform_device *pdev)
{
	struct pm8550b_charger *chg;
	struct device *dev = &pdev->dev;
	const struct pm8550b_charger_resources *res;
	struct device_node *np = dev->of_node;
	struct regmap *regmap;
	int ret;

	/* Get Data from driver for this device */
	res = of_device_get_match_data(dev);
	if (!res)
		return -ENODEV;

	/* Allocate memory for the device */
	chg = devm_kzalloc(dev, sizeof(*chg), GFP_KERNEL);
	if (!chg)
		return -ENOMEM;

	chg->dev = dev;

	/* Get the regmap for communication */
	regmap = dev_get_regmap(dev->parent, NULL);
	if (!regmap) {
		dev_err(dev, "Failed to get regmap\n");
		return -ENODEV;
	}
	chg->regmap = regmap;

	/* Get the base register */
	ret = of_property_read_u32_array(np, "reg", chg->base, BASE_COUNT);
	if (ret)
		return ret;

	ret = of_property_read_u32(np, "qph1-charge-current-max", &chg->qph1_charge_current_max);
	if (ret) {
		chg->qph1_charge_current_max = 500000;
		pr_warn("Setting Default QPH1 Current (%d)\n", chg->qph1_charge_current_max);
	}

	ret = of_property_read_u32(np, "qph2-charge-current-max", &chg->qph2_charge_current_max);
	if (ret) {
		chg->qph2_charge_current_max = 500000;
		pr_warn("Setting Default QPH2 Current (%d)\n", chg->qph1_charge_current_max);
	}

	/* Setup Lock */
	// TODO Should this be mutex or spinlock?
	// spin_lock_init(&chg->lock);

	/* Set Driver Data on platform device */
	platform_set_drvdata(pdev, chg);

	/* Setup Power Supply */
	struct power_supply_config psy_cfg = {};
	psy_cfg.fwnode = dev_fwnode(&pdev->dev);
	psy_cfg.drv_data = chg;
	psy_cfg.attr_grp = pm8550b_battery_groups;

	chg->psy = devm_power_supply_register(dev, &pm8550b_charger_desc, &psy_cfg);
	if (IS_ERR(chg->psy))
		return dev_err_probe(dev, PTR_ERR(chg->psy), "Failed to register power supply\n");

	/* Seed the cached status before the IRQs can update it */
	pm8550b_charger_update_status(chg);

	/* Setup IRQ */
	struct pm8550b_charger_irq_data *irq_data;
	int i, irq;
	const struct pm8550b_charger1_resources *chg1_res = res->charger1_res;
	const struct pm8550b_charger2_resources *chg2_res = res->charger2_res;
	const struct pm8550b_misc_resources *misc_res = res->misc_res;
	chg->nr_irqs = chg1_res->nr_irqs + chg2_res->nr_irqs + misc_res->nr_irqs;

	irq_data = devm_kzalloc(dev, sizeof(*irq_data) * chg->nr_irqs, GFP_KERNEL);
	if (!irq_data)
		return -ENOMEM;
	chg->irq_data = irq_data;

	/* Setup Charger 1 Interrupt */
	for (i = 0; i < chg1_res->nr_irqs; i++, irq_data++) {
		irq = platform_get_irq_byname(pdev, chg1_res->irq_params[i].irq_name);
		if (irq < 0)
			return irq;

		irq_data->charger = chg;
		irq_data->irq = irq;
		irq_data->virq = chg1_res->irq_params[i].virq;

		ret = devm_request_threaded_irq(dev, irq, NULL, qcom_pm8550b_charger_isr, IRQF_ONESHOT,
						chg1_res->irq_params[i].irq_name, irq_data);
		if (ret)
			return ret;
	}

	/* Setup Charger 2 Interrupt */
	for (i = 0; i < chg2_res->nr_irqs; i++, irq_data++) {
		irq = platform_get_irq_byname(pdev, chg2_res->irq_params[i].irq_name);
		if (irq < 0)
			return irq;

		irq_data->charger = chg;
		irq_data->irq = irq;
		irq_data->virq = chg2_res->irq_params[i].virq;

		ret = devm_request_threaded_irq(dev, irq, NULL, qcom_pm8550b_charger2_isr, IRQF_ONESHOT,
						chg2_res->irq_params[i].irq_name, irq_data);
		if (ret)
			return ret;
	}

	/* Setup MISC Interrupt */
	for (i = 0; i < misc_res->nr_irqs; i++, irq_data++) {
		irq = platform_get_irq_byname(pdev, misc_res->irq_params[i].irq_name);
		if (irq < 0)
			return irq;

		irq_data->charger = chg;
		irq_data->irq = irq;
		irq_data->virq = misc_res->irq_params[i].virq;

		ret = devm_request_threaded_irq(dev, irq, NULL, qcom_pm8550b_misc_isr, IRQF_ONESHOT,
						misc_res->irq_params[i].irq_name, irq_data);
		if (ret)
			return ret;
	}

	/* Initial Setup */
	pm8550b_charger_set_precharge_current(chg, 250000);
	/* 1.20 A for 2S - C/2.25
	 * NOTE When we measure this current the divider seems to be closer to 57 than 60...
	 * Not sure why this is. so all are currents are a little lower than this.
	 */
	qcom_pm8550b_set_qph_charge_current(chg);
	pm8550b_charger_set_charge_current(chg, 1000000);
	pm8550b_charger_set_float_voltage(chg, 8900000);
	pm8550b_charger_set_termination_current(chg, 135000);
	termination_enable_set(chg, true);

	/* Setup ISNS for QPH2 Charging with System Load */
	regmap_update_bits(chg->regmap, chg->base[BASE_CHG2] + CHGR2_ICHG_SNS_CTRL, CHGR2_ICHG_SNS_CTRL_MASK,
		CHGR2_ICHG_SNS_CTRL_RSNS_SEL_1MOHM | 
		CHGR2_ICHG_SNS_CTRL_USE_RSNS | 
		CHGR2_ICHG_SNS_CTRL_EN_QBAT1_ISNS_CHG | 
		CHGR2_ICHG_SNS_CTRL_EN_QBAT1_ISNS_DISCHG);

	/* disable watchdog */
	regmap_write(chg->regmap, chg->base[BASE_CHG1] + CHGR_WD_CFG_ADDR, 0x00);

	/* Clear Latched Registers (???) */
	/* regmap_write(regmap, 0x26af, 0xff); */

	// Disable QBG - This lowers ADC Reads and Reads on VMid
	regmap_write(regmap, 0x4F46, 0x00);

	qcom_pm8550b_charger_setup_jeita(chg);

	/* Enable and Clear Pause */
	charging_pause_set(chg, false);
	charging_enable_set(chg, true);

	return 0;
}

static struct pm8550b_charger1_resources pm8550b_charger1_res = {
	.irq_params = {
		{
			.irq_name = "charging-state-change",
			.virq = PM8550B_CHARGER_STATE_CHANGE_IRQ,
		},
		{
			.irq_name = "charger-error",
			.virq = PM8550B_CHARGER_CHARGING_ERROR_IRQ,
		},
	},
	.nr_irqs = 2,
};

static struct pm8550b_charger2_resources pm8550b_charger2_res = {
	.irq_params = {
		{
			.irq_name = "charging-path-change",
			.virq = PM8550B_CHARGER2_CHARGE_PATH_CHANGE_IRQ,
		},
		{
			.irq_name = "full-duty-charge-mode",
			.virq = PM8550B_CHARGER2_FULL_DUTY_CHARGE_IRQ,
		},
	},
	.nr_irqs = 2,
};

static struct pm8550b_misc_resources pm8550b_misc_res = {
	.irq_params = {
		{
			.irq_name = "aicl-fail",
			.virq = PM8550B_MISC_AICL_FAIL_IRQ,
		},
		{
			.irq_name = "aicl-done",
			.virq = PM8550B_MISC_AICL_DONE_IRQ,
		},
	},
	.nr_irqs = 2,
};

static struct pm8550b_charger_resources pm8550b_charger_res = {
	.charger1_res = &pm8550b_charger1_res,
	.charger2_res = &pm8550b_charger2_res,
	.misc_res = &pm8550b_misc_res,
};

static int qcom_pm8550b_charger_resume(struct platform_device *pdev)
{
	struct pm8550b_charger *chg = platform_get_drvdata(pdev);
	if ( chg ) {
		qcom_pm8550b_set_qph_charge_current(chg);
	} else {
		dev_err(&pdev->dev, "failed to get pm8550b chg\n");
	}
	return 0;
}

static const struct of_device_id qcom_pm8550b_charger_table[] = { { .compatible = "qcom,pm8550b-charger",
								    .data = &pm8550b_charger_res },
								  {} };
MODULE_DEVICE_TABLE(of, qcom_pm8550b_charger_table);

static struct platform_driver qcom_pm8550b_charger_driver = {
	.driver = {
		.name = "qcom,pm8550b-charger",
		.of_match_table = qcom_pm8550b_charger_table,
	},
	.probe = qcom_pm8550b_charger_probe,
	.resume = qcom_pm8550b_charger_resume,
};

module_platform_driver(qcom_pm8550b_charger_driver);

MODULE_DESCRIPTION("QCOM PMIC PM8550B Charger Driver");
MODULE_LICENSE("GPL");

// TODO Add to dtsi...
// qcom,max-voltage-uv = <8900000>;
// qcom,termchg-current-ma = <100>;

// qcom,jeita-fcc-ranges = <0  50   2500000
// 						51  400  3600000
// 						401 450  2500000>;
// qcom,jeita-fv-ranges = <0   50   4250000
// 						51  400  4350000
// 						401 450  4250000>;

// /* COOL hys = 8 DegC, WARM hys = 37 DegC */
// qcom,jeita-soft-hys-thresholds = <0x4f5e 0x2943>;
// /* COOL = 5 DegC, WARM = 40 DegC */
// qcom,jeita-soft-thresholds = <0x5314 0x25e3>;
// /* COLD = 0 DegC, HOT = 45 DegC */
// qcom,jeita-hard-thresholds = <0x58cd 0x20b8>;

// qcom,jeita-soft-fcc-ua = <2500000 2500000>;
// qcom,jeita-soft-fv-uv = <4250000 4250000>;

// TODO Add resume_noirq for aicl
