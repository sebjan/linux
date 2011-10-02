/*
 * OMAP4 Temperature sensor driver file
 *
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
 * Author: J Keerthy <j-keerthy <at> ti.com>
 * Author: Moiz Sonasath <m-sonasath <at> ti.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <plat/omap_device.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/jiffies.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/stddef.h>
#include <linux/sysfs.h>
#include <linux/err.h>
#include <linux/types.h>
#include <linux/mutex.h>
#include <linux/pm_runtime.h>
#include <linux/delay.h>
#include <plat/temperature_sensor.h>

#define TSHUT_HOT		920	/* 122 deg C */
#define TSHUT_COLD		866	/* 100 deg C */
#define T_HOT			800	/* 73 deg C */
#define T_COLD			795	/* 71 deg C */
#define OMAP_ADC_START_VALUE	530
#define OMAP_ADC_END_VALUE	923
#define MAX_FREQ		2000000
#define MIN_FREQ		1000000
#define MIN_TEMP		-40000
#define MAX_TEMP		123000
#define HYST_VAL		5000
#define NEG_HYST_VAL		-5000
/*
 * omap_temp_sensor structure
 * @hwmon_dev - hwmon device pointer
 * @pdev_dev - platform device pointer
 * @clock - Clock pointer
 * @registers - Pointer to structure with register offsets and bitfields
 * @sensor_mutex - Mutex for sysfs, irq and PM
 * @irq - MPU Irq number for thermal alert
 * @phy_base - Physical base of the temp I/O
 * @clk_rate - Holds current clock rate
 * @temp_sensor_ctrl - temp sensor control register value
 * @bg_ctrl - bandgap ctrl register value
 * @bg_counter - bandgap counter value
 * @bg_threshold - bandgap threshold register value
 * @temp_sensor_tshut_threshold - bandgap tshut register value
 * @clk_on - Manages the current clock state
 */
struct omap_temp_sensor {
	struct device		*hwmon_dev;
	struct device		*pdev_dev;
	struct clk		*clock;
	struct omap_temp_sensor_registers *registers;
	struct mutex		sensor_mutex; /* Mutex for sysfs, irq and PM */
	unsigned int		irq;
	void __iomem		*phy_base;
	u32			clk_rate;
	u32			temp_sensor_ctrl;
	u32			bg_ctrl;
	u32			bg_counter;
	u32			bg_threshold;
	u32			temp_sensor_tshut_threshold;
	bool                    clk_on;
};

/*
 * Temperature values in milli degree celsius
 * ADC code values from 530 to 923
 */
static int adc_to_temp[394] = {
	-40000, -40000, -40000, -40000, -39800, -39400, -39000, -38600, -38200,
	-37800, -37300, -36800, -36400, -36000, -35600, -35200, -34800,
	-34300, -33800, -33400, -33000, -32600, -32200, -31800, -31300,
	-30800, -30400, -30000, -29600, -29200, -28700, -28200, -27800,
	-27400, -27000, -26600, -26200, -25700, -25200, -24800, -24400,
	-24000, -23600, -23200, -22700, -22200, -21800, -21400, -21000,
	-20600, -20200, -19700, -19200, -18800, -18400, -18000, -17600,
	-17200, -16700, -16200, -15800, -15400, -15000, -14600, -14200,
	-13700, -13200, -12800, -12400, -12000, -11600, -11200, -10700,
	-10200, -9800, -9400, -9000, -8600, -8200, -7700, -7200, -6800,
	-6400, -6000, -5600, -5200, -4800, -4300, -3800, -3400, -3000,
	-2600, -2200, -1800, -1300, -800, -400, 0, 400, 800, 1200, 1600,
	2100, 2600, 3000, 3400, 3800, 4200, 4600, 5100, 5600, 6000, 6400,
	6800, 7200, 7600, 8000, 8500, 9000, 9400, 9800, 10200, 10600, 11000,
	11400, 11900, 12400, 12800, 13200, 13600, 14000, 14400, 14800,
	15300, 15800, 16200, 16600, 17000, 17400, 17800, 18200, 18700,
	19200, 19600, 20000, 20400, 20800, 21200, 21600, 22100, 22600,
	23000, 23400, 23800, 24200, 24600, 25000, 25400, 25900, 26400,
	26800, 27200, 27600, 28000, 28400, 28800, 29300, 29800, 30200,
	30600, 31000, 31400, 31800, 32200, 32600, 33100, 33600, 34000,
	34400, 34800, 35200, 35600, 36000, 36400, 36800, 37300, 37800,
	38200, 38600, 39000, 39400, 39800, 40200, 40600, 41100, 41600,
	42000, 42400, 42800, 43200, 43600, 44000, 44400, 44800, 45300,
	45800, 46200, 46600, 47000, 47400, 47800, 48200, 48600, 49000,
	49500, 50000, 50400, 50800, 51200, 51600, 52000, 52400, 52800,
	53200, 53700, 54200, 54600, 55000, 55400, 55800, 56200, 56600,
	57000, 57400, 57800, 58200, 58700, 59200, 59600, 60000, 60400,
	60800, 61200, 61600, 62000, 62400, 62800, 63300, 63800, 64200,
	64600, 65000, 65400, 65800, 66200, 66600, 67000, 67400, 67800,
	68200, 68700, 69200, 69600, 70000, 70400, 70800, 71200, 71600,
	72000, 72400, 72800, 73200, 73600, 74100, 74600, 75000, 75400,
	75800, 76200, 76600, 77000, 77400, 77800, 78200, 78600, 79000,
	79400, 79800, 80300, 80800, 81200, 81600, 82000, 82400, 82800,
	83200, 83600, 84000, 84400, 84800, 85200, 85600, 86000, 86400,
	86800, 87300, 87800, 88200, 88600, 89000, 89400, 89800, 90200,
	90600, 91000, 91400, 91800, 92200, 92600, 93000, 93400, 93800,
	94200, 94600, 95000, 95500, 96000, 96400, 96800, 97200, 97600,
	98000, 98400, 98800, 99200, 99600, 100000, 100400, 100800, 101200,
	101600, 102000, 102400, 102800, 103200, 103600, 104000, 104400,
	104800, 105200, 105600, 106100, 106600, 107000, 107400, 107800,
	108200, 108600, 109000, 109400, 109800, 110200, 110600, 111000,
	111400, 111800, 112200, 112600, 113000, 113400, 113800, 114200,
	114600, 115000, 115400, 115800, 116200, 116600, 117000, 117400,
	117800, 118200, 118600, 119000, 119400, 119800, 120200, 120600,
	121000, 121400, 121800, 122200, 122600, 123000
};

static unsigned long omap_temp_sensor_readl(struct omap_temp_sensor
					    *temp_sensor, u32 reg)
{
	return __raw_readl(temp_sensor->phy_base + reg);
}

static void omap_temp_sensor_writel(struct omap_temp_sensor *temp_sensor,
				    u32 val, u32 reg)
{
	__raw_writel(val, temp_sensor->phy_base + reg);
}

static int adc_to_temp_conversion(int adc_val)
{
	return adc_to_temp[adc_val - OMAP_ADC_START_VALUE];
}

static int temp_to_adc_conversion(long temp)
{
	int high, low, mid;

	if (temp < adc_to_temp[0] ||
		temp > adc_to_temp[OMAP_ADC_END_VALUE - OMAP_ADC_START_VALUE])
		return -EINVAL;

	high = OMAP_ADC_END_VALUE - OMAP_ADC_START_VALUE;
	low = 0;
	mid = (high + low) / 2;

	while (low < high) {
		if (temp < adc_to_temp[mid])
			high = mid - 1;
		else
			low = mid + 1;
		mid = (low + high) / 2;
	}

	return OMAP_ADC_START_VALUE + low;
}

static void omap_configure_temp_sensor_counter(struct omap_temp_sensor
					       *temp_sensor, u32 counter)
{
	u32 val;

	val = omap_temp_sensor_readl(temp_sensor,
			temp_sensor->registers->bgap_counter);
	val &= ~temp_sensor->registers->counter_mask;
	val |= counter << __ffs(temp_sensor->registers->counter_mask);
	omap_temp_sensor_writel(temp_sensor, val,
			temp_sensor->registers->bgap_counter);
}

static void omap_enable_continuous_mode(struct omap_temp_sensor *temp_sensor)
{
	u32 val;

	val = omap_temp_sensor_readl(temp_sensor,
			temp_sensor->registers->bgap_mode_ctrl);

	val |= 1 << __ffs(temp_sensor->registers->mode_ctrl_mask);

	omap_temp_sensor_writel(temp_sensor, val,
			temp_sensor->registers->bgap_mode_ctrl);
}

static void omap_temp_sensor_unmask_interrupts(struct omap_temp_sensor
						*temp_sensor, u8 hot, u8 cold)
{
	u32 reg_val;

	reg_val = omap_temp_sensor_readl(temp_sensor,
			temp_sensor->registers->bgap_mask_ctrl);
	if (hot)
		reg_val |= temp_sensor->registers->mask_hot_mask;
	else
		reg_val &= ~temp_sensor->registers->mask_hot_mask;

	if (cold)
		reg_val |= temp_sensor->registers->mask_cold_mask;
	else
		reg_val &= ~temp_sensor->registers->mask_cold_mask;

	omap_temp_sensor_writel(temp_sensor, reg_val,
				temp_sensor->registers->bgap_mask_ctrl);
}

static void add_hyst(int adc_val, int hyst_val, int *thresh_val)
{
	int temp = adc_to_temp_conversion(adc_val);

	temp += hyst_val;

	*thresh_val = temp_to_adc_conversion(temp);
}

static void omap_temp_sensor_configure_thresholds_mask(struct omap_temp_sensor
	*temp_sensor, bool set_hot, bool set_cold, int t_hot, int t_cold)
{
	int		reg_val, cold, hot, temp;

	if (set_hot) {
		/* obtain the T cold value */
		cold = omap_temp_sensor_readl(temp_sensor,
			temp_sensor->registers->bgap_threshold);
		cold = (cold & temp_sensor->registers->threshold_tcold_mask)
			>> __ffs(temp_sensor->registers->threshold_tcold_mask);

		if (t_hot < cold) {
			/* change the t_cold to t_hot - 5000 millidegrees */
			add_hyst(t_hot, NEG_HYST_VAL, &cold);
			/* write the new t_cold value */
			reg_val = omap_temp_sensor_readl(temp_sensor,
				temp_sensor->registers->bgap_threshold);
			reg_val &=
				~temp_sensor->registers->threshold_tcold_mask;
			reg_val |= cold <<
			__ffs(temp_sensor->registers->threshold_tcold_mask);
			omap_temp_sensor_writel(temp_sensor, reg_val,
			temp_sensor->registers->bgap_threshold);
			t_cold = cold;
		}

		/* write the new t_hot value */
		reg_val = omap_temp_sensor_readl(temp_sensor,
			temp_sensor->registers->bgap_threshold);
		reg_val &= ~temp_sensor->registers->threshold_thot_mask;
		reg_val |= (t_hot <<
			__ffs(temp_sensor->registers->threshold_thot_mask));
		omap_temp_sensor_writel(temp_sensor, reg_val,
			temp_sensor->registers->bgap_threshold);
	}

	if (set_cold) {
		/* obtain the T HOT value */
		hot = omap_temp_sensor_readl(temp_sensor,
			temp_sensor->registers->bgap_threshold);
		hot = (hot & temp_sensor->registers->threshold_thot_mask) >>
			__ffs(temp_sensor->registers->threshold_thot_mask);
		if (t_cold > hot) {
			/* change the t_hot to t_cold + 5000 millidegrees */
			add_hyst(t_cold, HYST_VAL, &hot);
			/* write the new t_hot value */
			reg_val = omap_temp_sensor_readl(temp_sensor,
				temp_sensor->registers->bgap_threshold);
			reg_val &=
				~temp_sensor->registers->threshold_thot_mask;
			reg_val |= (hot <<
			__ffs(temp_sensor->registers->threshold_thot_mask));
			omap_temp_sensor_writel(temp_sensor, reg_val,
				temp_sensor->registers->bgap_threshold);
			t_hot = hot;
		}

		/* write the new t_cold value */
		reg_val = omap_temp_sensor_readl(temp_sensor,
				temp_sensor->registers->bgap_threshold);
		reg_val &= ~temp_sensor->registers->threshold_tcold_mask;
		reg_val |= t_cold <<
			__ffs(temp_sensor->registers->threshold_tcold_mask);
		omap_temp_sensor_writel(temp_sensor, reg_val,
			temp_sensor->registers->bgap_threshold);
	}
	 /* obtain the T cold value */
	cold = omap_temp_sensor_readl(temp_sensor,
		temp_sensor->registers->bgap_threshold);
	cold = (cold & temp_sensor->registers->threshold_tcold_mask)
		>> __ffs(temp_sensor->registers->threshold_tcold_mask);

	/* obtain the T HOT value */
	hot = omap_temp_sensor_readl(temp_sensor,
		temp_sensor->registers->bgap_threshold);
	hot = (hot & temp_sensor->registers->threshold_thot_mask) >>
		__ffs(temp_sensor->registers->threshold_thot_mask);

	/* Read the current temperature */
	temp = omap_temp_sensor_readl(temp_sensor,
		temp_sensor->registers->temp_sensor_ctrl);
	temp &= temp_sensor->registers->bgap_dtemp_mask;

	/*
	 * If current temperature is in-between the hot and cold thresholds
	 * Enable both masks or in the init case enable both masks
	 */
	if ((temp > cold && temp < hot) || (set_cold && set_hot))
		omap_temp_sensor_unmask_interrupts(temp_sensor, 1, 1);

	/*
	 * If user sets the HIGH threshold(t_hot) greater than the current
	 * temperature(temp) unmask the HOT interrupts
	 */
	else if (hot > temp)
		omap_temp_sensor_unmask_interrupts(temp_sensor, 1, 0);

	/*
	 * If user sets the LOW threshold(t_cold) lower than the current
	 * temperature(temp) unmask the COLD interrupts
	 */
	else
		omap_temp_sensor_unmask_interrupts(temp_sensor, 0, 1);
}

/* Sysfs hook functions */

static ssize_t show_temp_max(struct device *dev,
			struct device_attribute *devattr, char *buf)
{
	struct omap_temp_sensor *temp_sensor = dev_get_drvdata(dev);
	int temp;

	temp = omap_temp_sensor_readl(temp_sensor,
			temp_sensor->registers->bgap_threshold);
	temp = (temp & temp_sensor->registers->threshold_thot_mask)
			>> __ffs(temp_sensor->registers->threshold_thot_mask);
	temp = adc_to_temp_conversion(temp);

	return snprintf(buf, 16, "%d\n", temp);
}

static ssize_t set_temp_max(struct device *dev,
			    struct device_attribute *devattr,
			    const char *buf, size_t count)
{
	struct omap_temp_sensor	*temp_sensor = dev_get_drvdata(dev);
	long			val;
	u32			t_hot;

	if (strict_strtol(buf, 10, &val))
		return -EINVAL;
	if (val < MIN_TEMP - HYST_VAL)
		return -EINVAL;

	t_hot = temp_to_adc_conversion(val);
	if (t_hot < 0)
		return t_hot;

	mutex_lock(&temp_sensor->sensor_mutex);
	omap_temp_sensor_configure_thresholds_mask(temp_sensor, 1, 0,
							t_hot, 0);
	mutex_unlock(&temp_sensor->sensor_mutex);

	return count;
}

static ssize_t show_temp_max_hyst(struct device *dev,
		struct device_attribute *devattr, char *buf)
{
	struct omap_temp_sensor *temp_sensor = dev_get_drvdata(dev);
	u32			temp;

	temp = omap_temp_sensor_readl(temp_sensor,
		temp_sensor->registers->bgap_threshold);
	temp = (temp & temp_sensor->registers->threshold_tcold_mask) >>
		__ffs(temp_sensor->registers->threshold_tcold_mask);

	if (temp < OMAP_ADC_START_VALUE || temp > OMAP_ADC_END_VALUE) {
		dev_err(dev, "invalid value\n");
		return -EIO;
	}

	temp = adc_to_temp_conversion(temp);

	return snprintf(buf, 16, "%d\n", temp);
}

static ssize_t set_temp_max_hyst(struct device *dev,
				 struct device_attribute *devattr,
				 const char *buf, size_t count)
{
	struct omap_temp_sensor		*temp_sensor = dev_get_drvdata(dev);
	u32				t_cold;
	long				val;

	if (strict_strtol(buf, 10, &val)) {
		count = -EINVAL;
		goto out;
	}

	if (val > MAX_TEMP - HYST_VAL) {
		count = -EINVAL;
		goto out;
	}

	t_cold = temp_to_adc_conversion(val);
	if (t_cold < 0)
		return t_cold;


	mutex_lock(&temp_sensor->sensor_mutex);
	omap_temp_sensor_configure_thresholds_mask(temp_sensor, 0, 1, 0,
							t_cold);
	mutex_unlock(&temp_sensor->sensor_mutex);

out:
	return count;
}

static ssize_t show_update_interval(struct device *dev,
			struct device_attribute *devattr, char *buf)
{
	struct omap_temp_sensor	*temp_sensor = dev_get_drvdata(dev);
	u32			temp;

	temp = omap_temp_sensor_readl(temp_sensor,
			temp_sensor->registers->bgap_counter);
	temp = (temp & temp_sensor->registers->counter_mask) >>
			__ffs(temp_sensor->registers->counter_mask);
	temp = temp * 1000 / temp_sensor->clk_rate;

	return sprintf(buf, "%d\n", temp);
}

static ssize_t set_update_interval(struct device *dev,
			       struct device_attribute *devattr,
			       const char *buf, size_t count)
{
	struct omap_temp_sensor	*temp_sensor = dev_get_drvdata(dev);
	long			val;

	if (strict_strtol(buf, 10, &val)) {
		count = -EINVAL;
		goto out;
	}

	if (val < 0)
		return -EINVAL;
	val *= temp_sensor->clk_rate / 1000;
	mutex_lock(&temp_sensor->sensor_mutex);
	omap_configure_temp_sensor_counter(temp_sensor, val);
	mutex_unlock(&temp_sensor->sensor_mutex);
out:
	return count;
}

static int omap_temp_sensor_read_temp(struct device *dev,
				      struct device_attribute *devattr,
				      char *buf)
{
	struct omap_temp_sensor	*temp_sensor = dev_get_drvdata(dev);
	int			temp;

	temp = omap_temp_sensor_readl(temp_sensor,
		temp_sensor->registers->temp_sensor_ctrl);
	temp &= temp_sensor->registers->bgap_dtemp_mask;

	/* look up for temperature in the table and return the temperature */
	if (temp < OMAP_ADC_START_VALUE || temp > OMAP_ADC_END_VALUE)
		return -EIO;

	temp = adc_to_temp[temp - OMAP_ADC_START_VALUE];

	return sprintf(buf, "%d\n", temp);
}

static SENSOR_DEVICE_ATTR(temp1_input, S_IRUGO, omap_temp_sensor_read_temp,
			NULL, 0);
static SENSOR_DEVICE_ATTR(temp1_max, S_IWUSR | S_IRUGO, show_temp_max,
			set_temp_max, 0);
static SENSOR_DEVICE_ATTR(temp1_max_hyst, S_IWUSR | S_IRUGO, show_temp_max_hyst,
			set_temp_max_hyst, 0);
static SENSOR_DEVICE_ATTR(update_interval, S_IWUSR | S_IRUGO,
			show_update_interval, set_update_interval, 0);

static struct attribute *omap_temp_sensor_attributes[] = {
	&sensor_dev_attr_temp1_input.dev_attr.attr,
	&sensor_dev_attr_temp1_max.dev_attr.attr,
	&sensor_dev_attr_temp1_max_hyst.dev_attr.attr,
	&sensor_dev_attr_update_interval.dev_attr.attr,
	NULL
};

static const struct attribute_group omap_temp_sensor_group = {
	.attrs = omap_temp_sensor_attributes,
};

static int omap_temp_sensor_clk_enable(struct omap_temp_sensor *temp_sensor)
{
	u32 ret = 0;

	if (temp_sensor->clk_on) {
		dev_err(temp_sensor->pdev_dev, "clock already on\n");
		goto out;
	}

	ret = pm_runtime_get_sync(temp_sensor->pdev_dev);
	if (ret < 0) {
		dev_err(temp_sensor->pdev_dev, "get sync failed\n");
		goto out;
	}

	temp_sensor->clk_on = 1;

out:
	return ret;
}

static void omap_temp_sensor_clk_disable(struct omap_temp_sensor *temp_sensor)
{
	/* Gate the clock */
	pm_runtime_put_sync(temp_sensor->hwmon_dev);
	temp_sensor->clk_on = 0;
}

static irqreturn_t omap_talert_irq_handler(int irq, void *data)
{
	struct omap_temp_sensor		*temp_sensor;
	int				t_hot, t_cold, temp;

	temp_sensor = data;
	mutex_lock(&temp_sensor->sensor_mutex);
	/* Read the status of t_hot */
	t_hot = omap_temp_sensor_readl(temp_sensor,
			temp_sensor->registers->bgap_status)
			& temp_sensor->registers->status_hot_mask;

	/* Read the status of t_cold */
	t_cold = omap_temp_sensor_readl(temp_sensor,
			temp_sensor->registers->bgap_status)
			& temp_sensor->registers->status_cold_mask;

	temp = omap_temp_sensor_readl(temp_sensor,
			temp_sensor->registers->bgap_mask_ctrl);
	/*
	 * One TALERT interrupt: Two sources
	 * If the interrupt is due to t_hot then mask t_hot and
	 * and unmask t_cold else mask t_cold and unmask t_hot
	 */
	if (t_hot) {
		temp &= ~temp_sensor->registers->mask_hot_mask;
		temp |= temp_sensor->registers->mask_cold_mask;
	} else if (t_cold) {
		temp &= ~temp_sensor->registers->mask_cold_mask;
		temp |= temp_sensor->registers->mask_hot_mask;
	}

	omap_temp_sensor_writel(temp_sensor, temp,
		temp_sensor->registers->bgap_mask_ctrl);

	if (temp_sensor->hwmon_dev)
		/* kobject_uvent to user space telling threshold crossed */
		kobject_uevent(&temp_sensor->hwmon_dev->kobj, KOBJ_CHANGE);

	mutex_unlock(&temp_sensor->sensor_mutex);

	return IRQ_HANDLED;
}

static int __devinit omap_temp_sensor_probe(struct platform_device *pdev)
{
	struct omap_temp_sensor_pdata	*pdata	= pdev->dev.platform_data;
	struct omap_temp_sensor		*temp_sensor;
	struct resource			*mem;
	int				ret = 0;
	int				clk_rate;
	u32				max_freq, min_freq;

	if (!pdata) {
		dev_err(&pdev->dev, "platform data missing\n");
		return -EINVAL;
	}

	temp_sensor = kzalloc(sizeof(*temp_sensor), GFP_KERNEL);
	if (!temp_sensor) {
		dev_err(&pdev->dev, "Memory allocation failed\n");
		return -ENOMEM;
	}

	mutex_init(&temp_sensor->sensor_mutex);

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem) {
		dev_err(&pdev->dev, "no mem resource\n");
		ret = -ENOMEM;
		goto plat_res_err;
	}

	temp_sensor->irq = platform_get_irq_byname(pdev, "thermal_alert");
	if (temp_sensor->irq < 0) {
		dev_err(&pdev->dev, "get_irq_byname failed\n");
		ret = temp_sensor->irq;
		goto plat_res_err;
	}

	temp_sensor->phy_base = ioremap(mem->start, resource_size(mem));
	if (!temp_sensor->phy_base) {
		dev_err(&pdev->dev, "ioremap failed\n");
		ret = -ENOMEM;
		goto plat_res_err;
	}

	temp_sensor->clock = NULL;
	temp_sensor->registers = pdata->registers;
	temp_sensor->pdev_dev = &pdev->dev;

	if (pdata->max_freq && pdata->min_freq) {
		max_freq = pdata->max_freq;
		min_freq = pdata->min_freq;
	} else {
		max_freq = MAX_FREQ;
		min_freq = MIN_FREQ;
	}

	pm_runtime_enable(&pdev->dev);
	pm_runtime_irq_safe(&pdev->dev);

	/*
	 * check if the efuse has a non-zero value if not
	 * it is an untrimmed sample and the temperatures
	 * may not be accurate
	 */

	if (omap_temp_sensor_readl(temp_sensor,
			temp_sensor->registers->bgap_efuse))
		dev_info(&pdev->dev,
			"Invalid EFUSE, Non-trimmed BGAP, Temp not accurate\n");

	dev_set_drvdata(&pdev->dev, temp_sensor);
	temp_sensor->clock = clk_get(&pdev->dev, "fck");
	if (IS_ERR(temp_sensor->clock)) {
		ret = PTR_ERR(temp_sensor->clock);
		dev_err(&pdev->dev,
			"unable to get fclk: %d\n", ret);
		goto plat_res_err;
	}

	ret = omap_temp_sensor_clk_enable(temp_sensor);
	if (ret)
		goto clken_err;

	clk_rate = clk_round_rate(temp_sensor->clock, max_freq);
	if (clk_rate < min_freq || clk_rate == 0xffffffff) {
		ret = -ENODEV;
		goto clken_err;
	}

	ret = clk_set_rate(temp_sensor->clock, clk_rate);
	if (ret) {
		dev_err(&pdev->dev, "Cannot set clock rate\n");
		goto clken_err;
	}

	temp_sensor->clk_rate = clk_rate;
	omap_enable_continuous_mode(temp_sensor);
	/* 1 clk cycle */
	omap_configure_temp_sensor_counter(temp_sensor, 1);

	/* Wait till the first conversion is done wait for at least 1ms */
	usleep_range(1000, 2000);

	/* Read the temperature once due to hw issue*/
	omap_temp_sensor_readl(temp_sensor,
			temp_sensor->registers->temp_sensor_ctrl);

	/* Set 2 seconds time as default counter */
	omap_configure_temp_sensor_counter(temp_sensor,
						temp_sensor->clk_rate * 2);

	ret = request_threaded_irq(temp_sensor->irq, NULL,
		omap_talert_irq_handler, IRQF_TRIGGER_RISING | IRQF_ONESHOT,
			"temp_sensor", temp_sensor);
	if (ret) {
		dev_err(&pdev->dev, "Request threaded irq failed.\n");
		goto req_irq_err;
	}

	ret = sysfs_create_group(&pdev->dev.kobj, &omap_temp_sensor_group);
	if (ret) {
		dev_err(&pdev->dev, "could not create sysfs files\n");
		goto sysfs_create_err;
	}

	temp_sensor->hwmon_dev = hwmon_device_register(&pdev->dev);
	if (IS_ERR(temp_sensor->hwmon_dev)) {
		dev_err(&pdev->dev, "hwmon_device_register failed.\n");
		ret = PTR_ERR(temp_sensor->hwmon_dev);
		goto hwmon_reg_err;
	}

	mutex_lock(&temp_sensor->sensor_mutex);
	omap_temp_sensor_configure_thresholds_mask(temp_sensor, 1, 1,
						T_HOT, T_COLD);
	mutex_unlock(&temp_sensor->sensor_mutex);

	return 0;

hwmon_reg_err:
	sysfs_remove_group(&temp_sensor->hwmon_dev->kobj,
				&omap_temp_sensor_group);
sysfs_create_err:
	free_irq(temp_sensor->irq, temp_sensor);
req_irq_err:
	omap_temp_sensor_clk_disable(temp_sensor);
clken_err:
	clk_put(temp_sensor->clock);
	iounmap(temp_sensor->phy_base);
plat_res_err:
	dev_set_drvdata(&pdev->dev, NULL);
	mutex_destroy(&temp_sensor->sensor_mutex);
	kfree(temp_sensor);
	return ret;
}

static int __devexit omap_temp_sensor_remove(struct platform_device *pdev)
{
	struct omap_temp_sensor *temp_sensor = platform_get_drvdata(pdev);

	hwmon_device_unregister(&pdev->dev);
	sysfs_remove_group(&temp_sensor->hwmon_dev->kobj,
			&omap_temp_sensor_group);
	free_irq(temp_sensor->irq, temp_sensor);
	omap_temp_sensor_clk_disable(temp_sensor);
	clk_put(temp_sensor->clock);
	iounmap(temp_sensor->phy_base);
	dev_set_drvdata(&pdev->dev, NULL);
	mutex_destroy(&temp_sensor->sensor_mutex);
	kfree(temp_sensor);

	return 0;
}

#ifdef CONFIG_PM
static void omap_temp_sensor_save_ctxt(struct omap_temp_sensor *temp_sensor)
{
	temp_sensor->temp_sensor_ctrl = omap_temp_sensor_readl(temp_sensor,
		temp_sensor->registers->temp_sensor_ctrl);
	temp_sensor->bg_ctrl = omap_temp_sensor_readl(temp_sensor,
		temp_sensor->registers->bgap_mask_ctrl);
	temp_sensor->bg_counter = omap_temp_sensor_readl(temp_sensor,
		temp_sensor->registers->bgap_counter);
	temp_sensor->bg_threshold = omap_temp_sensor_readl(temp_sensor,
		temp_sensor->registers->bgap_threshold);
	temp_sensor->temp_sensor_tshut_threshold =
		omap_temp_sensor_readl(temp_sensor,
		temp_sensor->registers->thsut_threshold);
}

static void omap_temp_sensor_restore_ctxt(struct omap_temp_sensor *temp_sensor)
{
	omap_temp_sensor_writel(temp_sensor,
				temp_sensor->temp_sensor_ctrl,
				temp_sensor->registers->temp_sensor_ctrl);
	omap_temp_sensor_writel(temp_sensor,
				temp_sensor->bg_ctrl,
				temp_sensor->registers->bgap_mask_ctrl);
	omap_temp_sensor_writel(temp_sensor,
				temp_sensor->bg_counter,
				temp_sensor->registers->bgap_counter);
	omap_temp_sensor_writel(temp_sensor,
				temp_sensor->bg_threshold,
				temp_sensor->registers->bgap_threshold);
	omap_temp_sensor_writel(temp_sensor,
				temp_sensor->temp_sensor_tshut_threshold,
				temp_sensor->registers->thsut_threshold);
}

static int omap_temp_sensor_suspend(struct device *dev)
{
	struct omap_temp_sensor *temp_sensor = dev_get_drvdata(dev);

	omap_temp_sensor_save_ctxt(temp_sensor);

	return 0;
}

static int omap_temp_sensor_resume(struct device *dev)
{
	struct omap_temp_sensor *temp_sensor = dev_get_drvdata(dev);

	omap_temp_sensor_restore_ctxt(temp_sensor);

	return 0;
}

static int omap_temp_sensor_runtime_suspend(struct device *dev)
{
	struct omap_temp_sensor *temp_sensor = dev_get_drvdata(dev);

	omap_temp_sensor_save_ctxt(temp_sensor);

	return 0;
}

static int omap_temp_sensor_runtime_resume(struct device *dev)
{
	static int context_loss_count;
	int temp;
	struct omap_temp_sensor *temp_sensor = dev_get_drvdata(dev);

	temp = omap_device_get_context_loss_count(to_platform_device(dev));

	if (temp != context_loss_count && context_loss_count != 0)
		omap_temp_sensor_restore_ctxt(temp_sensor);

	context_loss_count = temp;

	return 0;
}

static int omap_temp_sensor_idle(struct device *dev)
{
	return 0;
}

static const struct dev_pm_ops omap_temp_sensor_dev_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(omap_temp_sensor_suspend,
			omap_temp_sensor_resume)
	SET_RUNTIME_PM_OPS(omap_temp_sensor_runtime_suspend,
			omap_temp_sensor_runtime_resume, omap_temp_sensor_idle)
};

#define DEV_PM_OPS	(&omap_temp_sensor_dev_pm_ops)
#else
#define DEV_PM_OPS	NULL
#endif

static struct platform_driver omap_temp_sensor_driver = {
	.probe = omap_temp_sensor_probe,
	.remove = omap_temp_sensor_remove,
	.driver = {
			.name = "omap_temp_sensor",
			.pm = DEV_PM_OPS,
		  },
};

int __init omap_temp_sensor_init(void)
{
	return platform_driver_register(&omap_temp_sensor_driver);
}
module_init(omap_temp_sensor_init);

static void __exit omap_temp_sensor_exit(void)
{
	platform_driver_unregister(&omap_temp_sensor_driver);
}
module_exit(omap_temp_sensor_exit);

MODULE_DESCRIPTION("OMAP446X temperature sensor Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRIVER_NAME);
MODULE_AUTHOR("J Keerthy <j-keerthy@ti.com>");
