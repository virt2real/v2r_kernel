/*
 * DaVinci Power Management and Real Time Clock Driver for TI platforms
 *
 * Copyright (C) 2009 Texas Instruments, Inc
 *
 * Author: Miguel Aguilar <miguel.aguilar@ridgerun.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/rtc.h>
#include <linux/bcd.h>
#include <linux/platform_device.h>
#include <linux/io.h>

/*
 * The DaVinci RTC is a simple RTC with the following
 * Sec: 0 - 59 : BCD count
 * Min: 0 - 59 : BCD count
 * Hour: 0 - 23 : BCD count
 * Day: 0 - 0x7FFF(32767) : Binary count ( Over 89 years )
 */

/* PRTC interface registers */
#define DAVINCI_PRTCIF_PID			0x00
#define DAVINCI_PRTCIF_CTLR			0x04
#define DAVINCI_PRTCIF_LDATA			0x08
#define DAVINCI_PRTCIF_UDATA			0x0C
#define DAVINCI_PRTCIF_INTEN			0x10
#define DAVINCI_PRTCIF_INTFLG			0x14

/* DAVINCI_PRTCIF_CTLR bit fields */
#define DAVINCI_PRTCIF_CTLR_BUSY		BIT(31)
#define DAVINCI_PRTCIF_CTLR_SIZE		BIT(25)
#define DAVINCI_PRTCIF_CTLR_DIR			BIT(24)
#define DAVINCI_PRTCIF_CTLR_BENU_MSB		BIT(23)
#define DAVINCI_PRTCIF_CTLR_BENU_3RD_BYTE	BIT(22)
#define DAVINCI_PRTCIF_CTLR_BENU_2ND_BYTE	BIT(21)
#define DAVINCI_PRTCIF_CTLR_BENU_LSB		BIT(20)
#define DAVINCI_PRTCIF_CTLR_BENU_MASK		(0x00F00000)
#define DAVINCI_PRTCIF_CTLR_BENL_MSB		BIT(19)
#define DAVINCI_PRTCIF_CTLR_BENL_3RD_BYTE	BIT(18)
#define DAVINCI_PRTCIF_CTLR_BENL_2ND_BYTE	BIT(17)
#define DAVINCI_PRTCIF_CTLR_BENL_LSB		BIT(16)
#define DAVINCI_PRTCIF_CTLR_BENL_MASK		(0x000F0000)

/* DAVINCI_PRTCIF_INTEN bit fields */
#define DAVINCI_PRTCIF_INTEN_RTCSS		BIT(1)
#define DAVINCI_PRTCIF_INTEN_RTCIF		BIT(0)
#define DAVINCI_PRTCIF_INTEN_MASK		(DAVINCI_PRTCIF_INTEN_RTCSS \
						| DAVINCI_PRTCIF_INTEN_RTCIF)

/* DAVINCI_PRTCIF_INTFLG bit fields */
#define DAVINCI_PRTCIF_INTFLG_RTCSS		BIT(1)
#define DAVINCI_PRTCIF_INTFLG_RTCIF		BIT(0)
#define DAVINCI_PRTCIF_INTFLG_MASK		(DAVINCI_PRTCIF_INTFLG_RTCSS \
						| DAVINCI_PRTCIF_INTFLG_RTCIF)

/* PRTC subsystem registers */
#define DAVINCI_PRTCSS_RTC_INTC_EXTENA1		(0x0C)
#define DAVINCI_PRTCSS_RTC_CTRL			(0x10)
#define DAVINCI_PRTCSS_RTC_WDT			(0x11)
#define DAVINCI_PRTCSS_RTC_TMR0			(0x12)
#define DAVINCI_PRTCSS_RTC_TMR1			(0x13)
#define DAVINCI_PRTCSS_RTC_CCTRL		(0x14)
#define DAVINCI_PRTCSS_RTC_SEC			(0x15)
#define DAVINCI_PRTCSS_RTC_MIN			(0x16)
#define DAVINCI_PRTCSS_RTC_HOUR			(0x17)
#define DAVINCI_PRTCSS_RTC_DAY0			(0x18)
#define DAVINCI_PRTCSS_RTC_DAY1			(0x19)
#define DAVINCI_PRTCSS_RTC_AMIN			(0x1A)
#define DAVINCI_PRTCSS_RTC_AHOUR		(0x1B)
#define DAVINCI_PRTCSS_RTC_ADAY0		(0x1C)
#define DAVINCI_PRTCSS_RTC_ADAY1		(0x1D)
#define DAVINCI_PRTCSS_RTC_CLKC_CNT		(0x20)

/* DAVINCI_PRTCSS_RTC_INTC_EXTENA1 */
#define DAVINCI_PRTCSS_RTC_INTC_EXTENA1_MASK	(0x07)

/* DAVINCI_PRTCSS_RTC_CTRL bit fields */
#define DAVINCI_PRTCSS_RTC_CTRL_WDTBUS		BIT(7)
#define DAVINCI_PRTCSS_RTC_CTRL_WEN		BIT(6)
#define DAVINCI_PRTCSS_RTC_CTRL_WDRT		BIT(5)
#define DAVINCI_PRTCSS_RTC_CTRL_WDTFLG		BIT(4)
#define DAVINCI_PRTCSS_RTC_CTRL_TE		BIT(3)
#define DAVINCI_PRTCSS_RTC_CTRL_TIEN		BIT(2)
#define DAVINCI_PRTCSS_RTC_CTRL_TMRFLG		BIT(1)
#define DAVINCI_PRTCSS_RTC_CTRL_TMMD		BIT(0)

/* DAVINCI_PRTCSS_RTC_CCTRL bit fields */
#define DAVINCI_PRTCSS_RTC_CCTRL_CALBUSY	BIT(7)
#define DAVINCI_PRTCSS_RTC_CCTRL_DAEN		BIT(5)
#define DAVINCI_PRTCSS_RTC_CCTRL_HAEN		BIT(4)
#define DAVINCI_PRTCSS_RTC_CCTRL_MAEN		BIT(3)
#define DAVINCI_PRTCSS_RTC_CCTRL_ALMFLG		BIT(2)
#define DAVINCI_PRTCSS_RTC_CCTRL_AIEN		BIT(1)
#define DAVINCI_PRTCSS_RTC_CCTRL_CAEN		BIT(0)

static DEFINE_SPINLOCK(davinci_rtc_lock);

struct davinci_rtc {
	struct rtc_device 		*rtc;
	void __iomem			*base;
	resource_size_t			pbase;
	size_t				base_size;
	int				irq;
};

static inline void davinci_rtcif_write(struct davinci_rtc *davinci_rtc,
				       u32 val, u32 addr)
{
	writel(val, davinci_rtc->base + addr);
}

static inline u32 davinci_rtcif_read(struct davinci_rtc *davinci_rtc, u32 addr)
{
	return readl(davinci_rtc->base + addr);
}

static inline void davinci_rtcif_wait(struct davinci_rtc *davinci_rtc)
{
    while (davinci_rtcif_read(davinci_rtc, DAVINCI_PRTCIF_CTLR) &
	       DAVINCI_PRTCIF_CTLR_BUSY)
		cpu_relax();
}

static inline void davinci_rtcss_write(struct davinci_rtc *davinci_rtc,
				unsigned long val, u8 addr)
{
	davinci_rtcif_wait(davinci_rtc);

	davinci_rtcif_write(davinci_rtc, DAVINCI_PRTCIF_CTLR_BENL_LSB | addr,
			    DAVINCI_PRTCIF_CTLR);
	davinci_rtcif_write(davinci_rtc, val, DAVINCI_PRTCIF_LDATA);

	davinci_rtcif_wait(davinci_rtc);
}

static inline u8 davinci_rtcss_read(struct davinci_rtc *davinci_rtc, u8 addr)
{
	davinci_rtcif_wait(davinci_rtc);

	davinci_rtcif_write(davinci_rtc, DAVINCI_PRTCIF_CTLR_DIR |
			    DAVINCI_PRTCIF_CTLR_BENL_LSB |
			    addr, DAVINCI_PRTCIF_CTLR);

	davinci_rtcif_wait(davinci_rtc);

	return davinci_rtcif_read(davinci_rtc, DAVINCI_PRTCIF_LDATA);
}

static inline void davinci_rtcss_calendar_wait(struct davinci_rtc *davinci_rtc)
{
	while (davinci_rtcss_read(davinci_rtc, DAVINCI_PRTCSS_RTC_CCTRL) &
	       DAVINCI_PRTCSS_RTC_CCTRL_CALBUSY)
		cpu_relax();
}

static irqreturn_t davinci_rtc_interrupt(int irq, void *class_dev)
{
	struct davinci_rtc *davinci_rtc = class_dev;
	unsigned long events = 0;
	u32 irq_flg;
	u8 alm_irq, tmr_irq;
	u8 rtc_ctrl, rtc_cctrl;
	int ret = IRQ_NONE;

	irq_flg = davinci_rtcif_read(davinci_rtc, DAVINCI_PRTCIF_INTFLG) &
		  DAVINCI_PRTCIF_INTFLG_RTCSS;

	alm_irq = davinci_rtcss_read(davinci_rtc, DAVINCI_PRTCSS_RTC_CCTRL) &
		  DAVINCI_PRTCSS_RTC_CCTRL_ALMFLG;

	tmr_irq = davinci_rtcss_read(davinci_rtc, DAVINCI_PRTCSS_RTC_CTRL) &
		  DAVINCI_PRTCSS_RTC_CTRL_TMRFLG;

	if (irq_flg) {
		if (alm_irq) {
			events |= RTC_IRQF | RTC_AF;
			rtc_cctrl = davinci_rtcss_read(davinci_rtc,
						    DAVINCI_PRTCSS_RTC_CCTRL);
			rtc_cctrl |=  DAVINCI_PRTCSS_RTC_CCTRL_ALMFLG;
			davinci_rtcss_write(davinci_rtc, rtc_cctrl,
					    DAVINCI_PRTCSS_RTC_CCTRL);
		} else if (tmr_irq) {
			events |= RTC_IRQF | RTC_PF;
			rtc_ctrl = davinci_rtcss_read(davinci_rtc,
						      DAVINCI_PRTCSS_RTC_CTRL);
			rtc_ctrl |=  DAVINCI_PRTCSS_RTC_CTRL_TMRFLG;
			davinci_rtcss_write(davinci_rtc, rtc_ctrl,
					    DAVINCI_PRTCSS_RTC_CTRL);
		}

		davinci_rtcif_write(davinci_rtc, DAVINCI_PRTCIF_INTFLG_RTCSS,
				    DAVINCI_PRTCIF_INTFLG);
		rtc_update_irq(davinci_rtc->rtc, 1, events);
		ret = IRQ_HANDLED;
	}

	return ret;
}

static int
davinci_rtc_ioctl(struct device *dev, unsigned int cmd, unsigned long arg)
{
	struct davinci_rtc *davinci_rtc = dev_get_drvdata(dev);
	u8 rtc_ctrl;
	unsigned long flags;
	int ret = 0;

	spin_lock_irqsave(&davinci_rtc_lock, flags);

	rtc_ctrl = davinci_rtcss_read(davinci_rtc, DAVINCI_PRTCSS_RTC_CTRL);

	switch (cmd) {
	case RTC_WIE_ON:
		rtc_ctrl |= (DAVINCI_PRTCSS_RTC_CTRL_WEN |
			    DAVINCI_PRTCSS_RTC_CTRL_WDTFLG);
		break;
	case RTC_WIE_OFF:
		rtc_ctrl &= ~DAVINCI_PRTCSS_RTC_CTRL_WEN;
		break;
	case RTC_UIE_OFF:
	case RTC_UIE_ON:
		ret = -ENOTTY;
		break;
	default:
		ret = -ENOIOCTLCMD;
	}

	davinci_rtcss_write(davinci_rtc, rtc_ctrl, DAVINCI_PRTCSS_RTC_CTRL);

	spin_unlock_irqrestore(&davinci_rtc_lock, flags);

	return ret;
}

static int convertfromdays(u16 days, struct rtc_time *tm)
{
	int tmp_days, year, mon;

	for (year = 2000;; year++) {
		tmp_days = rtc_year_days(1, 12, year);
		if (days >= tmp_days)
			days -= tmp_days;
		else {
			for (mon = 0;; mon++) {
				tmp_days = rtc_month_days(mon, year);
				if (days >= tmp_days) {
					days -= tmp_days;
				} else {
					tm->tm_year = year - 1900;
					tm->tm_mon = mon;
					tm->tm_mday = days + 1;
					break;
				}
			}
			break;
		}
	}
	return 0;
}

static int convert2days(u16 *days, struct rtc_time *tm)
{
	int i;
	*days = 0;

	/* epoch == 1900 */
	if (tm->tm_year < 100 || tm->tm_year > 199)
		return -EINVAL;

	for (i = 2000; i < 1900 + tm->tm_year; i++)
		*days += rtc_year_days(1, 12, i);

	*days += rtc_year_days(tm->tm_mday, tm->tm_mon, 1900 + tm->tm_year);

	return 0;
}

static int davinci_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	struct davinci_rtc *davinci_rtc = dev_get_drvdata(dev);
	u16 days = 0;
	u8 day0, day1;
	unsigned long flags;

	spin_lock_irqsave(&davinci_rtc_lock, flags);

	davinci_rtcss_calendar_wait(davinci_rtc);
	tm->tm_sec = bcd2bin(davinci_rtcss_read(davinci_rtc,
						DAVINCI_PRTCSS_RTC_SEC));

	davinci_rtcss_calendar_wait(davinci_rtc);
	tm->tm_min = bcd2bin(davinci_rtcss_read(davinci_rtc,
						DAVINCI_PRTCSS_RTC_MIN));

	davinci_rtcss_calendar_wait(davinci_rtc);
	tm->tm_hour = bcd2bin(davinci_rtcss_read(davinci_rtc,
						 DAVINCI_PRTCSS_RTC_HOUR));

	davinci_rtcss_calendar_wait(davinci_rtc);
	day0 = davinci_rtcss_read(davinci_rtc, DAVINCI_PRTCSS_RTC_DAY0);

	davinci_rtcss_calendar_wait(davinci_rtc);
	day1 = davinci_rtcss_read(davinci_rtc, DAVINCI_PRTCSS_RTC_DAY1);

	spin_unlock_irqrestore(&davinci_rtc_lock, flags);

	days |= day1;
	days <<= 8;
	days |= day0;

	if (convertfromdays(days, tm) < 0)
		return -EINVAL;

	return 0;
}

static int davinci_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	struct davinci_rtc *davinci_rtc = dev_get_drvdata(dev);
	u16 days;
	u8 rtc_cctrl;
	unsigned long flags;

	if (convert2days(&days, tm) < 0)
		return -EINVAL;

	spin_lock_irqsave(&davinci_rtc_lock, flags);

	davinci_rtcss_calendar_wait(davinci_rtc);
	davinci_rtcss_write(davinci_rtc, bin2bcd(tm->tm_sec),
			    DAVINCI_PRTCSS_RTC_SEC);

	davinci_rtcss_calendar_wait(davinci_rtc);
	davinci_rtcss_write(davinci_rtc, bin2bcd(tm->tm_min),
			    DAVINCI_PRTCSS_RTC_MIN);

	davinci_rtcss_calendar_wait(davinci_rtc);
	davinci_rtcss_write(davinci_rtc, bin2bcd(tm->tm_hour),
			    DAVINCI_PRTCSS_RTC_HOUR);

	davinci_rtcss_calendar_wait(davinci_rtc);
	davinci_rtcss_write(davinci_rtc, days & 0xFF,
			    DAVINCI_PRTCSS_RTC_DAY0);

	davinci_rtcss_calendar_wait(davinci_rtc);
	davinci_rtcss_write(davinci_rtc, (days & 0xFF00) >> 8,
			    DAVINCI_PRTCSS_RTC_DAY1);

	rtc_cctrl = davinci_rtcss_read(davinci_rtc, DAVINCI_PRTCSS_RTC_CCTRL);

	rtc_cctrl |= DAVINCI_PRTCSS_RTC_CCTRL_CAEN;

	davinci_rtcss_write(davinci_rtc, rtc_cctrl, DAVINCI_PRTCSS_RTC_CCTRL);

	spin_unlock_irqrestore(&davinci_rtc_lock, flags);

	return 0;
}

static int davinci_rtc_alarm_irq_enable(struct device *dev,
					unsigned int enabled)
{
	struct davinci_rtc *davinci_rtc = dev_get_drvdata(dev);
	unsigned long flags;
	u8 rtc_cctrl = davinci_rtcss_read(davinci_rtc,
					  DAVINCI_PRTCSS_RTC_CCTRL);

	spin_lock_irqsave(&davinci_rtc_lock, flags);

	if (enabled)
		rtc_cctrl |= (DAVINCI_PRTCSS_RTC_CCTRL_DAEN |
			      DAVINCI_PRTCSS_RTC_CCTRL_HAEN |
			      DAVINCI_PRTCSS_RTC_CCTRL_MAEN |
			      DAVINCI_PRTCSS_RTC_CCTRL_ALMFLG |
			      DAVINCI_PRTCSS_RTC_CCTRL_AIEN);
	else
		rtc_cctrl &= ~DAVINCI_PRTCSS_RTC_CCTRL_AIEN;

	davinci_rtcss_calendar_wait(davinci_rtc);
	davinci_rtcss_write(davinci_rtc, rtc_cctrl, DAVINCI_PRTCSS_RTC_CCTRL);

	spin_unlock_irqrestore(&davinci_rtc_lock, flags);

	return 0;
}

static int davinci_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *alm)
{
	struct davinci_rtc *davinci_rtc = dev_get_drvdata(dev);
	u16 days = 0;
	u8 day0, day1;
	unsigned long flags;

	spin_lock_irqsave(&davinci_rtc_lock, flags);

	davinci_rtcss_calendar_wait(davinci_rtc);
	alm->time.tm_min = bcd2bin(davinci_rtcss_read(davinci_rtc,
				   DAVINCI_PRTCSS_RTC_AMIN));

	davinci_rtcss_calendar_wait(davinci_rtc);
	alm->time.tm_hour = bcd2bin(davinci_rtcss_read(davinci_rtc,
				    DAVINCI_PRTCSS_RTC_AHOUR));

	davinci_rtcss_calendar_wait(davinci_rtc);
	day0 = davinci_rtcss_read(davinci_rtc, DAVINCI_PRTCSS_RTC_ADAY0);

	davinci_rtcss_calendar_wait(davinci_rtc);
	day1 = davinci_rtcss_read(davinci_rtc, DAVINCI_PRTCSS_RTC_ADAY1);

	spin_unlock_irqrestore(&davinci_rtc_lock, flags);
	days |= day1;
	days <<= 8;
	days |= day0;

	if (convertfromdays(days, &alm->time) < 0)
		return -EINVAL;

	alm->pending = !!(davinci_rtcss_read(davinci_rtc,
			  DAVINCI_PRTCSS_RTC_CCTRL) &
			DAVINCI_PRTCSS_RTC_CCTRL_AIEN);
	alm->enabled = alm->pending && device_may_wakeup(dev);

	return 0;
}

static int davinci_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alm)
{
	struct davinci_rtc *davinci_rtc = dev_get_drvdata(dev);
	unsigned long flags;
	u16 days;

	if (alm->time.tm_mday <= 0 && alm->time.tm_mon < 0
	    && alm->time.tm_year < 0) {
		struct rtc_time tm;
		unsigned long now, then;

		davinci_rtc_read_time(dev, &tm);
		rtc_tm_to_time(&tm, &now);

		alm->time.tm_mday = tm.tm_mday;
		alm->time.tm_mon = tm.tm_mon;
		alm->time.tm_year = tm.tm_year;
		rtc_tm_to_time(&alm->time, &then);

		if (then < now) {
			rtc_time_to_tm(now + 24 * 60 * 60, &tm);
			alm->time.tm_mday = tm.tm_mday;
			alm->time.tm_mon = tm.tm_mon;
			alm->time.tm_year = tm.tm_year;
		}
	}

	if (convert2days(&days, &alm->time) < 0)
		return -EINVAL;

	spin_lock_irqsave(&davinci_rtc_lock, flags);

	davinci_rtcss_calendar_wait(davinci_rtc);
	davinci_rtcss_write(davinci_rtc, bin2bcd(alm->time.tm_min),
			    DAVINCI_PRTCSS_RTC_AMIN);

	davinci_rtcss_calendar_wait(davinci_rtc);
	davinci_rtcss_write(davinci_rtc, bin2bcd(alm->time.tm_hour),
			    DAVINCI_PRTCSS_RTC_AHOUR);

	davinci_rtcss_calendar_wait(davinci_rtc);
	davinci_rtcss_write(davinci_rtc, days & 0xFF,
			    DAVINCI_PRTCSS_RTC_ADAY0);

	davinci_rtcss_calendar_wait(davinci_rtc);
	davinci_rtcss_write(davinci_rtc, (days & 0xFF00) >> 8,
			    DAVINCI_PRTCSS_RTC_ADAY1);

	spin_unlock_irqrestore(&davinci_rtc_lock, flags);

	return 0;
}

static int davinci_rtc_irq_set_state(struct device *dev, int enabled)
{
	struct davinci_rtc *davinci_rtc = dev_get_drvdata(dev);
	unsigned long flags;
	u8 rtc_ctrl;

	spin_lock_irqsave(&davinci_rtc_lock, flags);

	rtc_ctrl = davinci_rtcss_read(davinci_rtc, DAVINCI_PRTCSS_RTC_CTRL);

	if (enabled) {
		while (davinci_rtcss_read(davinci_rtc, DAVINCI_PRTCSS_RTC_CTRL)
		       & DAVINCI_PRTCSS_RTC_CTRL_WDTBUS)
			cpu_relax();

		rtc_ctrl |= DAVINCI_PRTCSS_RTC_CTRL_TE;
		davinci_rtcss_write(davinci_rtc, rtc_ctrl,
				    DAVINCI_PRTCSS_RTC_CTRL);

		davinci_rtcss_write(davinci_rtc, 0x0,
				    DAVINCI_PRTCSS_RTC_CLKC_CNT);
		rtc_ctrl |= DAVINCI_PRTCSS_RTC_CTRL_TIEN |
			    DAVINCI_PRTCSS_RTC_CTRL_TMMD |
			    DAVINCI_PRTCSS_RTC_CTRL_TMRFLG;
	} else
		rtc_ctrl &= ~DAVINCI_PRTCSS_RTC_CTRL_TIEN;

	davinci_rtcss_write(davinci_rtc, rtc_ctrl, DAVINCI_PRTCSS_RTC_CTRL);

	spin_unlock_irqrestore(&davinci_rtc_lock, flags);

	return 0;
}

static int davinci_rtc_irq_set_freq(struct device *dev, int freq)
{
	struct davinci_rtc *davinci_rtc = dev_get_drvdata(dev);
	unsigned long flags;
	u16 tmr_counter = (0x8000 >> (ffs(freq) - 1));

	spin_lock_irqsave(&davinci_rtc_lock, flags);

	davinci_rtcss_write(davinci_rtc, tmr_counter & 0xFF,
			    DAVINCI_PRTCSS_RTC_TMR0);
	davinci_rtcss_write(davinci_rtc, (tmr_counter & 0xFF00) >> 8,
			    DAVINCI_PRTCSS_RTC_TMR1);

	spin_unlock_irqrestore(&davinci_rtc_lock, flags);

	return 0;
}

static struct rtc_class_ops davinci_rtc_ops = {
	.ioctl			= davinci_rtc_ioctl,
	.read_time		= davinci_rtc_read_time,
	.set_time		= davinci_rtc_set_time,
	.alarm_irq_enable	= davinci_rtc_alarm_irq_enable,
	.read_alarm		= davinci_rtc_read_alarm,
	.set_alarm		= davinci_rtc_set_alarm,
	.irq_set_state		= davinci_rtc_irq_set_state,
	.irq_set_freq		= davinci_rtc_irq_set_freq,
};

static int __init davinci_rtc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct davinci_rtc *davinci_rtc;
	struct resource *res, *mem;
	int ret = 0;

	davinci_rtc = kzalloc(sizeof(struct davinci_rtc), GFP_KERNEL);
	if (!davinci_rtc) {
		dev_dbg(dev, "could not allocate memory for private data\n");
		return -ENOMEM;
	}

	davinci_rtc->irq = platform_get_irq(pdev, 0);
	if (davinci_rtc->irq < 0) {
		dev_err(dev, "no RTC irq\n");
		ret = davinci_rtc->irq;
		goto fail1;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(dev, "no mem resource\n");
		ret = -EINVAL;
		goto fail1;
	}

	davinci_rtc->pbase = res->start;
	davinci_rtc->base_size = resource_size(res);

	mem = request_mem_region(davinci_rtc->pbase, davinci_rtc->base_size,
				 pdev->name);
	if (!mem) {
		dev_err(dev, "RTC registers at %08x are not free\n",
			davinci_rtc->pbase);
		ret = -EBUSY;
		goto fail1;
	}

	davinci_rtc->base = ioremap(davinci_rtc->pbase, davinci_rtc->base_size);
	if (!davinci_rtc->base) {
		dev_err(dev, "unable to ioremap MEM resource\n");
		ret = -ENOMEM;
		goto fail2;
	}

	davinci_rtc->rtc = rtc_device_register(pdev->name, &pdev->dev,
				    &davinci_rtc_ops, THIS_MODULE);
	if (IS_ERR(davinci_rtc->rtc)) {
		dev_err(dev, "unable to register RTC device, err %ld\n",
				PTR_ERR(davinci_rtc->rtc));
		goto fail3;
	}

	davinci_rtcif_write(davinci_rtc, DAVINCI_PRTCIF_INTFLG_RTCSS,
			    DAVINCI_PRTCIF_INTFLG);
	davinci_rtcif_write(davinci_rtc, 0, DAVINCI_PRTCIF_INTEN);
	davinci_rtcss_write(davinci_rtc, 0, DAVINCI_PRTCSS_RTC_INTC_EXTENA1);

	davinci_rtcss_write(davinci_rtc, 0, DAVINCI_PRTCSS_RTC_CTRL);
	davinci_rtcss_write(davinci_rtc, 0, DAVINCI_PRTCSS_RTC_CCTRL);

	ret = request_irq(davinci_rtc->irq, davinci_rtc_interrupt,
			  IRQF_DISABLED, "davinci_rtc", davinci_rtc);
	if (ret < 0) {
		dev_err(dev, "unable to register davinci RTC interrupt\n");
		goto fail4;
	}

	/* Enable interrupts */
	davinci_rtcif_write(davinci_rtc, DAVINCI_PRTCIF_INTEN_RTCSS,
			    DAVINCI_PRTCIF_INTEN);
	davinci_rtcss_write(davinci_rtc, DAVINCI_PRTCSS_RTC_INTC_EXTENA1_MASK,
			    DAVINCI_PRTCSS_RTC_INTC_EXTENA1);

	davinci_rtcss_write(davinci_rtc, DAVINCI_PRTCSS_RTC_CCTRL_CAEN,
			    DAVINCI_PRTCSS_RTC_CCTRL);
//	/*while(1)*/{
//		u8 rtc_res = 0xFF;
//		rtc_res = davinci_rtcss_read(davinci_rtc, 0x04);
//		printk("Setting 32 kHz output in RTC driver: start %d, %x\r\n", rtc_res, (unsigned int)davinci_rtc->base);
//		davinci_rtcss_write(davinci_rtc, 0x01, 0x04);
//		rtc_res = davinci_rtcss_read(davinci_rtc, 0x04);
//		printk("Setting 32 kHz output in RTC driver: finish %d\r\n", rtc_res);
//
//	}
	platform_set_drvdata(pdev, davinci_rtc);
	device_init_wakeup(&pdev->dev, 0);

	return 0;

fail4:
	rtc_device_unregister(davinci_rtc->rtc);
fail3:
	iounmap(davinci_rtc->base);
fail2:
	release_mem_region(davinci_rtc->pbase, davinci_rtc->base_size);
fail1:
	kfree(davinci_rtc);

	return ret;
}

static int __devexit davinci_rtc_remove(struct platform_device *pdev)
{
	struct davinci_rtc *davinci_rtc = platform_get_drvdata(pdev);

	device_init_wakeup(&pdev->dev, 0);

	davinci_rtcif_write(davinci_rtc, 0, DAVINCI_PRTCIF_INTEN);

	free_irq(davinci_rtc->irq, davinci_rtc);

	rtc_device_unregister(davinci_rtc->rtc);

	iounmap(davinci_rtc->base);
	release_mem_region(davinci_rtc->pbase, davinci_rtc->base_size);

	platform_set_drvdata(pdev, NULL);

	kfree(davinci_rtc);

	return 0;
}

static struct platform_driver davinci_rtc_driver = {
	.probe		= davinci_rtc_probe,
	.remove		= __devexit_p(davinci_rtc_remove),
	.driver		= {
		.name = "rtc_davinci",
		.owner = THIS_MODULE,
	},
};

static int __init rtc_init(void)
{
	return platform_driver_probe(&davinci_rtc_driver, davinci_rtc_probe);
}
module_init(rtc_init);

static void __exit rtc_exit(void)
{
	platform_driver_unregister(&davinci_rtc_driver);
}
module_exit(rtc_exit);

MODULE_AUTHOR("Miguel Aguilar");
MODULE_DESCRIPTION("Texas Instruments DaVinci PRTC Driver");
MODULE_LICENSE("GPL");