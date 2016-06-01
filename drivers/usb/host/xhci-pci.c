// SPDX-License-Identifier: GPL-2.0
/*
 * xHCI host controller driver PCI Bus Glue.
 *
 * Copyright (C) 2008 Intel Corp.
 *
 * Author: Sarah Sharp
 * Some code borrowed from the Linux EHCI driver.
 */

#include <linux/pci.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/acpi.h>
#include <linux/firmware.h>
#include <asm/unaligned.h>

#include "xhci.h"
#include "xhci-trace.h"

#define SSIC_PORT_NUM		2
#define SSIC_PORT_CFG2		0x880c
#define SSIC_PORT_CFG2_OFFSET	0x30
#define PROG_DONE		(1 << 30)
#define SSIC_PORT_UNUSED	(1 << 31)

/* Device for a quirk */
#define PCI_VENDOR_ID_FRESCO_LOGIC	0x1b73
#define PCI_DEVICE_ID_FRESCO_LOGIC_PDK	0x1000
#define PCI_DEVICE_ID_FRESCO_LOGIC_FL1009	0x1009
#define PCI_DEVICE_ID_FRESCO_LOGIC_FL1400	0x1400

#define PCI_VENDOR_ID_ETRON		0x1b6f
#define PCI_DEVICE_ID_EJ168		0x7023

#define PCI_DEVICE_ID_INTEL_LYNXPOINT_XHCI	0x8c31
#define PCI_DEVICE_ID_INTEL_LYNXPOINT_LP_XHCI	0x9c31
#define PCI_DEVICE_ID_INTEL_WILDCATPOINT_LP_XHCI	0x9cb1
#define PCI_DEVICE_ID_INTEL_CHERRYVIEW_XHCI		0x22b5
#define PCI_DEVICE_ID_INTEL_SUNRISEPOINT_H_XHCI		0xa12f
#define PCI_DEVICE_ID_INTEL_SUNRISEPOINT_LP_XHCI	0x9d2f
#define PCI_DEVICE_ID_INTEL_BROXTON_M_XHCI		0x0aa8
#define PCI_DEVICE_ID_INTEL_BROXTON_B_XHCI		0x1aa8
#define PCI_DEVICE_ID_INTEL_APL_XHCI			0x5aa8
#define PCI_DEVICE_ID_INTEL_DNV_XHCI			0x19d0
#define PCI_DEVICE_ID_INTEL_ALPINE_RIDGE_2C_XHCI	0x15b5
#define PCI_DEVICE_ID_INTEL_ALPINE_RIDGE_4C_XHCI	0x15b6
#define PCI_DEVICE_ID_INTEL_ALPINE_RIDGE_C_2C_XHCI	0x15db
#define PCI_DEVICE_ID_INTEL_ALPINE_RIDGE_C_4C_XHCI	0x15d4
#define PCI_DEVICE_ID_INTEL_TITAN_RIDGE_2C_XHCI		0x15e9
#define PCI_DEVICE_ID_INTEL_TITAN_RIDGE_4C_XHCI		0x15ec
#define PCI_DEVICE_ID_INTEL_TITAN_RIDGE_DD_XHCI		0x15f0

#define PCI_DEVICE_ID_AMD_PROMONTORYA_4			0x43b9
#define PCI_DEVICE_ID_AMD_PROMONTORYA_3			0x43ba
#define PCI_DEVICE_ID_AMD_PROMONTORYA_2			0x43bb
#define PCI_DEVICE_ID_AMD_PROMONTORYA_1			0x43bc
#define PCI_DEVICE_ID_ASMEDIA_1042A_XHCI		0x1142

static const char hcd_name[] = "xhci_hcd";

static struct hc_driver __read_mostly xhci_pci_hc_driver;

static int xhci_pci_setup(struct usb_hcd *hcd);

static const struct xhci_driver_overrides xhci_pci_overrides __initconst = {
	.reset = xhci_pci_setup,
};

/* called after powerup, by probe or system-pm "wakeup" */
static int xhci_pci_reinit(struct xhci_hcd *xhci, struct pci_dev *pdev)
{
	/*
	 * TODO: Implement finding debug ports later.
	 * TODO: see if there are any quirks that need to be added to handle
	 * new extended capabilities.
	 */

	/* PCI Memory-Write-Invalidate cycle support is optional (uncommon) */
	if (!pci_set_mwi(pdev))
		xhci_dbg(xhci, "MWI active\n");

	xhci_dbg(xhci, "Finished xhci_pci_reinit\n");
	return 0;
}

static void xhci_pci_quirks(struct device *dev, struct xhci_hcd *xhci)
{
	struct pci_dev		*pdev = to_pci_dev(dev);

	/* Look for vendor-specific quirks */
	if (pdev->vendor == PCI_VENDOR_ID_FRESCO_LOGIC &&
			(pdev->device == PCI_DEVICE_ID_FRESCO_LOGIC_PDK ||
			 pdev->device == PCI_DEVICE_ID_FRESCO_LOGIC_FL1400)) {
		if (pdev->device == PCI_DEVICE_ID_FRESCO_LOGIC_PDK &&
				pdev->revision == 0x0) {
			xhci->quirks |= XHCI_RESET_EP_QUIRK;
			xhci_dbg_trace(xhci, trace_xhci_dbg_quirks,
				"QUIRK: Fresco Logic xHC needs configure"
				" endpoint cmd after reset endpoint");
		}
		if (pdev->device == PCI_DEVICE_ID_FRESCO_LOGIC_PDK &&
				pdev->revision == 0x4) {
			xhci->quirks |= XHCI_SLOW_SUSPEND;
			xhci_dbg_trace(xhci, trace_xhci_dbg_quirks,
				"QUIRK: Fresco Logic xHC revision %u"
				"must be suspended extra slowly",
				pdev->revision);
		}
		if (pdev->device == PCI_DEVICE_ID_FRESCO_LOGIC_PDK)
			xhci->quirks |= XHCI_BROKEN_STREAMS;
		/* Fresco Logic confirms: all revisions of this chip do not
		 * support MSI, even though some of them claim to in their PCI
		 * capabilities.
		 */
		xhci->quirks |= XHCI_BROKEN_MSI;
		xhci_dbg_trace(xhci, trace_xhci_dbg_quirks,
				"QUIRK: Fresco Logic revision %u "
				"has broken MSI implementation",
				pdev->revision);
		xhci->quirks |= XHCI_TRUST_TX_LENGTH;
	}

	if (pdev->vendor == PCI_VENDOR_ID_FRESCO_LOGIC &&
			pdev->device == PCI_DEVICE_ID_FRESCO_LOGIC_FL1009)
		xhci->quirks |= XHCI_BROKEN_STREAMS;

	if (pdev->vendor == PCI_VENDOR_ID_NEC)
		xhci->quirks |= XHCI_NEC_HOST;

	if (pdev->vendor == PCI_VENDOR_ID_AMD && xhci->hci_version == 0x96)
		xhci->quirks |= XHCI_AMD_0x96_HOST;

	/* AMD PLL quirk */
	if (pdev->vendor == PCI_VENDOR_ID_AMD && usb_amd_quirk_pll_check())
		xhci->quirks |= XHCI_AMD_PLL_FIX;

	if (pdev->vendor == PCI_VENDOR_ID_AMD &&
		(pdev->device == 0x15e0 ||
		 pdev->device == 0x15e1 ||
		 pdev->device == 0x43bb))
		xhci->quirks |= XHCI_SUSPEND_DELAY;

	if (pdev->vendor == PCI_VENDOR_ID_AMD &&
	    (pdev->device == 0x15e0 || pdev->device == 0x15e1))
		xhci->quirks |= XHCI_SNPS_BROKEN_SUSPEND;

	if (pdev->vendor == PCI_VENDOR_ID_AMD)
		xhci->quirks |= XHCI_TRUST_TX_LENGTH;

	if ((pdev->vendor == PCI_VENDOR_ID_AMD) &&
		((pdev->device == PCI_DEVICE_ID_AMD_PROMONTORYA_4) ||
		(pdev->device == PCI_DEVICE_ID_AMD_PROMONTORYA_3) ||
		(pdev->device == PCI_DEVICE_ID_AMD_PROMONTORYA_2) ||
		(pdev->device == PCI_DEVICE_ID_AMD_PROMONTORYA_1)))
		xhci->quirks |= XHCI_U2_DISABLE_WAKE;

	if (pdev->vendor == PCI_VENDOR_ID_INTEL) {
		xhci->quirks |= XHCI_LPM_SUPPORT;
		xhci->quirks |= XHCI_INTEL_HOST;
		xhci->quirks |= XHCI_AVOID_BEI;
	}
	if (pdev->vendor == PCI_VENDOR_ID_INTEL &&
			pdev->device == PCI_DEVICE_ID_INTEL_PANTHERPOINT_XHCI) {
		xhci->quirks |= XHCI_EP_LIMIT_QUIRK;
		xhci->limit_active_eps = 64;
		xhci->quirks |= XHCI_SW_BW_CHECKING;
		/*
		 * PPT desktop boards DH77EB and DH77DF will power back on after
		 * a few seconds of being shutdown.  The fix for this is to
		 * switch the ports from xHCI to EHCI on shutdown.  We can't use
		 * DMI information to find those particular boards (since each
		 * vendor will change the board name), so we have to key off all
		 * PPT chipsets.
		 */
		xhci->quirks |= XHCI_SPURIOUS_REBOOT;
	}
	if (pdev->vendor == PCI_VENDOR_ID_INTEL &&
		(pdev->device == PCI_DEVICE_ID_INTEL_LYNXPOINT_LP_XHCI ||
		 pdev->device == PCI_DEVICE_ID_INTEL_WILDCATPOINT_LP_XHCI)) {
		xhci->quirks |= XHCI_SPURIOUS_REBOOT;
		xhci->quirks |= XHCI_SPURIOUS_WAKEUP;
	}
	if (pdev->vendor == PCI_VENDOR_ID_INTEL &&
		(pdev->device == PCI_DEVICE_ID_INTEL_SUNRISEPOINT_LP_XHCI ||
		 pdev->device == PCI_DEVICE_ID_INTEL_SUNRISEPOINT_H_XHCI ||
		 pdev->device == PCI_DEVICE_ID_INTEL_CHERRYVIEW_XHCI ||
		 pdev->device == PCI_DEVICE_ID_INTEL_BROXTON_M_XHCI ||
		 pdev->device == PCI_DEVICE_ID_INTEL_BROXTON_B_XHCI ||
		 pdev->device == PCI_DEVICE_ID_INTEL_APL_XHCI ||
		 pdev->device == PCI_DEVICE_ID_INTEL_DNV_XHCI)) {
		xhci->quirks |= XHCI_PME_STUCK_QUIRK;
	}
	if (pdev->vendor == PCI_VENDOR_ID_INTEL &&
	    pdev->device == PCI_DEVICE_ID_INTEL_CHERRYVIEW_XHCI)
		xhci->quirks |= XHCI_SSIC_PORT_UNUSED;
	if (pdev->vendor == PCI_VENDOR_ID_INTEL &&
	    (pdev->device == PCI_DEVICE_ID_INTEL_CHERRYVIEW_XHCI ||
	     pdev->device == PCI_DEVICE_ID_INTEL_SUNRISEPOINT_LP_XHCI ||
	     pdev->device == PCI_DEVICE_ID_INTEL_APL_XHCI))
		xhci->quirks |= XHCI_INTEL_USB_ROLE_SW;
	if (pdev->vendor == PCI_VENDOR_ID_INTEL &&
	    (pdev->device == PCI_DEVICE_ID_INTEL_CHERRYVIEW_XHCI ||
	     pdev->device == PCI_DEVICE_ID_INTEL_SUNRISEPOINT_LP_XHCI ||
	     pdev->device == PCI_DEVICE_ID_INTEL_SUNRISEPOINT_H_XHCI ||
	     pdev->device == PCI_DEVICE_ID_INTEL_APL_XHCI ||
	     pdev->device == PCI_DEVICE_ID_INTEL_DNV_XHCI))
		xhci->quirks |= XHCI_MISSING_CAS;

	if (pdev->vendor == PCI_VENDOR_ID_INTEL &&
	    (pdev->device == PCI_DEVICE_ID_INTEL_ALPINE_RIDGE_2C_XHCI ||
	     pdev->device == PCI_DEVICE_ID_INTEL_ALPINE_RIDGE_4C_XHCI ||
	     pdev->device == PCI_DEVICE_ID_INTEL_ALPINE_RIDGE_C_2C_XHCI ||
	     pdev->device == PCI_DEVICE_ID_INTEL_ALPINE_RIDGE_C_4C_XHCI ||
	     pdev->device == PCI_DEVICE_ID_INTEL_TITAN_RIDGE_2C_XHCI ||
	     pdev->device == PCI_DEVICE_ID_INTEL_TITAN_RIDGE_4C_XHCI ||
	     pdev->device == PCI_DEVICE_ID_INTEL_TITAN_RIDGE_DD_XHCI))
		xhci->quirks |= XHCI_DEFAULT_PM_RUNTIME_ALLOW;

	if (pdev->vendor == PCI_VENDOR_ID_ETRON &&
			pdev->device == PCI_DEVICE_ID_EJ168) {
		xhci->quirks |= XHCI_RESET_ON_RESUME;
		xhci->quirks |= XHCI_TRUST_TX_LENGTH;
		xhci->quirks |= XHCI_BROKEN_STREAMS;
	}
	if (pdev->vendor == PCI_VENDOR_ID_RENESAS &&
	    pdev->device == 0x0014) {
		xhci->quirks |= XHCI_TRUST_TX_LENGTH;
		xhci->quirks |= XHCI_ZERO_64B_REGS;
	}
	if (pdev->vendor == PCI_VENDOR_ID_RENESAS &&
	    pdev->device == 0x0015) {
		xhci->quirks |= XHCI_RESET_ON_RESUME;
		xhci->quirks |= XHCI_ZERO_64B_REGS;
	}
	if (pdev->vendor == PCI_VENDOR_ID_VIA)
		xhci->quirks |= XHCI_RESET_ON_RESUME;

	/* See https://bugzilla.kernel.org/show_bug.cgi?id=79511 */
	if (pdev->vendor == PCI_VENDOR_ID_VIA &&
			pdev->device == 0x3432)
		xhci->quirks |= XHCI_BROKEN_STREAMS;

	if (pdev->vendor == PCI_VENDOR_ID_ASMEDIA &&
			pdev->device == 0x1042)
		xhci->quirks |= XHCI_BROKEN_STREAMS;
	if (pdev->vendor == PCI_VENDOR_ID_ASMEDIA &&
			pdev->device == 0x1142)
		xhci->quirks |= XHCI_TRUST_TX_LENGTH;

	if (pdev->vendor == PCI_VENDOR_ID_ASMEDIA &&
		pdev->device == PCI_DEVICE_ID_ASMEDIA_1042A_XHCI)
		xhci->quirks |= XHCI_ASMEDIA_MODIFY_FLOWCONTROL;

	if (pdev->vendor == PCI_VENDOR_ID_TI && pdev->device == 0x8241)
		xhci->quirks |= XHCI_LIMIT_ENDPOINT_INTERVAL_7;

	if ((pdev->vendor == PCI_VENDOR_ID_BROADCOM ||
	     pdev->vendor == PCI_VENDOR_ID_CAVIUM) &&
	     pdev->device == 0x9026)
		xhci->quirks |= XHCI_RESET_PLL_ON_DISCONNECT;

	if (xhci->quirks & XHCI_RESET_ON_RESUME)
		xhci_dbg_trace(xhci, trace_xhci_dbg_quirks,
				"QUIRK: Resetting on resume");
}

#ifdef CONFIG_ACPI
static void xhci_pme_acpi_rtd3_enable(struct pci_dev *dev)
{
	static const guid_t intel_dsm_guid =
		GUID_INIT(0xac340cb7, 0xe901, 0x45bf,
			  0xb7, 0xe6, 0x2b, 0x34, 0xec, 0x93, 0x1e, 0x23);
	union acpi_object *obj;

	obj = acpi_evaluate_dsm(ACPI_HANDLE(&dev->dev), &intel_dsm_guid, 3, 1,
				NULL);
	ACPI_FREE(obj);
}
#else
static void xhci_pme_acpi_rtd3_enable(struct pci_dev *dev) { }
#endif /* CONFIG_ACPI */

static const struct renesas_fw_entry {
	const char *firmware_name;
	u16 device;
	u8 revision;
	u16 expected_version;
} renesas_fw_table[] = {
	/*
	 * Only the uPD720201K8-711-BAC-A or uPD720202K8-711-BAA-A
	 * are listed in R19UH0078EJ0500 Rev.5.00 as devices which
	 * need the software loader.
	 *
	 * PP2U/ReleaseNote_USB3-201-202-FW.txt:
	 *
	 * Note: This firmware is for the following devices.
	 *  - uPD720201 ES 2.0 sample whose revision ID is 2.
	 *  - uPD720201 ES 2.1 sample & CS sample & Mass product, ID is 3.
	 *  - uPD720202 ES 2.0 sample & CS sample & Mass product, ID is 2.
	 */
	{ "K2013080.mem", 0x0014, 0x02, 0x2013 },
	{ "K2013080.mem", 0x0014, 0x03, 0x2013 },
	{ "K2013080.mem", 0x0015, 0x02, 0x2013 },
};

static const struct renesas_fw_entry *renesas_needs_fw_dl(struct pci_dev *dev)
{
	const struct renesas_fw_entry *entry;
	size_t i;

	/* This loader will only work with a RENESAS device. */
	if (!(dev->vendor == PCI_VENDOR_ID_RENESAS))
		return NULL;

	for (i = 0; i < ARRAY_SIZE(renesas_fw_table); i++) {
		entry = &renesas_fw_table[i];
		if (entry->device == dev->device &&
		    entry->revision == dev->revision)
			return entry;
	}

	return NULL;
}

static int renesas_fw_download_image(struct pci_dev *dev,
				     const u32 *fw,
				     size_t step)
{
	size_t i;
	int err;
	u8 fw_status;
	bool data0_or_data1;

	/*
	 * The hardware does alternate between two 32-bit pages.
	 * (This is because each row of the firmware is 8 bytes).
	 *
	 * for even steps we use DATA0, for odd steps DATA1.
	 */
	data0_or_data1 = (step & 1) == 1;

	/* step+1. Read "Set DATAX" and confirm it is cleared. */
	for (i = 0; i < 10000; i++) {
		err = pci_read_config_byte(dev, 0xF5, &fw_status);
		if (err)
			return pcibios_err_to_errno(err);
		if (!(fw_status & BIT(data0_or_data1)))
			break;

		udelay(1);
	}
	if (i == 10000)
		return -ETIMEDOUT;

	/*
	 * step+2. Write FW data to "DATAX".
	 * "LSB is left" => force little endian
	 */
	err = pci_write_config_dword(dev, data0_or_data1 ? 0xFC : 0xF8,
				     (__force u32) cpu_to_le32(fw[step]));
	if (err)
		return pcibios_err_to_errno(err);

	udelay(100);

	/* step+3. Set "Set DATAX". */
	err = pci_write_config_byte(dev, 0xF5, BIT(data0_or_data1));
	if (err)
		return pcibios_err_to_errno(err);

	return 0;
}

static int renesas_fw_verify(struct pci_dev *dev,
			     const void *fw_data,
			     size_t length)
{
	const struct renesas_fw_entry *entry = renesas_needs_fw_dl(dev);
	u16 fw_version_pointer;
	u16 fw_version;

	if (!entry)
		return -EINVAL;

	/*
	 * The Firmware's Data Format is describe in
	 * "6.3 Data Format" R19UH0078EJ0500 Rev.5.00 page 124
	 */

	/* "Each row is 8 bytes". => firmware size must be a multiple of 8. */
	if (length % 8 != 0) {
		dev_err(&dev->dev, "firmware size is not a multipe of 8.");
		return -EINVAL;
	}

	/*
	 * The bootrom chips of the big brother have sizes up to 64k, let's
	 * assume that's the biggest the firmware can get.
	 */
	if (length < 0x1000 || length >= 0x10000) {
		dev_err(&dev->dev, "firmware is size %zd is not (4k - 64k).",
			length);
		return -EINVAL;
	}

	/* The First 2 bytes are fixed value (55aa). "LSB on Left" */
	if (get_unaligned_le16(fw_data) != 0x55aa) {
		dev_err(&dev->dev, "no valid firmware header found.");
		return -EINVAL;
	}

	/* verify the firmware version position and print it. */
	fw_version_pointer = get_unaligned_le16(fw_data + 4);
	if (fw_version_pointer + 2 >= length) {
		dev_err(&dev->dev, "firmware version pointer is outside of the firmware image.");
		return -EINVAL;
	}

	fw_version = get_unaligned_le16(fw_data + fw_version_pointer);
	dev_dbg(&dev->dev, "got firmware version: %02x.", fw_version);

	if (fw_version != entry->expected_version) {
		dev_err(&dev->dev, "firmware version mismatch, expected version: %02x.",
			 entry->expected_version);
		return -EINVAL;
	}

	return 0;
}

static int renesas_fw_check_running(struct pci_dev *pdev)
{
	int err;
	u8 fw_state;

	/*
	 * Test if the device is actually needing the firmware. As most
	 * BIOSes will initialize the device for us. If the device is
	 * initialized.
	 */
	err = pci_read_config_byte(pdev, 0xF4, &fw_state);
	if (err)
		return pcibios_err_to_errno(err);

	/*
	 * Check if "FW Download Lock" is locked. If it is and the FW is
	 * ready we can simply continue. If the FW is not ready, we have
	 * to give up.
	 */
	if (fw_state & BIT(1)) {
		dev_dbg(&pdev->dev, "FW Download Lock is engaged.");

		if (fw_state & BIT(4))
			return 0;

		dev_err(&pdev->dev, "FW Download Lock is set and FW is not ready. Giving Up.");
		return -EIO;
	}

	/*
	 * Check if "FW Download Enable" is set. If someone (us?) tampered
	 * with it and it can't be resetted, we have to give up too... and
	 * ask for a forgiveness and a reboot.
	 */
	if (fw_state & BIT(0)) {
		dev_err(&pdev->dev, "FW Download Enable is stale. Giving Up (poweroff/reboot needed).");
		return -EIO;
	}

	/* Otherwise, Check the "Result Code" Bits (6:4) and act accordingly */
	switch ((fw_state & 0x70)) {
	case 0: /* No result yet */
		dev_dbg(&pdev->dev, "FW is not ready/loaded yet.");

		/* tell the caller, that this device needs the firmware. */
		return 1;

	case BIT(4): /* Success, device should be working. */
		dev_dbg(&pdev->dev, "FW is ready.");
		return 0;

	case BIT(5): /* Error State */
		dev_err(&pdev->dev, "hardware is in an error state. Giving up (poweroff/reboot needed).");
		return -ENODEV;

	default: /* All other states are marked as "Reserved states" */
		dev_err(&pdev->dev, "hardware is in an invalid state %x. Giving up (poweroff/reboot needed).",
			(fw_state & 0x70) >> 4);
		return -EINVAL;
	}
}

static int renesas_hw_check_run_stop_busy(struct pci_dev *pdev)
{
#if 0
	u32 val;

	/*
	 * 7.1.3 Note 3: "... must not set 'FW Download Enable' when
	 * 'RUN/STOP' of USBCMD Register is set"
	 */
	val = readl(hcd->regs + 0x20);
	if (val & BIT(0)) {
		dev_err(&pdev->dev, "hardware is busy and can't receive a FW.");
		return -EBUSY;
	}
#endif
	return 0;
}

static int renesas_fw_download(struct pci_dev *pdev,
	const struct firmware *fw, unsigned int retry_counter)
{
	const u32 *fw_data = (const u32 *) fw->data;
	size_t i;
	int err;
	u8 fw_status;

	/*
	 * For more information and the big picture: please look at the
	 * "Firmware Download Sequence" in "7.1 FW Download Interface"
	 * of R19UH0078EJ0500 Rev.5.00 page 131
	 */
	err = renesas_hw_check_run_stop_busy(pdev);
	if (err)
		return err;

	/*
	 * 0. Set "FW Download Enable" bit in the
	 * "FW Download Control & Status Register" at 0xF4
	 */
	err = pci_write_config_byte(pdev, 0xF4, BIT(0));
	if (err)
		return pcibios_err_to_errno(err);

	/* 1 - 10 follow one step after the other. */
	for (i = 0; i < fw->size / 4; i++) {
		err = renesas_fw_download_image(pdev, fw_data, i);
		if (err) {
			dev_err(&pdev->dev, "Firmware Download Step %zd failed at position %zd bytes with (%d).",
				 i, i * 4, err);
			return err;
		}
	}

	/*
	 * This sequence continues until the last data is written to
	 * "DATA0" or "DATA1". Naturally, we wait until "SET DATA0/1"
	 * is cleared by the hardware beforehand.
	 */
	for (i = 0; i < 10000; i++) {
		err = pci_read_config_byte(pdev, 0xF5, &fw_status);
		if (err)
			return pcibios_err_to_errno(err);
		if (!(fw_status & (BIT(0) | BIT(1))))
			break;

		udelay(1);
	}
	if (i == 10000)
		dev_warn(&pdev->dev, "Final Firmware Download step timed out.");

	/*
	 * 11. After finishing writing the last data of FW, the
	 * System Software must clear "FW Download Enable"
	 */
	err = pci_write_config_byte(pdev, 0xF4, 0);
	if (err)
		return pcibios_err_to_errno(err);

	/* 12. Read "Result Code" and confirm it is good. */
	for (i = 0; i < 10000; i++) {
		err = pci_read_config_byte(pdev, 0xF4, &fw_status);
		if (err)
			return pcibios_err_to_errno(err);
		if (fw_status & BIT(4))
			break;

		udelay(1);
	}
	if (i == 10000) {
		/* Timed out / Error - let's see if we can fix this */
		err = renesas_fw_check_running(pdev);
		switch (err) {
		case 0: /*
			 * we shouldn't end up here.
			 * maybe it took a little bit longer.
			 * But all should be well?
			 */
			break;

		case 1: /* (No result yet? - we can try to retry) */
			if (retry_counter < 10) {
				retry_counter++;
				dev_warn(&pdev->dev, "Retry Firmware download: %d try.",
					  retry_counter);
				return renesas_fw_download(pdev, fw,
							   retry_counter);
			}
			return -ETIMEDOUT;

		default:
			return err;
		}
	}
	/*
	 * Optional last step: Engage Firmware Lock
	 *
	 * err = pci_write_config_byte(pdev, 0xF4, BIT(2));
	 * if (err)
	 *	return pcibios_err_to_errno(err);
	 */

	return 0;
}

struct renesas_fw_ctx {
	struct pci_dev *pdev;
	const struct pci_device_id *id;
	bool resume;
};

static int xhci_pci_probe(struct pci_dev *pdev,
			  const struct pci_device_id *id);

static void renesas_fw_callback(const struct firmware *fw,
				void *context)
{
	struct renesas_fw_ctx *ctx = context;
	struct pci_dev *pdev = ctx->pdev;
	struct device *parent = pdev->dev.parent;
	int err = -ENOENT;

	if (fw) {
		err = renesas_fw_verify(pdev, fw->data, fw->size);
		if (!err) {
			err = renesas_fw_download(pdev, fw, 0);
			release_firmware(fw);
			if (!err) {
				if (ctx->resume)
					return;

				err = xhci_pci_probe(pdev, ctx->id);
				if (!err) {
					/* everything worked */
					devm_kfree(&pdev->dev, ctx);
					return;
				}

				/* in case of an error - fall through */
			} else {
				dev_err(&pdev->dev, "firmware failed to download (%d).",
					err);
			}
		}
	} else {
		dev_err(&pdev->dev, "firmware failed to load (%d).", err);
	}

	dev_info(&pdev->dev, "Unloading driver");

	if (parent)
		device_lock(parent);

	device_release_driver(&pdev->dev);

	if (parent)
		device_unlock(parent);

	pci_dev_put(pdev);
}

static int renesas_fw_alive_check(struct pci_dev *pdev)
{
	const struct renesas_fw_entry *entry;
	int err;

	/* check if we have a eligible RENESAS' uPD720201/2 w/o FW. */
	entry = renesas_needs_fw_dl(pdev);
	if (!entry)
		return 0;

	err = renesas_fw_check_running(pdev);
	/* Also go ahead, if the firmware is running */
	if (err == 0)
		return 0;

	/* At this point, we can be sure that the FW isn't ready. */
	return err;
}

static int renesas_fw_download_to_hw(struct pci_dev *pdev,
				     const struct pci_device_id *id,
				     bool do_resume)
{
	const struct renesas_fw_entry *entry;
	struct renesas_fw_ctx *ctx;
	int err;

	/* check if we have a eligible RENESAS' uPD720201/2 w/o FW. */
	entry = renesas_needs_fw_dl(pdev);
	if (!entry)
		return 0;

	err = renesas_fw_check_running(pdev);
	/* Continue ahead, if the firmware is already running. */
	if (err == 0)
		return 0;

	if (err != 1)
		return err;

	ctx = devm_kzalloc(&pdev->dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;
	ctx->pdev = pdev;
	ctx->resume = do_resume;
	ctx->id = id;

	pci_dev_get(pdev);
	err = request_firmware_nowait(THIS_MODULE, 1, entry->firmware_name,
		&pdev->dev, GFP_KERNEL, ctx, renesas_fw_callback);
	if (err) {
		pci_dev_put(pdev);
		return err;
	}

	/*
	 * The renesas_fw_callback() callback will continue the probe
	 * process, once it aquires the firmware.
	 */
	return 1;
}

/* called during probe() after chip reset completes */
static int xhci_pci_setup(struct usb_hcd *hcd)
{
	struct xhci_hcd		*xhci;
	struct pci_dev		*pdev = to_pci_dev(hcd->self.controller);
	int			retval;

	xhci = hcd_to_xhci(hcd);
	if (!xhci->sbrn)
		pci_read_config_byte(pdev, XHCI_SBRN_OFFSET, &xhci->sbrn);

	/* imod_interval is the interrupt moderation value in nanoseconds. */
	xhci->imod_interval = 40000;

	retval = xhci_gen_setup(hcd, xhci_pci_quirks);
	if (retval)
		return retval;

	if (!usb_hcd_is_primary_hcd(hcd))
		return 0;

	xhci_dbg(xhci, "Got SBRN %u\n", (unsigned int) xhci->sbrn);

	/* Find any debug ports */
	return xhci_pci_reinit(xhci, pdev);
}

/*
 * We need to register our own PCI probe function (instead of the USB core's
 * function) in order to create a second roothub under xHCI.
 */
static int xhci_pci_probe(struct pci_dev *dev, const struct pci_device_id *id)
{
	int retval;
	struct xhci_hcd *xhci;
	struct hc_driver *driver;
	struct usb_hcd *hcd;

	/*
	 * Check if this device is a RENESAS uPD720201/2 device.
	 * Otherwise, we can continue with xhci_pci_probe as usual.
	 */
	retval = renesas_fw_download_to_hw(dev, id, false);
	switch (retval) {
	case 0:
		break;

	case 1: /* let it load the firmware and recontinue the probe. */
		return 0;

	default:
		return retval;
	};

	driver = (struct hc_driver *)id->driver_data;

	/* Prevent runtime suspending between USB-2 and USB-3 initialization */
	pm_runtime_get_noresume(&dev->dev);

	/* Register the USB 2.0 roothub.
	 * FIXME: USB core must know to register the USB 2.0 roothub first.
	 * This is sort of silly, because we could just set the HCD driver flags
	 * to say USB 2.0, but I'm not sure what the implications would be in
	 * the other parts of the HCD code.
	 */
	retval = usb_hcd_pci_probe(dev, id);

	if (retval)
		goto put_runtime_pm;

	/* USB 2.0 roothub is stored in the PCI device now. */
	hcd = dev_get_drvdata(&dev->dev);
	xhci = hcd_to_xhci(hcd);
	xhci->shared_hcd = usb_create_shared_hcd(driver, &dev->dev,
				pci_name(dev), hcd);
	if (!xhci->shared_hcd) {
		retval = -ENOMEM;
		goto dealloc_usb2_hcd;
	}

	retval = xhci_ext_cap_init(xhci);
	if (retval)
		goto put_usb3_hcd;

	retval = usb_add_hcd(xhci->shared_hcd, dev->irq,
			IRQF_SHARED);
	if (retval)
		goto put_usb3_hcd;
	/* Roothub already marked as USB 3.0 speed */

	if (!(xhci->quirks & XHCI_BROKEN_STREAMS) &&
			HCC_MAX_PSA(xhci->hcc_params) >= 4)
		xhci->shared_hcd->can_do_streams = 1;

	if (xhci->quirks & XHCI_PME_STUCK_QUIRK)
		xhci_pme_acpi_rtd3_enable(dev);

	/* USB-2 and USB-3 roothubs initialized, allow runtime pm suspend */
	pm_runtime_put_noidle(&dev->dev);

	if (xhci->quirks & XHCI_DEFAULT_PM_RUNTIME_ALLOW)
		pm_runtime_allow(&dev->dev);

	return 0;

put_usb3_hcd:
	usb_put_hcd(xhci->shared_hcd);
dealloc_usb2_hcd:
	usb_hcd_pci_remove(dev);
put_runtime_pm:
	pm_runtime_put_noidle(&dev->dev);
	return retval;
}

static void xhci_pci_remove(struct pci_dev *dev)
{
	struct xhci_hcd *xhci;

	if (renesas_fw_alive_check(dev)) {
		/*
		 * bail out early, if this was a renesas device w/o FW.
		 * Else we might hit the NMI watchdog in xhci_handsake
		 * during xhci_reset as part of the driver's unloading.
		 * which we forced in the renesas_fw_callback().
		 */
		return;
	}

	xhci = hcd_to_xhci(pci_get_drvdata(dev));
	xhci->xhc_state |= XHCI_STATE_REMOVING;

	if (xhci->quirks & XHCI_DEFAULT_PM_RUNTIME_ALLOW)
		pm_runtime_forbid(&dev->dev);

	if (xhci->shared_hcd) {
		usb_remove_hcd(xhci->shared_hcd);
		usb_put_hcd(xhci->shared_hcd);
		xhci->shared_hcd = NULL;
	}

	/* Workaround for spurious wakeups at shutdown with HSW */
	if (xhci->quirks & XHCI_SPURIOUS_WAKEUP)
		pci_set_power_state(dev, PCI_D3hot);

	usb_hcd_pci_remove(dev);
}

#ifdef CONFIG_PM
/*
 * In some Intel xHCI controllers, in order to get D3 working,
 * through a vendor specific SSIC CONFIG register at offset 0x883c,
 * SSIC PORT need to be marked as "unused" before putting xHCI
 * into D3. After D3 exit, the SSIC port need to be marked as "used".
 * Without this change, xHCI might not enter D3 state.
 */
static void xhci_ssic_port_unused_quirk(struct usb_hcd *hcd, bool suspend)
{
	struct xhci_hcd	*xhci = hcd_to_xhci(hcd);
	u32 val;
	void __iomem *reg;
	int i;

	for (i = 0; i < SSIC_PORT_NUM; i++) {
		reg = (void __iomem *) xhci->cap_regs +
				SSIC_PORT_CFG2 +
				i * SSIC_PORT_CFG2_OFFSET;

		/* Notify SSIC that SSIC profile programming is not done. */
		val = readl(reg) & ~PROG_DONE;
		writel(val, reg);

		/* Mark SSIC port as unused(suspend) or used(resume) */
		val = readl(reg);
		if (suspend)
			val |= SSIC_PORT_UNUSED;
		else
			val &= ~SSIC_PORT_UNUSED;
		writel(val, reg);

		/* Notify SSIC that SSIC profile programming is done */
		val = readl(reg) | PROG_DONE;
		writel(val, reg);
		readl(reg);
	}
}

/*
 * Make sure PME works on some Intel xHCI controllers by writing 1 to clear
 * the Internal PME flag bit in vendor specific PMCTRL register at offset 0x80a4
 */
static void xhci_pme_quirk(struct usb_hcd *hcd)
{
	struct xhci_hcd	*xhci = hcd_to_xhci(hcd);
	void __iomem *reg;
	u32 val;

	reg = (void __iomem *) xhci->cap_regs + 0x80a4;
	val = readl(reg);
	writel(val | BIT(28), reg);
	readl(reg);
}

static int xhci_pci_suspend(struct usb_hcd *hcd, bool do_wakeup)
{
	struct xhci_hcd	*xhci = hcd_to_xhci(hcd);
	struct pci_dev		*pdev = to_pci_dev(hcd->self.controller);
	int			ret;

	/*
	 * Systems with the TI redriver that loses port status change events
	 * need to have the registers polled during D3, so avoid D3cold.
	 */
	if (xhci->quirks & XHCI_COMP_MODE_QUIRK)
		pci_d3cold_disable(pdev);

	if (xhci->quirks & XHCI_PME_STUCK_QUIRK)
		xhci_pme_quirk(hcd);

	if (xhci->quirks & XHCI_SSIC_PORT_UNUSED)
		xhci_ssic_port_unused_quirk(hcd, true);

	ret = xhci_suspend(xhci, do_wakeup);
	if (ret && (xhci->quirks & XHCI_SSIC_PORT_UNUSED))
		xhci_ssic_port_unused_quirk(hcd, false);

	return ret;
}

static int xhci_pci_resume(struct usb_hcd *hcd, bool hibernated)
{
	struct xhci_hcd		*xhci = hcd_to_xhci(hcd);
	struct pci_dev		*pdev = to_pci_dev(hcd->self.controller);
	int			retval = 0;

	/* The BIOS on systems with the Intel Panther Point chipset may or may
	 * not support xHCI natively.  That means that during system resume, it
	 * may switch the ports back to EHCI so that users can use their
	 * keyboard to select a kernel from GRUB after resume from hibernate.
	 *
	 * The BIOS is supposed to remember whether the OS had xHCI ports
	 * enabled before resume, and switch the ports back to xHCI when the
	 * BIOS/OS semaphore is written, but we all know we can't trust BIOS
	 * writers.
	 *
	 * Unconditionally switch the ports back to xHCI after a system resume.
	 * It should not matter whether the EHCI or xHCI controller is
	 * resumed first. It's enough to do the switchover in xHCI because
	 * USB core won't notice anything as the hub driver doesn't start
	 * running again until after all the devices (including both EHCI and
	 * xHCI host controllers) have been resumed.
	 */

	if (pdev->vendor == PCI_VENDOR_ID_INTEL)
		usb_enable_intel_xhci_ports(pdev);

	if (xhci->quirks & XHCI_SSIC_PORT_UNUSED)
		xhci_ssic_port_unused_quirk(hcd, false);

	if (xhci->quirks & XHCI_PME_STUCK_QUIRK)
		xhci_pme_quirk(hcd);

	retval = xhci_resume(xhci, hibernated);
	return retval;
}
#endif /* CONFIG_PM */

/*-------------------------------------------------------------------------*/

/* PCI driver selection metadata; PCI hotplugging uses this */
static const struct pci_device_id pci_ids[] = { {
	/* handle any USB 3.0 xHCI controller */
	PCI_DEVICE_CLASS(PCI_CLASS_SERIAL_USB_XHCI, ~0),
	.driver_data =	(unsigned long) &xhci_pci_hc_driver,
	},
	{ /* end: all zeroes */ }
};
MODULE_DEVICE_TABLE(pci, pci_ids);

/* pci driver glue; this is a "new style" PCI driver module */
static struct pci_driver xhci_pci_driver = {
	.name =		(char *) hcd_name,
	.id_table =	pci_ids,

	.probe =	xhci_pci_probe,
	.remove =	xhci_pci_remove,
	/* suspend and resume implemented later */

	.shutdown = 	usb_hcd_pci_shutdown,
#ifdef CONFIG_PM
	.driver = {
		.pm = &usb_hcd_pci_pm_ops
	},
#endif
};

static int __init xhci_pci_init(void)
{
	xhci_init_driver(&xhci_pci_hc_driver, &xhci_pci_overrides);
#ifdef CONFIG_PM
	xhci_pci_hc_driver.pci_suspend = xhci_pci_suspend;
	xhci_pci_hc_driver.pci_resume = xhci_pci_resume;
#endif
	return pci_register_driver(&xhci_pci_driver);
}
module_init(xhci_pci_init);

static void __exit xhci_pci_exit(void)
{
	pci_unregister_driver(&xhci_pci_driver);
}
module_exit(xhci_pci_exit);

MODULE_DESCRIPTION("xHCI PCI Host Controller Driver");
MODULE_LICENSE("GPL");
