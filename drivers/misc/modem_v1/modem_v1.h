/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Samsung modem interface (SVNET2 / SIPC5): board data and shared enums.
 *
 * Copyright (C) 2010 Samsung Electronics.
 * 6.6 port: board data comes from DT only; GPIOs are gpio_desc.
 *
 * The numeric values of every enum below are part of the user-space
 * contract (cbd and RIL pass them through ioctls or read them from
 * sysfs) or of the SIPC5/SBD wire format. Do not renumber them.
 */

#ifndef __MODEM_V1_H__
#define __MODEM_V1_H__

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/of.h>

struct gpio_desc;

#define MAX_STR_LEN		256
#define MAX_NAME_LEN		64
#define MAX_DUMP_LEN		20

enum modem_t {
	IMC_XMM6260,
	IMC_XMM6262,
	VIA_CBP71,
	VIA_CBP72,
	VIA_CBP82,
	SEC_CMC220,
	SEC_CMC221,
	SEC_SS222,
	SEC_SS300,
	SEC_SH222AP,
	QC_MDM6600,
	QC_ESC6270,
	QC_QSC6085,
	SPRD_SC8803,
	IMC_XMM7260,
	SEC_SS333,		/* 15 */
	DUMMY,
	MAX_MODEM_TYPE
};

enum dev_format {
	IPC_FMT,
	IPC_RAW,
	IPC_RFS,
	IPC_MULTI_RAW,
	IPC_BOOT,
	IPC_DUMP,
	IPC_CMD,
	IPC_DEBUG,
	MAX_DEV_FORMAT,
};
#define MAX_SIPC_DEVICES	(IPC_RFS + 1)	/* FMT, RAW, RFS */
#define MAX_SIPC5_DEVICES	(IPC_RAW + 1)	/* FMT, RAW */
#define MAX_SIPC_CHANNELS	256
#define MAX_LINK_CHANNELS	32

enum modem_io {
	IODEV_MISC,
	IODEV_NET,
	IODEV_DUMMY,
};

/* The order must match cbd's "-b"/"-m" link letters. */
enum modem_link {
	LINKDEV_UNDEFINED,
	LINKDEV_MIPI,
	LINKDEV_DPRAM,
	LINKDEV_SPI,		/* 3 */
	LINKDEV_USB,
	LINKDEV_HSIC,
	LINKDEV_C2C,
	LINKDEV_UART,
	LINKDEV_PLD,
	LINKDEV_SHMEM,
	LINKDEV_LLI,		/* 10 */
	LINKDEV_MAX
};
#define LINKTYPE(modem_link) (1u << (modem_link))

enum modem_network {
	UMTS_NETWORK,
	CDMA_NETWORK,
	TDSCDMA_NETWORK,
	LTE_NETWORK,
	MAX_MODEM_NETWORK
};

enum sipc_ver {
	NO_SIPC_VER = 0,
	SIPC_VER_40 = 40,
	SIPC_VER_41 = 41,
	SIPC_VER_42 = 42,
	SIPC_VER_50 = 50,
	MAX_SIPC_VER
};

enum sipc_ch_id {
	SIPC_CH_ID_RAW_0 = 0,	/* reserved */
	SIPC_CH_ID_CS_VT_DATA,
	SIPC_CH_ID_CS_VT_CONTROL,
	SIPC_CH_ID_CS_VT_AUDIO,
	SIPC_CH_ID_CS_VT_VIDEO,
	SIPC_CH_ID_RAW_5,	/* reserved */
	SIPC_CH_ID_RAW_6,	/* reserved */
	SIPC_CH_ID_CDMA_DATA,
	SIPC_CH_ID_PCM_DATA,
	SIPC_CH_ID_TRANSFER_SCREEN,
	SIPC_CH_ID_PDP_0 = 10,
	SIPC_CH_ID_PDP_1,
	SIPC_CH_ID_PDP_2,
	SIPC_CH_ID_PDP_3,
	SIPC_CH_ID_PDP_4,
	SIPC_CH_ID_PDP_5,
	SIPC_CH_ID_PDP_6,
	SIPC_CH_ID_PDP_7,
	SIPC_CH_ID_PDP_8,
	SIPC_CH_ID_PDP_9,
	SIPC_CH_ID_PDP_10,
	SIPC_CH_ID_PDP_11,
	SIPC_CH_ID_PDP_12,
	SIPC_CH_ID_PDP_13,
	SIPC_CH_ID_PDP_14,
	SIPC_CH_ID_BT_DUN,	/* 25 */
	SIPC_CH_ID_CIQ_DATA,
	SIPC_CH_ID_PDP_17,	/* reserved */
	SIPC_CH_ID_CPLOG1,	/* 28 */
	SIPC_CH_ID_CPLOG2,	/* 29 */
	SIPC_CH_ID_LOOPBACK1,	/* 30 */
	SIPC_CH_ID_LOOPBACK2,	/* 31 */
	SIPC_CH_ID_CALIBCMD,	/* 32 */
	SIPC_CH_ID_DOD,		/* 33 */

	SIPC5_CH_ID_BOOT_0 = 215,
	SIPC5_CH_ID_BOOT_1,
	SIPC5_CH_ID_BOOT_2,
	SIPC5_CH_ID_BOOT_3,
	SIPC5_CH_ID_BOOT_4,
	SIPC5_CH_ID_BOOT_5,
	SIPC5_CH_ID_BOOT_6,
	SIPC5_CH_ID_BOOT_7,
	SIPC5_CH_ID_BOOT_8,
	SIPC5_CH_ID_BOOT_9,

	SIPC5_CH_ID_DUMP_0 = 225,
	SIPC5_CH_ID_DUMP_1,
	SIPC5_CH_ID_DUMP_2,
	SIPC5_CH_ID_DUMP_3,
	SIPC5_CH_ID_DUMP_4,
	SIPC5_CH_ID_DUMP_5,
	SIPC5_CH_ID_DUMP_6,
	SIPC5_CH_ID_DUMP_7,
	SIPC5_CH_ID_DUMP_8,
	SIPC5_CH_ID_DUMP_9,

	SIPC5_CH_ID_FMT_0 = 235,
	SIPC5_CH_ID_FMT_1,
	SIPC5_CH_ID_FMT_2,
	SIPC5_CH_ID_FMT_3,
	SIPC5_CH_ID_FMT_4,
	SIPC5_CH_ID_FMT_5,
	SIPC5_CH_ID_FMT_6,
	SIPC5_CH_ID_FMT_7,
	SIPC5_CH_ID_FMT_8,
	SIPC5_CH_ID_FMT_9,

	SIPC5_CH_ID_RFS_0 = 245,
	SIPC5_CH_ID_RFS_1,
	SIPC5_CH_ID_RFS_2,
	SIPC5_CH_ID_RFS_3,
	SIPC5_CH_ID_RFS_4,
	SIPC5_CH_ID_RFS_5,
	SIPC5_CH_ID_RFS_6,
	SIPC5_CH_ID_RFS_7,
	SIPC5_CH_ID_RFS_8,
	SIPC5_CH_ID_RFS_9,

	SIPC5_CH_ID_MAX = 255
};

struct __packed multi_frame_control {
	u8 id:7,
	   more:1;
};

enum direction {
	TX = 0,
	UL = 0,
	AP2CP = 0,
	RX = 1,
	DL = 1,
	CP2AP = 1,
	TXRX = 2,
	ULDL = 2,
	MAX_DIR = 2
};

enum read_write {
	RD = 0,
	WR = 1,
	RDWR = 2
};

/* Bits of the "mif,link_attrs" DT property */
enum link_attr_bit {
	LINK_ATTR_SBD_IPC,	/* IPC over SBD (from MIPI-LLI)		*/
	LINK_ATTR_IPC_ALIGNED,	/* IPC with 4-bytes alignment		*/
	LINK_ATTR_IOSM_MESSAGE,	/* IOSM message (XMM7260, unused)	*/
	LINK_ATTR_DPRAM_MAGIC,	/* DPRAM-style magic code validation	*/
	LINK_ATTR_SBD_BOOT,	/* BOOT over SBD			*/
	LINK_ATTR_SBD_DUMP,	/* DUMP over SBD			*/
	LINK_ATTR_MEM_BOOT,	/* BOOT over legacy memory-type I/F	*/
	LINK_ATTR_MEM_DUMP,	/* DUMP over legacy memory-type I/F	*/
	LINK_ATTR_BOOT_ALIGNED,	/* BOOT with 4-bytes alignment		*/
	LINK_ATTR_DUMP_ALIGNED,	/* DUMP with 4-bytes alignment		*/
	LINK_ATTR_XMIT_BTDLR,	/* Used to download CP bootloader	*/
};
#define LINK_ATTR(b)	(0x1 << (b))

/* Bits of the "attrs" property of each iodev */
enum iodev_attr_bit {
	ATTR_SIPC4,
	ATTR_SIPC5,
	ATTR_CDC_NCM,
	ATTR_MULTIFMT,
	ATTR_HANDOVER,
	ATTR_LEGACY_RFS,
	ATTR_RX_FRAGMENT,
	ATTR_SBD_IPC,		/* IPC using SBD designed from MIPI-LLI	*/
	ATTR_NO_LINK_HEADER,	/* Link-layer header is not needed	*/
	ATTR_NO_CHECK_MAXQ,	/* no need to check rxq overflow	*/
};
#define IODEV_ATTR(b)	(0x1 << (b))

struct modem_io_t {
	const char *name;
	u32 id;
	enum dev_format format;
	enum modem_io io_type;
	u32 links;
	enum modem_link tx_link;
	u32 attrs;
	const char *app;
	unsigned int ul_num_buffers;
	unsigned int ul_buffer_size;
	unsigned int dl_num_buffers;
	unsigned int dl_buffer_size;
};

/*
 * Board data, filled from the "samsung,ss333-modem" DT node.
 * gpio_cp_status and the link-PM "cp2ap_status" are the same pin (gpa1-0)
 * on this board, so there is one descriptor for both.
 */
struct modem_data {
	const char *name;

	struct gpio_desc *gpio_cp_on;
	struct gpio_desc *gpio_cp_reset;
	struct gpio_desc *gpio_pda_active;
	struct gpio_desc *gpio_phone_active;
	struct gpio_desc *gpio_ipc_int2cp;	/* optional */
	struct gpio_desc *gpio_ap_dump_int;
	struct gpio_desc *gpio_cp_dump_int;

	/* link PM */
	struct gpio_desc *gpio_ap_wakeup;	/* cp2ap_wakeup, input */
	struct gpio_desc *gpio_cp_wakeup;	/* ap2cp_wakeup, output */
	struct gpio_desc *gpio_cp_status;	/* cp2ap_status, input */
	struct gpio_desc *gpio_ap_status;	/* ap2cp_status, output */

	enum modem_network modem_net;
	enum modem_t modem_type;
	u32 link_types;
	const char *link_name;
	unsigned long link_attrs;
	enum sipc_ver ipc_version;
	unsigned int max_ipc_dev;
	unsigned int num_iodevs;
	struct modem_io_t *iodevs;
	bool use_handover;
};

struct modem_irq {
	spinlock_t lock;
	unsigned int num;
	char name[MAX_NAME_LEN];
	unsigned long flags;
	bool active;
};

#define mif_dt_read_enum(np, prop, dest) \
	do { \
		u32 val; \
		if (of_property_read_u32(np, prop, &val)) \
			return -EINVAL; \
		dest = (__typeof__(dest))(val); \
	} while (0)

#define mif_dt_read_bool(np, prop, dest) \
	do { \
		u32 val; \
		if (of_property_read_u32(np, prop, &val)) \
			return -EINVAL; \
		dest = val ? true : false; \
	} while (0)

#define mif_dt_read_string(np, prop, dest) \
	do { \
		if (of_property_read_string(np, prop, &(dest))) \
			return -EINVAL; \
	} while (0)

#define mif_dt_read_u32(np, prop, dest) \
	do { \
		u32 val; \
		if (of_property_read_u32(np, prop, &val)) \
			return -EINVAL; \
		dest = val; \
	} while (0)

#define LOG_TAG	"mif: "
#define FUNC	(__func__)
#define CALLER	(__builtin_return_address(0))

#define mif_err_limited(fmt, ...) \
	printk_ratelimited(KERN_ERR LOG_TAG "%s: " fmt, __func__, ##__VA_ARGS__)
#define mif_err(fmt, ...) \
	pr_err(LOG_TAG "%s: " fmt, __func__, ##__VA_ARGS__)
#define mif_info(fmt, ...) \
	pr_info(LOG_TAG "%s: " fmt, __func__, ##__VA_ARGS__)
#define mif_debug(fmt, ...) \
	pr_debug(LOG_TAG "%s: " fmt, __func__, ##__VA_ARGS__)

/* Crash reasons reported through the CP crash notifier */
enum modemctl_event {
	MDM_EVENT_CP_FORCE_RESET,
	MDM_EVENT_CP_FORCE_CRASH,
	MDM_EVENT_CP_ABNORMAL_RX,
	MDM_CRASH_PM_FAIL,
	MDM_CRASH_PM_CP_FAIL,
	MDM_CRASH_INVALID_RB,
	MDM_CRASH_INVALID_IOD,
	MDM_CRASH_INVALID_SKBCB,
	MDM_CRASH_INVALID_SKBIOD,
	MDM_CRASH_NO_MEM,
	MDM_CRASH_PACKET_CRACKED,
	MDM_CRASH_BY_IOSM,
	MDM_CRASH_CMD_RESET = 90,
	MDM_CRASH_CMD_EXIT,
};

struct notifier_block;
int register_cp_crash_notifier(struct notifier_block *nb);
int unregister_cp_crash_notifier(struct notifier_block *nb);
void modemctl_notify_event(enum modemctl_event evt);

/* Modem state events for in-kernel listeners (values follow modem_state) */
enum modem_event {
	MODEM_STATE_OFFLINE,
	MODEM_EVENT_RESET,
	MODEM_EVENT_EXIT,
	MODEM_EVENT_ONLINE	= 4,
	MODEM_EVENT_WATCHDOG	= 9,
};
int register_modem_event_notifier(struct notifier_block *nb);
void modem_notify_event(enum modem_event evt);

#endif
