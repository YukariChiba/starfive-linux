/*
 *  Bluetooth support for Realtek devices
 *
 *  Copyright (C) 2015 Endless Mobile, Inc.
 *  Copyright (C) 2018 Realtek Semiconductor Corp
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 */

#define RTL_FRAG_LEN 252
#define BTCOEX
#define CONFIG_COEX
/* #define CONFIG_TEST */
#define RTKBTSDIO_LPS

struct btsdio_data {
	struct hci_dev   *hdev;
	struct sdio_func *func;

	struct work_struct work;

	struct sk_buff_head txq;
	int int_count;

	struct task_struct *thread;
	wait_queue_head_t wq;

	spinlock_t lock;

#ifdef CONFIG_TEST
	struct delayed_work timer_loop;
	struct delayed_work timer_delay;
	struct workqueue_struct *test_workqueue;
#endif

#ifdef RTKBTSDIO_LPS
	unsigned long last_busy;
#endif
};

struct rtl_download_cmd {
	__u8 index;
	__u8 data[RTL_FRAG_LEN];
} __packed;

struct rtl_download_response {
	__u8 status;
	__u8 index;
} __packed;

struct rtl_rom_version_evt {
	__u8 status;
	__u8 version;
} __packed;

struct rtl_epatch_header {
	__u8 signature[8];
	__le32 fw_version;
	__le16 num_patches;
} __packed;

int btrtl_setup_realtek(struct hci_dev *hdev);
