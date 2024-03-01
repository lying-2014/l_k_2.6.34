/*******************************************************************************

  Intel PRO/1000 Linux driver
  Copyright(c) 1999 - 2006 Intel Corporation.

  This program is free software; you can redistribute it and/or modify it
  under the terms and conditions of the GNU General Public License,
  version 2, as published by the Free Software Foundation.

  This program is distributed in the hope it will be useful, but WITHOUT
  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
  more details.

  You should have received a copy of the GNU General Public License along with
  this program; if not, write to the Free Software Foundation, Inc.,
  51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.

  The full GNU General Public License is included in this distribution in
  the file called "COPYING".

  Contact Information:
  Linux NICS <linux.nics@intel.com>
  e1000-devel Mailing List <e1000-devel@lists.sourceforge.net>
  Intel Corporation, 5200 N.E. Elam Young Parkway, Hillsboro, OR 97124-6497

*******************************************************************************/


/* Linux PRO/1000 Ethernet Driver main header file */

#ifndef _E1000_H_
#define _E1000_H_

#include <linux/stddef.h>
#include <linux/module.h>
#include <linux/types.h>
#include <asm/byteorder.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/errno.h>
#include <linux/ioport.h>
#include <linux/pci.h>
#include <linux/kernel.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/delay.h>
#include <linux/timer.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/interrupt.h>
#include <linux/string.h>
#include <linux/pagemap.h>
#include <linux/dma-mapping.h>
#include <linux/bitops.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <linux/capability.h>
#include <linux/in.h>
#include <linux/ip.h>
#include <linux/ipv6.h>
#include <linux/tcp.h>
#include <linux/udp.h>
#include <net/pkt_sched.h>
#include <linux/list.h>
#include <linux/reboot.h>
#include <net/checksum.h>
#include <linux/mii.h>
#include <linux/ethtool.h>
#include <linux/if_vlan.h>

#define BAR_0		0
#define BAR_1		1
#define BAR_5		5

#define INTEL_E1000_ETHERNET_DEVICE(device_id) {\
	PCI_DEVICE(PCI_VENDOR_ID_INTEL, device_id)}

struct e1000_adapter;

#include "e1000_hw.h"

#ifdef DBG
#define E1000_DBG(args...) printk(KERN_DEBUG "e1000: " args)
#else
#define E1000_DBG(args...)
#endif

#define E1000_ERR(args...) printk(KERN_ERR "e1000: " args)

#define PFX "e1000: "

#define DPRINTK(nlevel, klevel, fmt, args...)				\
do {									\
	if (NETIF_MSG_##nlevel & adapter->msg_enable)			\
		printk(KERN_##klevel PFX "%s: %s: " fmt,		\
		       adapter->netdev->name, __func__, ##args);	\
} while (0)

#define E1000_MAX_INTR 10

/* TX/RX descriptor defines */
#define E1000_DEFAULT_TXD                  256
#define E1000_MAX_TXD                      256
#define E1000_MIN_TXD                       80
#define E1000_MAX_82544_TXD               4096

#define E1000_DEFAULT_RXD                  256
#define E1000_MAX_RXD                      256
#define E1000_MIN_RXD                       80
#define E1000_MAX_82544_RXD               4096

#define E1000_MIN_ITR_USECS		10 /* 100000 irq/sec */
#define E1000_MAX_ITR_USECS		10000 /* 100    irq/sec */

/* this is the size past which hardware will drop packets when setting LPE=0 */
#define MAXIMUM_ETHERNET_VLAN_SIZE 1522

/* Supported Rx Buffer Sizes */
#define E1000_RXBUFFER_128   128    /* Used for packet split */
#define E1000_RXBUFFER_256   256    /* Used for packet split */
#define E1000_RXBUFFER_512   512
#define E1000_RXBUFFER_1024  1024
#define E1000_RXBUFFER_2048  2048
#define E1000_RXBUFFER_4096  4096
#define E1000_RXBUFFER_8192  8192
#define E1000_RXBUFFER_16384 16384

/* SmartSpeed delimiters */
#define E1000_SMARTSPEED_DOWNSHIFT 3
#define E1000_SMARTSPEED_MAX       15

/* Packet Buffer allocations */
#define E1000_PBA_BYTES_SHIFT 0xA
#define E1000_TX_HEAD_ADDR_SHIFT 7
#define E1000_PBA_TX_MASK 0xFFFF0000

/* Flow Control Watermarks */
#define E1000_FC_HIGH_DIFF 0x1638  /* High: 5688 bytes below Rx FIFO size */
#define E1000_FC_LOW_DIFF 0x1640   /* Low:  5696 bytes below Rx FIFO size */

#define E1000_FC_PAUSE_TIME 0xFFFF /* pause for the max or until send xon */

/* How many Tx Descriptors do we need to call netif_wake_queue ? */
#define E1000_TX_QUEUE_WAKE	16
/* How many Rx Buffers do we bundle into one write to the hardware ? */
#define E1000_RX_BUFFER_WRITE	16	/* Must be power of 2 */

#define AUTO_ALL_MODES            0
#define E1000_EEPROM_82544_APM    0x0004
#define E1000_EEPROM_APME         0x0400

#ifndef E1000_MASTER_SLAVE
/* Switch to override PHY master/slave setting */
#define E1000_MASTER_SLAVE	e1000_ms_hw_default	//ms=master slave
#endif

#define E1000_MNG_VLAN_NONE (-1)

/* wrapper around a pointer to a socket buffer,
 * so a DMA handle can be stored along with the buffer */
struct e1000_buffer {
	struct sk_buff *skb;
	dma_addr_t dma;
	struct page *page;
	unsigned long time_stamp;
	u16 length;
	u16 next_to_watch;
	u16 mapped_as_page;
};

struct e1000_tx_ring {//发送描述符环，是用于存储和管理网络数据包发送操作的数据结构；注意是“操作”不是“数据包”
	/* pointer to the descriptor ring memory */
	void *desc;//描述符环内存的指针；描述符是用于网络数据包传输，包含了物理数据包的位置和状态信息。
	/* physical address of the descriptor ring */
	dma_addr_t dma;//描述符环的物理地址；DMA操作使用的是这个地址。
	/* length of descriptor ring in bytes */
	unsigned int size;描述符环的总长度；它决定了环中可以存放多少描述符。
	/* number of descriptors in the ring */
	unsigned int count;环中描述符的数量；对应着环可以处理的最大数据包数量
	/* next descriptor to associate a buffer with */
	unsigned int next_to_use;环中下一个将被用于关联数据包的描述符的索引。驱动程序会在此位置放置新的数据包以进行传输[应该是把skb_buff的指针放到对应的描述符上]。
	/* next descriptor to check for DD status bit */
	unsigned int next_to_clean;环中下一个需要检查是否传输完成的描述符的索引。驱动程序会检查这个位置的描述符来确定数据包是否已成功发送。
	/* array of buffer information structs */
	struct e1000_buffer *buffer_info;指向e1000缓冲区信息结构体数组的指针。每个缓冲区信息结构体与描述符环中的一个描述符相关联。;为啥每个描述符还要配一个缓冲区信息结构体呢？

	u16 tdh;通常用来表示硬件的“头”位置，在某些设备上用于同步硬件和驱动程序的状态；不理解
	u16 tdt;通常表示硬件的“尾”位置，用于告知硬件新的数据包已经放置在环中并准备发送；不理解
	bool last_tx_tso;标记最后一个进行TCP分段卸载（TSO）的数据包
};

struct e1000_rx_ring {接收环用于存储和管理网络数据包接收操作的数据结构
	/* pointer to the descriptor ring memory */
	void *desc;指向描述符环内存的指针，描述符环在哪；每个描述符包含了数据包的位置和状态信息
	/* physical address of the descriptor ring */
	dma_addr_t dma;描述符环的物理地址；上一个是描述符环的驱动程序虚拟内存空间地址，二者不同的用途。
	/* length of descriptor ring in bytes */
	unsigned int size;描述符环的总长度，单位为字节
	/* number of descriptors in the ring */
	unsigned int count;环中描述符的数量，从而定义了环可以处理的最大数据包数量。
	/* next descriptor to associate a buffer with */
	unsigned int next_to_use;环中下一个将被用于关联数据包的描述符的索引。驱动程序会在此位置放置新的数据包以进行接收
	/* next descriptor to check for DD status bit */
	unsigned int next_to_clean;环中下一个需要检查是否接收完成的描述符的索引。驱动程序会检查这个位置的描述符来确定数据包是否已成功接收。
	/* array of buffer information structs */
	struct e1000_buffer *buffer_info;指向e1000缓冲区信息结构体数组的指针。每个缓冲区信息结构体与描述符环中的一个描述符相关联。
	struct sk_buff *rx_skb_top;通常用于存储接收到的数据包；为什么不在描述符中存储呢？

	/* cpu for rx queue */
	int cpu;处理该接收队列的CPU

	u16 rdh;表示硬件“头”位置的索引。它用于跟踪硬件当前处理的描述符位置。不理解
	u16 rdt;表示硬件“尾”位置的索引。它用于通知硬件新的数据包已经放置在环中并准备接收。不理解
};这个变量是对 描述符ring 进行描述，即 rx描述符ring的 相关信息。

#define E1000_DESC_UNUSED(R)						\
	((((R)->next_to_clean > (R)->next_to_use)			\
	  ? 0 : (R)->count) + (R)->next_to_clean - (R)->next_to_use - 1)

#define E1000_RX_DESC_EXT(R, i)						\
	(&(((union e1000_rx_desc_extended *)((R).desc))[i]))
#define E1000_GET_DESC(R, i, type)	(&(((struct type *)((R).desc))[i]))
#define E1000_RX_DESC(R, i)		E1000_GET_DESC(R, i, e1000_rx_desc)
#define E1000_TX_DESC(R, i)		E1000_GET_DESC(R, i, e1000_tx_desc)
#define E1000_CONTEXT_DESC(R, i)	E1000_GET_DESC(R, i, e1000_context_desc)

/* board specific private data structure */

struct e1000_adapter {
	struct timer_list tx_fifo_stall_timer;
	struct timer_list watchdog_timer;
	struct timer_list phy_info_timer;
	struct vlan_group *vlgrp;
	u16 mng_vlan_id;
	u32 bd_number;
	u32 rx_buffer_len;
	u32 wol;
	u32 smartspeed;
	u32 en_mng_pt;
	u16 link_speed;
	u16 link_duplex;
	spinlock_t stats_lock;
	unsigned int total_tx_bytes;
	unsigned int total_tx_packets;
	unsigned int total_rx_bytes;
	unsigned int total_rx_packets;
	/* Interrupt Throttle Rate */
	u32 itr;
	u32 itr_setting;
	u16 tx_itr;
	u16 rx_itr;

	struct work_struct reset_task;
	u8 fc_autoneg;

	struct timer_list blink_timer;
	unsigned long led_status;

	/* TX */
	struct e1000_tx_ring *tx_ring;      /* One per active queue */
	unsigned int restart_queue;
	u32 txd_cmd;
	u32 tx_int_delay;
	u32 tx_abs_int_delay;
	u32 gotcl;
	u64 gotcl_old;
	u64 tpt_old;
	u64 colc_old;
	u32 tx_timeout_count;
	u32 tx_fifo_head;
	u32 tx_head_addr;
	u32 tx_fifo_size;
	u8  tx_timeout_factor;
	atomic_t tx_fifo_stall;
	bool pcix_82544;
	bool detect_tx_hung;

	/* RX */
	bool (*clean_rx)(struct e1000_adapter *adapter,
			 struct e1000_rx_ring *rx_ring,
			 int *work_done, int work_to_do);
	void (*alloc_rx_buf)(struct e1000_adapter *adapter,
			     struct e1000_rx_ring *rx_ring,
			     int cleaned_count);
	struct e1000_rx_ring *rx_ring;      /* One per active queue */
	struct napi_struct napi;

	int num_tx_queues;
	int num_rx_queues;

	u64 hw_csum_err;
	u64 hw_csum_good;
	u32 alloc_rx_buff_failed;
	u32 rx_int_delay;
	u32 rx_abs_int_delay;
	bool rx_csum;
	u32 gorcl;
	u64 gorcl_old;

	/* OS defined structs */
	struct net_device *netdev;
	struct pci_dev *pdev;

	/* structs defined in e1000_hw.h */
	struct e1000_hw hw;
	struct e1000_hw_stats stats;
	struct e1000_phy_info phy_info;
	struct e1000_phy_stats phy_stats;

	u32 test_icr;
	struct e1000_tx_ring test_tx_ring;
	struct e1000_rx_ring test_rx_ring;

	int msg_enable;

	/* to not mess up cache alignment, always add to the bottom */
	bool tso_force;
	bool smart_power_down;	/* phy smart power down */
	bool quad_port_a;
	unsigned long flags;
	u32 eeprom_wol;

	/* for ioport free */
	int bars;
	int need_ioport;

	bool discarding;
};

enum e1000_state_t {
	__E1000_TESTING,
	__E1000_RESETTING,
	__E1000_DOWN
};

extern char e1000_driver_name[];
extern const char e1000_driver_version[];

extern int e1000_up(struct e1000_adapter *adapter);
extern void e1000_down(struct e1000_adapter *adapter);
extern void e1000_reinit_locked(struct e1000_adapter *adapter);
extern void e1000_reset(struct e1000_adapter *adapter);
extern int e1000_set_spd_dplx(struct e1000_adapter *adapter, u16 spddplx);
extern int e1000_setup_all_rx_resources(struct e1000_adapter *adapter);
extern int e1000_setup_all_tx_resources(struct e1000_adapter *adapter);
extern void e1000_free_all_rx_resources(struct e1000_adapter *adapter);
extern void e1000_free_all_tx_resources(struct e1000_adapter *adapter);
extern void e1000_update_stats(struct e1000_adapter *adapter);
extern bool e1000_has_link(struct e1000_adapter *adapter);
extern void e1000_power_up_phy(struct e1000_adapter *);
extern void e1000_set_ethtool_ops(struct net_device *netdev);
extern void e1000_check_options(struct e1000_adapter *adapter);

#endif /* _E1000_H_ */
