
#include <linux/module.h>
#include <linux/kobject.h>
#include <linux/firmware.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <asm/uaccess.h>
#include <asm/unaligned.h>


#include "MMS100S_ISC_Updater_Customize.h"
#include "MMS100S_ISC_Updater.h"

/*
 * ISC_XFER_LEN	- ISC unit transfer length.
 * Give number of 2 power n, where  n is between 2 and 10
 * i.e. 4, 8, 16 ,,, 1024
 */
#define ISC_XFER_LEN		256//1024

#define MMS_FLASH_PAGE_SZ	1024
#define ISC_BLOCK_NUM		(MMS_FLASH_PAGE_SZ / ISC_XFER_LEN)

#define FLASH_VERBOSE_DEBUG	1
#define MAX_SECTION_NUM		3

/* Registers */
#define MMS_CMD_ENTER_ISC	0x5F
#define MMS_FW_VERSION		0xE1

/* Firmware file name */
//#define FW_NAME			"tsp_melfas/mms_cs02_gf1.fw"
//const char *fw_name = FW_NAME;

//"mms_ts.fw"

enum {
	ISC_ADDR		= 0xD5,

	ISC_CMD_READ_STATUS	= 0xD9,
	ISC_CMD_READ		= 0x4000,
	ISC_CMD_EXIT		= 0x8200,
	ISC_CMD_PAGE_ERASE	= 0xC000,
	ISC_CMD_ERASE			= 0xC100,
	ISC_PAGE_ERASE_DONE	= 0x10000,
	ISC_PAGE_ERASE_ENTER	= 0x20000,
};

struct mms_bin_hdr {
	char	tag[8];
	u16	core_version;
	u16	section_num;
	u16	contains_full_binary;
	u16	reserved0;

	u32	binary_offset;
	u32	binary_length;

	u32	extention_offset;
	u32	reserved1;

} __attribute__ ((packed));

struct mms_fw_img {
	u16	type;
	u16	version;

	u16	start_page;
	u16	end_page;

	u32	offset;
	u32	length;

} __attribute__ ((packed));

struct isc_packet {
	u8	cmd;
	u32	addr;
	u8	data[0];
} __attribute__ ((packed));


const struct firmware *fw =NULL;


static int mms_isc_read_status(struct i2c_client *client, u32 val);
static int mms_isc_transfer_cmd(struct i2c_client *client, int cmd);
static int mms_isc_erase_page(struct i2c_client *client, int page);
static int mms_isc_enter(struct i2c_client *client);
static int mms_isc_exit(struct i2c_client *client);
static int mms_flash_section(struct i2c_client *client, struct mms_fw_img *img, const u8 *data);
static int get_fw_version(struct i2c_client *client, u8 *buf);
static int mms_flash_fw(struct i2c_client *client);

static int mms_isc_read_status(struct i2c_client *client, u32 val)
{
	u8 cmd = ISC_CMD_READ_STATUS;
	u32 result = 0;
	int cnt = 100;
	int ret = 0;

	do {
		i2c_smbus_read_i2c_block_data(client, cmd, 4, (u8 *)&result);
		if (result == val)
			break;
		msleep(1);
	} while (--cnt);

	if (!cnt) {
		dev_err(&client->dev,
			"status read fail. cnt : %d, val : 0x%x != 0x%x\n",
			cnt, result, val);
	}
	return ret;
}

static int mms_isc_transfer_cmd(struct i2c_client *client, int cmd)
{
	struct isc_packet pkt = { ISC_ADDR, cmd };
	struct i2c_msg msg = {
		.addr = client->addr,
		.flags = 0,
		.len = sizeof(struct isc_packet),
		.buf = (u8 *)&pkt,
	};

	return (i2c_transfer(client->adapter, &msg, 1) != 1);
}

static int mms_isc_erase_page(struct i2c_client *client, int page)
{
	return mms_isc_transfer_cmd(client, ISC_CMD_PAGE_ERASE | page) ||
		mms_isc_read_status(client, ISC_PAGE_ERASE_DONE | ISC_PAGE_ERASE_ENTER | page);
}

static int mms_isc_enter(struct i2c_client *client)
{
	return i2c_smbus_write_byte_data(client, MMS_CMD_ENTER_ISC, true);
}

static int mms_isc_exit(struct i2c_client *client)
{
	return mms_isc_transfer_cmd(client, ISC_CMD_EXIT);
}

static int mms_flash_section(struct i2c_client *client, struct mms_fw_img *img, const u8 *data)
{
	struct isc_packet *isc_packet;
	int ret;
	int page, i;
	struct i2c_msg msg[2] = {
		{
			.addr = client->addr,
			.flags = 0,
		}, {
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = ISC_XFER_LEN,
		},
	};
	int ptr = img->offset;

	isc_packet = kzalloc(sizeof(*isc_packet) + ISC_XFER_LEN, GFP_KERNEL);
	isc_packet->cmd = ISC_ADDR;

	msg[0].buf = (u8 *)isc_packet;
	msg[1].buf = kzalloc(ISC_XFER_LEN, GFP_KERNEL);

	for (page = img->start_page; page <= img->end_page; page++) {
		mms_isc_erase_page(client, page);

		for (i = 0; i < ISC_BLOCK_NUM; i++, ptr += ISC_XFER_LEN) {
			/* flash firmware */
			u32 tmp = page * 256 + i * (ISC_XFER_LEN / 4);
			put_unaligned_le32(tmp, &isc_packet->addr);
			msg[0].len = sizeof(struct isc_packet) + ISC_XFER_LEN;

			memcpy(isc_packet->data, data + ptr, ISC_XFER_LEN);
			if (i2c_transfer(client->adapter, msg, 1) != 1)
				goto i2c_err;

			/* verify firmware */
			tmp |= ISC_CMD_READ;
			put_unaligned_le32(tmp, &isc_packet->addr);
			msg[0].len = sizeof(struct isc_packet);

			if (i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg)) != ARRAY_SIZE(msg))
				goto i2c_err;

			if (memcmp(isc_packet->data, msg[1].buf, ISC_XFER_LEN)) {
#if FLASH_VERBOSE_DEBUG
				print_hex_dump(KERN_ERR, "mms fw wr : ",
						DUMP_PREFIX_OFFSET, 16, 1,
						isc_packet->data, ISC_XFER_LEN, false);

				print_hex_dump(KERN_ERR, "mms fw rd : ",
						DUMP_PREFIX_OFFSET, 16, 1,
						msg[1].buf, ISC_XFER_LEN, false);
#endif
				dev_err(&client->dev, "flash verify failed\n");
				ret = -1;
				goto out;
			}

		}
	}

	dev_info(&client->dev, "section [%d] update succeeded\n", img->type);

	ret = 0;
	goto out;

i2c_err:
	dev_err(&client->dev, "i2c failed @ %s\n", __func__);
	ret = -1;

out:
	kfree(isc_packet);
	kfree(msg[1].buf);

	return ret;
}

#if defined(CONFIG_TOUCHSCREEN_MELFAS_ARUBA3G)
unsigned char TSP_PanelVersion[3], TSP_PhoneVersion[3];
#else
unsigned char TSP_PanelVersion, TSP_PhoneVersion;
#endif

static int get_fw_version(struct i2c_client *client, u8 *buf)
{
	u8 cmd = MMS_FW_VERSION;
	struct i2c_msg msg[2] = {
		{
			.addr = client->addr,
			.flags = 0,
			.buf = &cmd,
			.len = 1,
		}, {
			.addr = client->addr,
			.flags = I2C_M_RD,
			.buf = buf,
			.len = MAX_SECTION_NUM,
		},
	};
	return (i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg)) != ARRAY_SIZE(msg));
}


static int mms_flash_fw(struct i2c_client *client)
{
	int ret =0;
	int i;
	struct mms_bin_hdr *fw_hdr;
	struct mms_fw_img **img;
	u8 ver[MAX_SECTION_NUM];
	u8 target[MAX_SECTION_NUM];
	int offset = sizeof(struct mms_bin_hdr);
	int retires = 3;
	bool update_flag = false;
	bool isc_flag = true;

	fw_hdr = (struct mms_bin_hdr *)fw->data;
	img = kzalloc(sizeof(*img) * fw_hdr->section_num, GFP_KERNEL);

	printk(KERN_ERR "%s,%d FW:(%d,%d,%d,%d) \n", __func__, __LINE__, (u8)fw_hdr->core_version, (u8)fw_hdr->contains_full_binary, (u8)fw_hdr->reserved0, (u8)fw_hdr->section_num);

	while (retires--) {
		if (!get_fw_version(client, ver))
			break;
		else
			MFS_TSP_reboot();
	}

	if (retires < 0) {
		dev_warn(&client->dev, "failed to obtain ver. info\n");
		isc_flag = false;
		memset(ver, 0xff, sizeof(ver));
	} else {
		print_hex_dump(KERN_INFO, "mms_ts fw ver : ", DUMP_PREFIX_NONE, 16, 1,
				ver, MAX_SECTION_NUM, false);
		printk(KERN_ERR "%s,%d IC V:%d,%d,%d \n", __func__, __LINE__, ver[0],ver[1],ver[2]);
	}

	for (i = 0; i < fw_hdr->section_num; i++, offset += sizeof(struct mms_fw_img)) {
		img[i] = (struct mms_fw_img *)(fw->data + offset);
		target[i] = img[i]->version;

		printk(KERN_ERR "%s,%d i=%d, Check IC:%2x, B:%2x \n", __func__, __LINE__,i, (ver[img[i]->type]), target[i]);

		if(i==1){
#if defined(CONFIG_TOUCHSCREEN_MELFAS_ARUBA3G)
			TSP_PanelVersion[2] =  ver[img[i]->type];
			TSP_PhoneVersion[2] = target[i];
#else
			TSP_PanelVersion =  ver[img[i]->type];
			TSP_PhoneVersion = target[i];
#endif
		}

#if 1 // only verion read
		// update condition.
		// 0 : boot : 0xff = crack
		// 1 : core : under bin version,  over bin +10 version, 0x50~0x70 is gff type
		if ( ((i==0)&&(ver[img[i]->type]==0xff)) \
			|| ((i==0)&&(ver[img[i]->type]==0x02)&&(target[i]==0x03))  \
			|| ((i==1)&&((ver[img[i]->type] < target[i]) || (ver[img[i]->type] > (target[i] + 10 )))))
		{


			if(isc_flag){
				mms_isc_enter(client);
				isc_flag = false;
			}
			update_flag = true;
			dev_info(&client->dev,
				"section [%d] is need to be updated. ver : 0x%x, bin : 0x%x\n",
				img[i]->type, ver[img[i]->type], target[i]);

			if ((ret = mms_flash_section(client, img[i], fw->data + fw_hdr->binary_offset))) {
				if(ret==0 && i==1){
#if defined(CONFIG_TOUCHSCREEN_MELFAS_ARUBA3G)
					TSP_PanelVersion[2] = target[i];
#else
					TSP_PanelVersion = target[i];
#endif
					dev_info(&client->dev, "section %d[%d] version changed to %d\n", i, img[i]->type, target[i]);
				}
				MFS_TSP_reboot();
				goto out;
			}
			memset(ver, 0xff, sizeof(ver));
		} else {
			dev_info(&client->dev, "section [%d] do not update\n", i);
		}
	}
	printk(KERN_ERR "%s,%d \n", __func__, __LINE__);

	if (update_flag){
		mms_isc_exit(client);
		msleep(5);
		MFS_TSP_reboot();
	}
#endif

	printk(KERN_ERR "%s,%d \n", __func__, __LINE__);

	if (get_fw_version(client, ver)) {
		dev_err(&client->dev, "failed to obtain version after flash\n");
		ret = -1;
		goto out;
	} else {
		for (i = 0; i < fw_hdr->section_num; i++) {
			if (ver[img[i]->type] != target[i]) {
				dev_info(&client->dev,
					"version mismatch after flash. [%d] 0x%x != 0x%x\n",
					i, ver[img[i]->type], target[i]);

				ret = -1;
				goto out;
			}
		}
	}

	ret = 0;

out:
	kfree(img);

	return ret;
}

eMFSRet_t MFS_ISC_update(struct i2c_client *client)
{
	int retires = 3;
	int ret;

#if defined(CONFIG_TOUCHSCREEN_MELFAS_ARUBA3G)
	const char *fw_name = "tsp_melfas/mms_aruba3g_gf1.fw";
#else
	const char *fw_name = "tsp_melfas/mms_cs02_gf1.fw";
#endif
	ret = request_firmware(&fw, fw_name, &client->dev);
	if (ret){
		dev_err(&client->dev, "request firmware error!!\n");
		return MRET_FILE_OPEN_ERROR;
	}

	do {
		ret = mms_flash_fw(client);
	} while (ret && --retires);

	if (!retires) {
		dev_err(&client->dev, "failed to flash firmware after retires\n");
	}
	release_firmware(fw);
	return ret;
}
