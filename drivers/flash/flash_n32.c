/*
 * Copyright (c) 2023, Quincy.W <wangqyfm@foxmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nations_n32_flash_controller
#define SOC_NV_FLASH_NODE DT_INST(0, soc_nv_flash)

#define FLASH_WRITE_BLK_SZ DT_PROP(SOC_NV_FLASH_NODE, write_block_size)
#define FLASH_ERASE_BLK_SZ DT_PROP(SOC_NV_FLASH_NODE, erase_block_size)

#if defined(N32G45X)
#define _FLASH_BASE_ADDRESS 0x08000000
#elif defined(N32WB03X)
#define _FLASH_BASE_ADDRESS 0x01000000
#endif

#include <soc.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/init.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(flash_n32, CONFIG_FLASH_LOG_LEVEL);

#include <string.h>

#define DEV_DATA(dev) ((struct flash_n32_data *)(dev)->data)

struct flash_n32_data
{
	struct k_sem sem;
};

static const struct flash_parameters flash_n32_parameters = {
	.write_block_size = DT_PROP(SOC_NV_FLASH_NODE, erase_block_size),
	.erase_value = 0xff};

static int flash_n32_read(const struct device *dev, off_t offset,
						  void *data, size_t len)
{
	int rc;

	// 读取最小单位是 4 字节
	if (((offset % 4) != 0) || ((len % 4) != 0) || ((offset + len) > (CONFIG_FLASH_SIZE * 1024)))
	{
		LOG_ERR("Read range invalid. Offset: %ld, len: %zu",
				(long int)offset, len);
		return -EINVAL;
	}

	if (!len)
	{
		return 0;
	}

	LOG_DBG("Read offset: %ld, len: %zu", (long int)offset, len);

#if defined(N32G45X)
	memcpy(data, (void *)offset, len);
	rc = 0;
#elif defined(N32WB03X)
	rc = Qflash_Read(offset, (uint8_t *)data, len) == FlashOperationSuccess ? 0 : -EINVAL;
#endif

	return rc;
}

static int flash_n32_erase(const struct device *dev, off_t offset,
						   size_t len)
{
	struct flash_n32_data *dev_data = DEV_DATA(dev);
	int rc;

	if (((offset % FLASH_ERASE_BLK_SZ) != 0) || ((len % FLASH_ERASE_BLK_SZ) != 0) || ((offset + len) > (CONFIG_FLASH_SIZE * 1024)))
	{
		LOG_ERR("Erase range invalid. Offset: %ld, len: %zu",
				(long int)offset, len);
		return -EINVAL;
	}

	if (!len)
	{
		return 0;
	}

	k_sem_take(&dev_data->sem, K_FOREVER);

	LOG_DBG("Erase offset: %ld, len: %zu", (long int)offset, len);

#if defined(N32G45X)
	FLASH_Unlock();
	rc = FLASH_EraseOnePage(offset + _FLASH_BASE_ADDRESS) == FLASH_COMPL ? 0 : -EINVAL;
	FLASH_Lock();
#elif defined(N32WB03X)
	rc = Qflash_Erase_Sector(offset) == FlashOperationSuccess ? 0 : -EINVAL;
#endif

	k_sem_give(&dev_data->sem);

	return rc;
}

static int flash_n32_write(const struct device *dev, off_t offset,
						   const void *data, size_t len)
{
	struct flash_n32_data *dev_data = DEV_DATA(dev);
	int rc = 0;
	
	if (((offset % FLASH_WRITE_BLK_SZ) != 0) || ((len % 4) != 0) || ((offset + len) > (CONFIG_FLASH_SIZE * 1024)))
	{
		LOG_ERR("Write range invalid. Offset: %ld, len: %zu",
				(long int)offset, len);
		return -EINVAL;
	}

	if (!len)
	{
		return 0;
	}

	k_sem_take(&dev_data->sem, K_FOREVER);

	LOG_DBG("Write offset: %ld, len: %zu", (long int)offset, len);

#if defined(N32G45X)
	uint32_t address = offset + _FLASH_BASE_ADDRESS;
	const uint32_t *ptr = (const uint32_t *)data;

	FLASH_Unlock();

	while((len != 0) && (rc == 0))
	{
		rc = FLASH_ProgramWord(address, *ptr++) == FLASH_COMPL ? 0 : -EINVAL;

		address += sizeof(uint32_t);
		len -= sizeof(uint32_t);
	}

	FLASH_Lock();
#elif defined(N32WB03X)
	rc = Qflash_Write(offset, (uint8_t *)data, len) == FlashOperationSuccess? 0 : -EINVAL;
#endif

	k_sem_give(&dev_data->sem);

	return rc;
}

static const struct flash_parameters *
flash_n32_get_parameters(const struct device *dev)
{
	ARG_UNUSED(dev);

	return &flash_n32_parameters;
}

#if CONFIG_FLASH_PAGE_LAYOUT
static const struct flash_pages_layout flash_n32_pages_layout = {
	.pages_count = DT_REG_SIZE(SOC_NV_FLASH_NODE) / FLASH_ERASE_BLK_SZ,
	.pages_size = DT_PROP(SOC_NV_FLASH_NODE, erase_block_size),
};

void flash_n32_page_layout(const struct device *dev,
						   const struct flash_pages_layout **layout,
						   size_t *layout_size)
{
	*layout = &flash_n32_pages_layout;
	*layout_size = 1;
}
#endif /* CONFIG_FLASH_PAGE_LAYOUT */

static int n32_flash_init(const struct device *dev)
{
	struct flash_n32_data *dev_data = DEV_DATA(dev);

	// LOG_INF("Flash block size: %d, write size: %d", FLASH_ERASE_BLK_SZ, FLASH_WRITE_BLK_SZ);

	k_sem_init(&dev_data->sem, 1, 1);

#if defined(N32G45X)
	;
#elif defined(N32WB03X)
	Qflash_Init();
#endif

	return 0;
}

static const struct flash_driver_api flash_n32_api = {
	.erase = flash_n32_erase,
	.write = flash_n32_write,
	.read = flash_n32_read,
	.get_parameters = flash_n32_get_parameters,
#ifdef CONFIG_FLASH_PAGE_LAYOUT
	.page_layout = flash_n32_page_layout,
#endif
};

static struct flash_n32_data flash_data;

DEVICE_DT_INST_DEFINE(0, n32_flash_init, NULL,
					  &flash_data, NULL, POST_KERNEL,
					  CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &flash_n32_api);
