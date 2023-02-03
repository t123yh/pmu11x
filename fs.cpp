//
// Created by 田韵豪 on 2023/1/30.
//

#include <pico/platform.h>
#include "fs.h"
#include "littlefs/lfs.h"
#include <FreeRTOS.h>
#include <semphr.h>
#include <utils.h>
#include "ws/mongoose.h"
#include <hardware/flash.h>
#include <hardware/structs/ssi.h>
#include <hardware/structs/dma.h>
#include <hardware/regs/dreq.h>
#include <hardware/dma.h>

// variables used by the filesystem
lfs_t lfs_instance;

// Begin at 12MB
const size_t kFlashStartSector = 3072;
const size_t kFlashStartOffset = FLASH_SECTOR_SIZE * kFlashStartSector;

static SemaphoreHandle_t lfsMutex;

// This example DMAs 16kB of data from the start of flash to SRAM, and
// measures the transfer speed.
//
// The SSI (flash interface) inside the XIP block has DREQ logic, so we can
// DMA directly from its FIFOs. Unlike the XIP stream hardware (see
// flash_xip_stream.c) this can *not* be done whilst code is running from
// flash, without careful footwork like we do here. The tradeoff is that it's
// ~2.5x as fast in QSPI mode, ~2x as fast in SPI mode.

static void __no_inline_not_in_flash_func(flash_bulk_read)(uint32_t *rxbuf, uint32_t flash_offs, size_t len,
                                                           uint dma_chan) {
  CRITICAL_BLOCK;
  // SSI must be disabled to set transfer size. If software is executing
  // from flash right now then it's about to have a bad time
  ssi_hw->ssienr = 0;
  ssi_hw->ctrlr1 = len - 1; // NDF, number of data frames
  ssi_hw->dmacr = SSI_DMACR_TDMAE_BITS | SSI_DMACR_RDMAE_BITS;
  ssi_hw->ssienr = 1;
  // Other than NDF, the SSI configuration used for XIP is suitable for a bulk read too.

  // Configure and start the DMA. Note we are avoiding the dma_*() functions
  // as we can't guarantee they'll be inlined
  dma_hw->ch[dma_chan].read_addr = (uint32_t) &ssi_hw->dr0;
  dma_hw->ch[dma_chan].write_addr = (uint32_t) rxbuf;
  dma_hw->ch[dma_chan].transfer_count = len;
  // Must enable DMA byteswap because non-XIP 32-bit flash transfers are
  // big-endian on SSI (we added a hardware tweak to make XIP sensible)
  dma_hw->ch[dma_chan].ctrl_trig =
      DMA_CH0_CTRL_TRIG_BSWAP_BITS |
      DREQ_XIP_SSIRX << DMA_CH0_CTRL_TRIG_TREQ_SEL_LSB |
      dma_chan << DMA_CH0_CTRL_TRIG_CHAIN_TO_LSB |
      DMA_CH0_CTRL_TRIG_INCR_WRITE_BITS |
      DMA_CH0_CTRL_TRIG_DATA_SIZE_VALUE_SIZE_WORD << DMA_CH0_CTRL_TRIG_DATA_SIZE_LSB |
      DMA_CH0_CTRL_TRIG_EN_BITS;

  // Now DMA is waiting, kick off the SSI transfer (mode continuation bits in LSBs)
  ssi_hw->dr0 = (flash_offs << 8u) | 0xa0u;

  // Wait for DMA finish
  while (dma_hw->ch[dma_chan].ctrl_trig & DMA_CH0_CTRL_TRIG_BUSY_BITS);

  // Reconfigure SSI before we jump back into flash!
  ssi_hw->ssienr = 0;
  ssi_hw->ctrlr1 = 0; // Single 32-bit data frame per transfer
  ssi_hw->dmacr = 0;
  ssi_hw->ssienr = 1;
}

static uint read_dma_ch;

static int flashReadCallback(const struct lfs_config *c, lfs_block_t block,
                           lfs_off_t off, void *buffer, lfs_size_t size) {
  flash_bulk_read((uint32_t *) buffer, kFlashStartOffset + block * FLASH_SECTOR_SIZE + off, size >> 2, read_dma_ch);
  return 0;
}

static int __no_inline_not_in_flash_func(flashProgCallback)(const struct lfs_config *c, lfs_block_t block,
                                                          lfs_off_t off, const void *buffer, lfs_size_t size) {
  CRITICAL_BLOCK;
  flash_range_program(kFlashStartOffset + block * FLASH_SECTOR_SIZE + off, (const uint8_t *) buffer, size);
  return 0;
}

static int __no_inline_not_in_flash_func(flashEraseCallback)(const struct lfs_config *c, lfs_block_t block) {
  CRITICAL_BLOCK;
  flash_range_erase(kFlashStartOffset + block * FLASH_SECTOR_SIZE, 4096);
  return 0;
}

static int __no_inline_not_in_flash_func(flashSyncCallback)(const struct lfs_config *c) {
  return 0;
}

static int lockCallback(const struct lfs_config *c) {
  return xSemaphoreTake(lfsMutex, portMAX_DELAY) == pdTRUE ? 0 : -1;
}

static int unlockCallback(const struct lfs_config *c) {
  xSemaphoreGive(lfsMutex);
  return 0;
}

// configuration of the filesystem is provided by this struct
static struct lfs_config cfg = {
    // block device operations
    .read  = flashReadCallback,
    .prog  = flashProgCallback,
    .erase = flashEraseCallback,
    .sync  = flashSyncCallback,

    .lock = lockCallback,
    .unlock = unlockCallback,

    // block device configuration
    .read_size = FLASH_PAGE_SIZE,
    .prog_size = FLASH_PAGE_SIZE,
    .block_size = FLASH_SECTOR_SIZE,
    .block_count = 1024,
    .block_cycles = 500,
    .cache_size = 256,
    .lookahead_size = 128,
};

void lfsInit() {
  lfsMutex = xSemaphoreCreateMutex();
  read_dma_ch = dma_claim_unused_channel(true);
  int err = lfs_mount(&lfs_instance, &cfg);
  if (err) {
    // TODO: Only format if user consents
    lfs_format(&lfs_instance, &cfg);
    lfs_mount(&lfs_instance, &cfg);
  }
}

static int mg_lfs_stat(const char *path, size_t *size, time_t *mtime) {
  lfs_info info;
  int err = lfs_stat(&lfs_instance, path, &info);
  if (err)
    return 0;
  *size = info.size;
  return MG_FS_READ | MG_FS_WRITE | (info.type == LFS_TYPE_DIR ? MG_FS_DIR : 0);
}

static void mg_lfs_list(const char *dir, void (*fn)(const char *, void *),
                        void *userdata) {
  lfs_dir_t ent;
  int err = lfs_dir_open(&lfs_instance, &ent, dir);
  if (err < 0) {
    return;
  }
  lfs_info dir_info;
  while (lfs_dir_read(&lfs_instance, &ent, &dir_info) > 0) {
    fn(dir_info.name, userdata);
  }
  lfs_dir_close(&lfs_instance, &ent);
}

static void *mg_lfs_open(const char *path, int flags) {
  int f = flags == MG_FS_READ ? lfs_open_flags::LFS_O_RDONLY : lfs_open_flags::LFS_O_APPEND;
  lfs_file_t *ptr = (lfs_file_t *) pvPortMalloc(sizeof(lfs_file_t));
  if (!ptr)
    return nullptr;
  int err = lfs_file_open(&lfs_instance, ptr, path, f);
  if (err < 0) {
    vPortFree(ptr);
    ptr = nullptr;
  }
  return ptr;
}

static void mg_lfs_close(void *fp) {
  lfs_file_close(&lfs_instance, (lfs_file_t *) fp);
  vPortFree(fp);
}

static size_t mg_lfs_read(void *fp, void *buf, size_t len) {
  return lfs_file_read(&lfs_instance, (lfs_file_t *) fp, buf, len);
}

static size_t mg_lfs_write(void *fp, const void *buf, size_t len) {
  return lfs_file_write(&lfs_instance, (lfs_file_t *) fp, buf, len);
}

static size_t mg_lfs_seek(void *fp, size_t offset) {
  return lfs_file_seek(&lfs_instance, (lfs_file_t *) fp, offset, LFS_SEEK_SET);
}

static bool mg_lfs_rename(const char *from, const char *to) {
  return lfs_rename(&lfs_instance, from, to) == 0;
}

static bool mg_lfs_remove(const char *path) {
  return lfs_remove(&lfs_instance, path) == 0;
}

static bool mg_lfs_mkdir(const char *path) {
  return lfs_mkdir(&lfs_instance, path) == 0;
}

struct mg_fs mg_fs_littlefs = {mg_lfs_stat, mg_lfs_list, mg_lfs_open, mg_lfs_close, mg_lfs_read,
                            mg_lfs_write, mg_lfs_seek, mg_lfs_rename, mg_lfs_remove, mg_lfs_mkdir};
