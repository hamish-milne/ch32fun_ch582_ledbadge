#include "ch32fun.h"
#include <stdbool.h>
#include <stdio.h>

#if __GNUC__
#define INLINE     inline __attribute__((always_inline))
#define SHORT_ENUM __attribute__((packed))
#else
#define INLINE     inline
#define SHORT_ENUM
#endif

#ifndef FLASH_DECORATOR
#define FLASH_DECORATOR __HIGH_CODE
#endif

typedef enum {
    FLASH_READ = 0,
    FLASH_WRITE = (1 << 0),
    FLASH_CODE_AREA = (1 << 1),
} _flash_access_t;

FLASH_DECORATOR INLINE
static uint32_t _flash_enable(_flash_access_t access) {
    uint8_t flags = RB_ROM_CTRL_EN | ((access & FLASH_WRITE) ? (
        (access & FLASH_CODE_AREA) ? RB_ROM_CODE_WE : RB_ROM_DATA_WE
    ) : 0);
    SYS_SAFE_ACCESS(
        R8_RESET_STATUS |= flags;
    );
}

FLASH_DECORATOR INLINE
static void _flash_disable() {
    SYS_SAFE_ACCESS(
        R8_RESET_STATUS &= RB_ROM_CODE_OFS;
    );
}

typedef enum {
    FLASH_CMD_START = 0xff,
    FLASH_CMD_READ = 0xb,
    FLASH_CMD_READ_UID = 0x4b,
    FLASH_CMD_WRITE = 0x2,
    FLASH_CMD_WRITE_ENABLE = 0x6,
    FLASH_CMD_COMMIT = 0x5,
    FLASH_CMD_ERASE_0x100 = 0x81,
    FLASH_CMD_ERASE_0x1000 = 0x20,
    FLASH_CMD_ERASE_0x10000 = 0xd8,
    FLASH_CMD_SET_LOCK = 0x1,
    FLASH_CMD_POWER_UP = 0xab,
    FLASH_CMD_POWER_DOWN = 0xb9,
    FLASH_CMD_SWR0 = 0x66,
    FLASH_CMD_SWR1 = 0x99,
} SHORT_ENUM _flash_cmd_t;

typedef enum {
    FLASH_CTRL_NONE = 0,
    FLASH_CTRL_OUT = (1 << 0),
    FLASH_CTRL_IN = (1 << 2),
    FLASH_CTRL_WORD_OUT = (1 << 4),
    FLASH_CTRL_BUSY = (1 << 7),
} SHORT_ENUM _flash_ctrl_t;

FLASH_DECORATOR INLINE
static void _flash_cmd(_flash_cmd_t cmd) {
    R8_FLASH_CTRL = FLASH_CTRL_NONE;
    R8_FLASH_CTRL = FLASH_CTRL_IN | FLASH_CTRL_OUT;
    R8_FLASH_DATA = cmd;
}

FLASH_DECORATOR INLINE
static void _flash_wait() {
    while (R8_FLASH_CTRL & FLASH_CTRL_BUSY);
}

FLASH_DECORATOR INLINE
static void _flash_cmd_end() {
    _flash_wait();
    R8_FLASH_CTRL = FLASH_CTRL_NONE;
}

FLASH_DECORATOR INLINE
static uint8_t _flash_read_byte() {
    _flash_wait();
    return R8_FLASH_DATA;
}

FLASH_DECORATOR INLINE
static void _flash_write_byte(uint8_t byte) {
    _flash_wait();
    R8_FLASH_DATA = byte;
}

FLASH_DECORATOR
static void _flash_addr(_flash_access_t access, _flash_cmd_t cmd, uint32_t addr) {
    if (access & FLASH_WRITE) {
        _flash_cmd(FLASH_CMD_WRITE_ENABLE);
        _flash_cmd_end();
    }
    _flash_cmd(cmd);
    for (int i = 16; i >= 0; i -= 8) {
        _flash_write_byte((uint8_t)(addr >> i));
    }
    if (!(access & FLASH_WRITE)) {
        _flash_write_byte(0);
        _flash_write_byte(0);
    }
}

FLASH_DECORATOR
static void _flash_read_bytes(uint32_t addr, uint8_t *buf, uint32_t len) {
    _flash_addr(FLASH_READ, FLASH_CMD_READ, addr);
    for (uint32_t i = 0; i < len; i++) {
        buf[i] = _flash_read_byte();
    }
}

FLASH_DECORATOR
static void _flash_read_words(uint32_t addr, uint32_t *buf, uint32_t len) {
    _flash_addr(FLASH_READ, FLASH_CMD_READ, addr);
    for (uint32_t i = 0; i < len; i++) {
        for (int j = 0; j < 4; j++) {
            _flash_read_byte();
        }
        buf[i] = R32_FLASH_DATA;
    }
}

typedef enum {
    FLASH_OK = 0,
    FLASH_ERROR_COMMIT = -1,
    FLASH_ERROR_BOUNDS = -2,
    FLASH_ERROR_ALIGNMENT = -3,
} flash_result_t;

#define FLASH_IS_ACTIVE  RB_ROM_CTRL_EN

FLASH_DECORATOR
static void _flash_start(_flash_access_t access) {
    __disable_irq();
    NVIC->IRER[0] = (~0);
    NVIC->IRER[1] = (~0);
    _flash_enable(access);
    R8_FLASH_CTRL = FLASH_CTRL_IN;
    _flash_cmd(FLASH_CMD_START);
}

FLASH_DECORATOR
static void _flash_finish() {
    _flash_cmd_end();
    _flash_disable();
    __enable_irq();
}

FLASH_DECORATOR
static bool _flash_commit() {
    _flash_cmd_end();
    for (int i = 0x80000; i > 0; i--) {
        _flash_cmd(FLASH_CMD_COMMIT);
        _flash_read_byte();
        uint8_t status = _flash_read_byte();
        _flash_cmd_end();
        if ((status & 1) == 0) {
            return true;
        }
    }
    return false;
}

#define FLASH_WRITE_PAGE_SIZE  0x100

FLASH_DECORATOR
static flash_result_t _flash_write_bytes(uint32_t addr, const uint8_t *buf, uint32_t len) {
    const uint8_t *p = buf;
    const uint8_t *end = buf + len;
    while (p < end) {
        _flash_addr(FLASH_WRITE, FLASH_CMD_WRITE, addr);
        for (int i = 0; i < FLASH_WRITE_PAGE_SIZE && p < end; i++) {
            _flash_write_byte(*(p++));
        }
        if (!_flash_commit()) {
            return FLASH_ERROR_COMMIT;
        }
        addr += FLASH_WRITE_PAGE_SIZE;
    }
    return FLASH_OK;
}

FLASH_DECORATOR
static flash_result_t _flash_write_words(uint32_t addr, const uint32_t *buf, uint32_t len) {
    const uint32_t *p = buf;
    const uint32_t *end = buf + len;
    while (p < end) {
        _flash_addr(FLASH_WRITE, FLASH_CMD_WRITE, addr);
        for (int i = 0; i < FLASH_WRITE_PAGE_SIZE / sizeof(uint32_t) && p < end; i++) {
            R32_FLASH_DATA = *p++;
            for (int j = 0; j < 4; j++) {
                _flash_wait();
                R8_FLASH_CTRL = FLASH_CTRL_IN | FLASH_CTRL_OUT | FLASH_CTRL_WORD_OUT;
            }
        }
        if (!_flash_commit()) {
            return FLASH_ERROR_COMMIT;
        }
        addr += FLASH_WRITE_PAGE_SIZE;
    }
    return FLASH_OK;
}

FLASH_DECORATOR
static flash_result_t _flash_erase(_flash_access_t access, uint32_t addr, uint32_t length) {
    uint32_t block_max = 0x1000;
    uint32_t block_min = 0x100;
    if (access & FLASH_CODE_AREA) {
        block_max = 0x10000;
        block_min = 0x1000;
    }
    if ((addr & (block_min - 1)) != 0) {
        return FLASH_ERROR_ALIGNMENT;
    }
    uint32_t erase_size = block_max;
    do {
        while (length >= erase_size) {
            _flash_cmd_t cmd;
            if (erase_size == 0x10000) {
                cmd = FLASH_CMD_ERASE_0x10000;
            } else if (erase_size == 0x1000) {
                cmd = FLASH_CMD_ERASE_0x1000;
            } else {
                cmd = FLASH_CMD_ERASE_0x100;
            }
            _flash_addr(access, cmd, addr);
            if (!_flash_commit()) {
                return FLASH_ERROR_COMMIT;
            }
            addr += erase_size;
            length -= erase_size;
        }
        erase_size >>= 4;
    } while (erase_size >= block_min);
    return FLASH_OK;
}

#define CODEFLASH_OFFSET  0x00000
#define CODEFLASH_LENGTH  0x70000

#define DATAFLASH_OFFSET  0x70000
#define DATAFLASH_LENGTH  0x08000

#define BOOTLOADER_OFFSET 0x78000
#define BOOTLOADER_LENGTH 0x06000

#define INFOFLASH_OFFSET  0x7E000
#define INFOFLASH_LENGTH  0x02000

FLASH_DECORATOR INLINE
static flash_result_t _flash_data_bounds(uint32_t *addr, uint32_t len) {
    if (len == 0 || *addr > DATAFLASH_LENGTH || (*addr + len) > DATAFLASH_LENGTH) {
        return FLASH_ERROR_BOUNDS;
    }
    *addr = (*addr + DATAFLASH_OFFSET) | 0x80000;
    return FLASH_OK;
}

FLASH_DECORATOR INLINE
static flash_result_t _flash_code_bounds(uint32_t *addr, uint32_t len) {
    if (len == 0 || *addr > CODEFLASH_LENGTH || (*addr + len) > CODEFLASH_LENGTH) {
        return FLASH_ERROR_BOUNDS;
    }
#if CH583
    *addr = (*addr + CODEFLASH_OFFSET) | 0x80000;
#else
    *addr = (*addr + CODEFLASH_OFFSET);
#endif
    return FLASH_OK;
}

FLASH_DECORATOR
static flash_result_t flash_data_read(uint32_t addr, uint8_t *buf, uint32_t len) {
    flash_result_t res = _flash_data_bounds(&addr, len);
    if (res != FLASH_OK) {
        return res;
    }
    _flash_start(FLASH_READ);
    _flash_read_bytes(addr, buf, len);
    _flash_finish();
    return FLASH_OK;
}

FLASH_DECORATOR
static flash_result_t flash_data_write(uint32_t addr, const uint8_t *buf, uint32_t len) {
    flash_result_t res = _flash_data_bounds(&addr, len);
    if (res != FLASH_OK) {
        return res;
    }
    _flash_start(FLASH_WRITE);
    res = _flash_write_bytes(addr, buf, len);
    _flash_finish();
    return res;
}

FLASH_DECORATOR
static flash_result_t flash_data_erase(uint32_t addr, uint32_t length) {
    flash_result_t res = _flash_data_bounds(&addr, length);
    if (res != FLASH_OK) {
        return res;
    }
    _flash_start(FLASH_WRITE);
    res = _flash_erase(FLASH_WRITE, addr, length);
    _flash_finish();
    return res;
}

FLASH_DECORATOR
static uint64_t flash_uid_read() {
    _flash_start(FLASH_READ);
    _flash_addr(FLASH_READ, FLASH_CMD_READ_UID, 0);
    uint64_t uid = 0;
    uint8_t *bytes = (uint8_t*)&uid;
    for (int i = 0xf; i >= 0; i--) {
        bytes[i & 7] ^= _flash_read_byte();
    }
    _flash_finish();
    return uid;
}

typedef union { 
    uint8_t bytes[6];
    uint32_t words[2];
} mac_addr_t;

#define FLASH_MAC_ADDR_OFFSET 0x7F018

FLASH_DECORATOR
static mac_addr_t flash_mac_read() {
    mac_addr_t mac;
    _flash_start(FLASH_READ);
    _flash_read_words(FLASH_MAC_ADDR_OFFSET | 0x80000, mac.words, sizeof(mac.words) / sizeof(uint32_t));
    _flash_cmd_end();
    _flash_finish();
    return mac;
}

FLASH_DECORATOR
static flash_result_t flash_code_write(uint32_t addr, const uint32_t *buf, uint32_t len) {
    flash_result_t res = _flash_code_bounds(&addr, len * sizeof(uint32_t));
    if (res != FLASH_OK) {
        return res;
    }
    _flash_start(FLASH_WRITE | FLASH_CODE_AREA);
    res = _flash_write_words(addr, buf, len);
    _flash_finish();
    return res;
}

FLASH_DECORATOR
static flash_result_t flash_code_erase(uint32_t addr, uint32_t length) {
    flash_result_t res = _flash_code_bounds(&addr, length * sizeof(uint32_t));
    if (res != FLASH_OK) {
        return res;
    }
    _flash_start(FLASH_WRITE | FLASH_CODE_AREA);
    res = _flash_erase(FLASH_WRITE | FLASH_CODE_AREA, addr, length);
    _flash_finish();
    return res;
}

FLASH_DECORATOR
static flash_result_t flash_code_read(uint32_t addr, uint32_t *buf, uint32_t len) {
    flash_result_t res = _flash_code_bounds(&addr, len * sizeof(uint32_t));
    if (res != FLASH_OK) {
        return res;
    }
    _flash_start(FLASH_READ | FLASH_CODE_AREA);
    _flash_read_words(addr, buf, len);
    _flash_finish();
    return FLASH_OK;
}

FLASH_DECORATOR
static void flash_power_set(bool power_up) {
    _flash_start(FLASH_READ);
    _flash_cmd(power_up ? FLASH_CMD_POWER_UP : FLASH_CMD_POWER_DOWN);
    _flash_finish();
}

FLASH_DECORATOR
static void flash_reset() {
    _flash_start(FLASH_READ);
    _flash_cmd(FLASH_CMD_SWR0);
    _flash_cmd_end();
    _flash_cmd(FLASH_CMD_SWR1);
    _flash_finish();
}

typedef enum {
    FLASH_LOCKCMD_UNLOCK = 0,
    FLASH_LOCKCMD_BOOT = 0x44,
    FLASH_LOCKCMD_DATA = 0x50,
    FLASH_LOCKCMD_ALL = 0x3c,
} SHORT_ENUM _flash_lockcmd_t;

typedef enum {
    FLASH_LOCK_NONE = 0,
    FLASH_LOCK_BOOT = (1 << 0),
    FLASH_LOCK_DATA = (1 << 1),
    FLASH_LOCK_ALL = FLASH_LOCK_BOOT | FLASH_LOCK_DATA,
} flash_lock_t;

FLASH_DECORATOR
static flash_result_t flash_lock_set(flash_lock_t lock) {
    _flash_start(FLASH_WRITE);
    _flash_cmd(FLASH_CMD_WRITE_ENABLE);
    _flash_cmd_end();
    _flash_cmd(FLASH_CMD_SET_LOCK);
    _flash_lockcmd_t lockcmd;
    if (lock & FLASH_LOCK_ALL) {
        lockcmd = FLASH_LOCKCMD_ALL;
    } else if (lock & FLASH_LOCK_DATA) {
        lockcmd = FLASH_LOCKCMD_DATA;
    } else if (lock & FLASH_LOCK_BOOT) {
        lockcmd = FLASH_LOCKCMD_BOOT;
    } else {
        lockcmd = FLASH_LOCKCMD_UNLOCK;
    }
    _flash_write_byte((uint8_t)lockcmd);
    flash_result_t res = _flash_commit();
    _flash_finish();
    return res;
}
