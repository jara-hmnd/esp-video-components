/*
 * SPDX-FileCopyrightText: 2026
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * ST VD66GY bring-up: register semantics follow STMicroelectronics vd56g3 Linux
 * driver behavior; firmware blobs live in component st_vd66gy_fw (GPL-2.0).
 */

#include <inttypes.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_rom_sys.h"

#include "esp_cam_sensor.h"
#include "esp_cam_sensor_detect.h"
#include "esp_sccb_intf.h"
#include "vd66gy.h"

#define VD66GY_IO_MUX_LOCK(mux)
#define VD66GY_IO_MUX_UNLOCK(mux)
#define VD66GY_ENABLE_OUT_CLOCK(pin, clk)
#define VD66GY_DISABLE_OUT_CLOCK(pin)

#ifndef portTICK_RATE_MS
#define portTICK_RATE_MS portTICK_PERIOD_MS
#endif

/* Use RTOS delay for multi-ms sleeps; sub-tick waits must use esp_rom_delay_us (see poll loop). */
static inline void vd66gy_delay_ms(unsigned ms)
{
    if (ms == 0) {
        return;
    }
    TickType_t ticks = pdMS_TO_TICKS(ms);
    vTaskDelay(ticks > 0 ? ticks : 1);
}

static const char *TAG = "vd66gy";

/* GPL firmware symbols (st_vd66gy_fw) */
extern const uint8_t patch_cut2[];
extern const size_t vd66gy_patch_cut2_size;
extern const uint8_t vtpatch[];
/* Must match struct vtram_area in vd56g3_vtpatch.c (GPL blob). */
struct vtram_area {
    int offset;
    int size;
};
extern struct vtram_area vtpatch_desc[];
extern int vtpatch_area_nb;

#define HZ_PER_MHZ 1000000UL
#define VD56G3_TARGET_PLL 804000000UL
#define VD56G3_VT_CLOCK_DIV 5

#define VD56G3_MODEL_ID 0x5603
#define VD56G3_REVISION_CUT2 0x20
#define VD56G3_REVISION_CUT3 0x31
#define VD56G3_OPTICAL_REVISION_BAYER 1

#define VD56G3_SYSTEM_FSM_READY_TO_BOOT 0x01
#define VD56G3_SYSTEM_FSM_SW_STBY       0x02
#define VD56G3_SYSTEM_FSM_STREAMING     0x03

static const char *vd66gy_fsm_name(uint8_t f)
{
    switch (f) {
    case VD56G3_SYSTEM_FSM_READY_TO_BOOT:
        return "READY_TO_BOOT";
    case VD56G3_SYSTEM_FSM_SW_STBY:
        return "SW_STBY";
    case VD56G3_SYSTEM_FSM_STREAMING:
        return "STREAMING";
    default:
        return "UNKNOWN";
    }
}

#define VD56G3_CMD_ACK           0
#define VD56G3_CMD_BOOT          1
#define VD56G3_CMD_PATCH_SETUP   2
#define VD56G3_CMD_START_STREAM  1
#define VD56G3_CMD_STOP_STREAM   1
#define VD56G3_CMD_START_VTRAM_UPDATE 1
#define VD56G3_CMD_END_VTRAM_UPDATE   2
#define VD56G3_CMD_THSENS_READ   4

#define VD56G3_EXP_MODE_AUTO   0

#define MIPI_CSI2_DT_RAW8 0x2a

#define VD56G3_NATIVE_WIDTH         1124
#define VD56G3_NATIVE_HEIGHT        1364
#define VD56G3_LINE_LENGTH_MIN        1236
#define VD56G3_FRAME_LENGTH_DEF_60FPS 2168
#define VD56G3_EXPOSURE_MARGIN        75
#define VD56G3_EXPOSURE_MIN           21
#define VD56G3_EXPOSURE_DEFAULT       1420U

#define VD56G3_LINK_FREQ_DEF_2LANES 402000000UL

#define VD56G3_DUSTER_ENABLE_DEF_MODULES 0x13
#define VD56G3_DARKCAL_ENABLE            1

#define VD56G3_GPIOX_GPIO_IN 0x01

#define CCI_REG_ADDR_MASK 0xffffu
#define CCI_REG_WIDTH_SHIFT 16

#define CCI_REG8(a)  ((1u << CCI_REG_WIDTH_SHIFT) | (a))
#define CCI_REG16(a) ((2u << CCI_REG_WIDTH_SHIFT) | (a))
#define CCI_REG32(a) ((4u << CCI_REG_WIDTH_SHIFT) | (a))

#define REG_MODEL_ID           CCI_REG16(0x0000)
#define REG_REVISION           CCI_REG16(0x0002)
#define REG_OPTICAL_REVISION   CCI_REG8(0x001a)
#define REG_FWPATCH_REVISION   CCI_REG16(0x001e)
#define REG_VTPATCH_ID         CCI_REG8(0x0020)
#define REG_SYSTEM_FSM         CCI_REG8(0x0028)
#define REG_BOOT               CCI_REG8(0x0200)
#define REG_STBY               CCI_REG8(0x0201)
#define REG_STREAMING          CCI_REG8(0x0202)
#define REG_VTPATCHING         CCI_REG8(0x0203)
#define REG_EXT_CLOCK          CCI_REG32(0x0220)
#define REG_CLK_PLL_PREDIV     CCI_REG8(0x0224)
#define REG_CLK_SYS_PLL_MULT   CCI_REG8(0x0226)
#define REG_ORIENTATION        CCI_REG8(0x0302)
#define REG_FORMAT_CTRL        CCI_REG8(0x030a)
#define REG_OIF_CTRL           CCI_REG16(0x030c)
#define REG_OIF_CSI_BITRATE    CCI_REG16(0x0312)
#define REG_OIF_IMG_CTRL       CCI_REG8(0x030f)
#define REG_ISL_ENABLE         CCI_REG8(0x0333)
#define REG_DUSTER_CTRL        CCI_REG8(0x0318)
#define REG_DARKCAL_CTRL       CCI_REG8(0x0340)
#define REG_PATGEN_CTRL        CCI_REG16(0x0400)
#define REG_EXP_MODE           CCI_REG8(0x044c)
#define REG_AE_COLDSTART_COARSE_EXPOSURE CCI_REG16(0x042a)
#define REG_AE_COLDSTART_ANALOG_GAIN     CCI_REG8(0x042c)
#define REG_AE_COLDSTART_DIGITAL_GAIN    CCI_REG16(0x042e)
#define REG_FRAME_LENGTH       CCI_REG16(0x0458)
#define REG_Y_START            CCI_REG16(0x045a)
#define REG_Y_END              CCI_REG16(0x045c)
#define REG_OUT_ROI_X_START    CCI_REG16(0x045e)
#define REG_OUT_ROI_X_END      CCI_REG16(0x0460)
#define REG_OUT_ROI_Y_START    CCI_REG16(0x0462)
#define REG_OUT_ROI_Y_END      CCI_REG16(0x0464)
#define REG_AE_ROI_START_H     CCI_REG16(0x0432)
#define REG_AE_ROI_END_H       CCI_REG16(0x0436)
#define REG_AE_ROI_START_V     CCI_REG16(0x0434)
#define REG_AE_ROI_END_V       CCI_REG16(0x0438)
#define REG_READOUT_CTRL       CCI_REG8(0x047e)
#define READOUT_NORMAL         0x00

#define VT_REVISION_EXPECT 17

typedef struct {
    uint32_t pll_prediv;
    uint32_t pll_mult;
    uint32_t pixel_clock;
    uint16_t oif_ctrl;
    bool is_fastboot;
    bool patch_loaded;
    uint32_t xclk_hz;
} vd66gy_priv_t;

/* STM CCI 16-bit registers are big-endian on the bus; esp_sccb a16v16 uses LE uint16. */
static inline uint16_t vd66gy_u16_bus_to_logical(uint16_t raw)
{
    return (uint16_t)((raw << 8) | (raw >> 8));
}

static esp_err_t vd66gy_cci_read(esp_sccb_io_handle_t io, uint32_t cci, uint32_t *val)
{
    uint16_t addr = cci & CCI_REG_ADDR_MASK;
    unsigned w = (cci >> CCI_REG_WIDTH_SHIFT) & 7u;
    esp_err_t err;

    switch (w) {
    case 1: {
        uint8_t v = 0;
        err = esp_sccb_transmit_receive_reg_a16v8(io, addr, &v);
        if (err == ESP_OK) {
            *val = v;
        }
        return err;
    }
    case 2: {
        uint16_t v = 0;
        err = esp_sccb_transmit_receive_reg_a16v16(io, addr, &v);
        if (err == ESP_OK) {
            *val = (uint32_t)vd66gy_u16_bus_to_logical(v);
        }
        return err;
    }
    default:
        return ESP_ERR_NOT_SUPPORTED;
    }
}

static esp_err_t vd66gy_cci_write(esp_sccb_io_handle_t io, uint32_t cci, uint32_t val)
{
    uint16_t addr = cci & CCI_REG_ADDR_MASK;
    unsigned w = (cci >> CCI_REG_WIDTH_SHIFT) & 7u;

    switch (w) {
    case 1:
        return esp_sccb_transmit_reg_a16v8(io, addr, (uint8_t)val);
    case 2:
        return esp_sccb_transmit_reg_a16v16(io, addr, vd66gy_u16_bus_to_logical((uint16_t)val));
    case 4: {
        uint8_t b[4];
        b[0] = (uint8_t)(val & 0xff);
        b[1] = (uint8_t)((val >> 8) & 0xff);
        b[2] = (uint8_t)((val >> 16) & 0xff);
        b[3] = (uint8_t)((val >> 24) & 0xff);
        esp_err_t e = ESP_OK;
        for (int i = 0; i < 4; i++) {
            e = esp_sccb_transmit_reg_a16v8(io, addr + (uint16_t)i, b[i]);
            if (e != ESP_OK) {
                return e;
            }
        }
        return ESP_OK;
    }
    default:
        return ESP_ERR_NOT_SUPPORTED;
    }
}

static esp_err_t vd66gy_write_array(esp_sccb_io_handle_t io, uint16_t reg, size_t len, const uint8_t *data)
{
    const size_t chunk = 256;
    for (size_t off = 0; off < len; off += chunk) {
        size_t n = (len - off) > chunk ? chunk : (len - off);
        for (size_t i = 0; i < n; i++) {
            esp_err_t e = esp_sccb_transmit_reg_a16v8(io, reg + (uint16_t)(off + i), data[off + i]);
            if (e != ESP_OK) {
                return e;
            }
        }
    }
    return ESP_OK;
}

static esp_err_t vd66gy_poll_reg8(esp_sccb_io_handle_t io, uint32_t cci, uint8_t expect, int timeout_ms)
{
    uint32_t last = 0;
    for (int t = 0; t < timeout_ms; t++) {
        uint32_t v = 0;
        esp_err_t e = vd66gy_cci_read(io, cci, &v);
        if (e != ESP_OK) {
            ESP_LOGE(TAG, "VID66GY_STREAM_FAIL: SCCB read reg 0x%04" PRIx32 " during poll: %s",
                     cci & CCI_REG_ADDR_MASK, esp_err_to_name(e));
            return e;
        }
        last = v;
        if ((uint8_t)v == expect) {
            return ESP_OK;
        }
        esp_rom_delay_us(1000);
    }
    ESP_LOGE(TAG, "VID66GY_STREAM_FAIL: poll timeout reg 0x%04" PRIx32 " expect 0x%02x last 0x%02x",
             cci & CCI_REG_ADDR_MASK, expect, (unsigned)(last & 0xff));
    return ESP_ERR_TIMEOUT;
}

static esp_err_t vd66gy_wait_fsm(esp_sccb_io_handle_t io, uint8_t state, int timeout_ms)
{
    esp_err_t err = vd66gy_poll_reg8(io, REG_SYSTEM_FSM, state, timeout_ms);
    if (err != ESP_OK) {
        uint32_t fsm = 0;
        if (vd66gy_cci_read(io, REG_SYSTEM_FSM, &fsm) == ESP_OK) {
            uint8_t fb = (uint8_t)(fsm & 0xff);
            ESP_LOGE(TAG, "VID66GY_STREAM_FAIL: FSM wait need %s (0x%02x) still %s (0x%02x)",
                     vd66gy_fsm_name(state), state, vd66gy_fsm_name(fb), fb);
        }
    }
    return err;
}

/* After power/reset, ROM reports READY_TO_BOOT; a warm module may already be in SW_STBY or STREAMING. */
static esp_err_t vd66gy_wait_rom_ready(esp_sccb_io_handle_t io, int timeout_ms)
{
    for (int t = 0; t < timeout_ms; t++) {
        uint32_t v = 0;
        esp_err_t e = vd66gy_cci_read(io, REG_SYSTEM_FSM, &v);
        if (e != ESP_OK) {
            ESP_LOGE(TAG, "VID66GY_STREAM_FAIL: ROM wait read SYSTEM_FSM: %s", esp_err_to_name(e));
            return e;
        }
        uint8_t f = (uint8_t)v;
        if (f == VD56G3_SYSTEM_FSM_READY_TO_BOOT || f == VD56G3_SYSTEM_FSM_SW_STBY ||
            f == VD56G3_SYSTEM_FSM_STREAMING) {
            return ESP_OK;
        }
        esp_rom_delay_us(1000);
    }
    uint32_t fsm = 0, model = 0;
    (void)vd66gy_cci_read(io, REG_SYSTEM_FSM, &fsm);
    (void)vd66gy_cci_read(io, REG_MODEL_ID, &model);
    ESP_LOGE(TAG, "ROM ready timeout: SYSTEM_FSM=0x%02" PRIx32 " MODEL_ID=0x%04" PRIx32,
             fsm & 0xffu, model & 0xffffu);
    return ESP_ERR_TIMEOUT;
}

static esp_err_t vd66gy_prepare_clock_tree(vd66gy_priv_t *p, uint32_t xclk_hz)
{
    const unsigned predivs[] = {1, 2, 4};

    if (xclk_hz < 6 * HZ_PER_MHZ || xclk_hz > 27 * HZ_PER_MHZ) {
        ESP_LOGE(TAG, "xclk %" PRIu32 " Hz out of 6–27 MHz range", xclk_hz);
        return ESP_ERR_INVALID_ARG;
    }

    p->xclk_hz = xclk_hz;
    size_t i;
    for (i = 0; i < sizeof(predivs) / sizeof(predivs[0]); i++) {
        p->pll_prediv = predivs[i];
        if (xclk_hz / p->pll_prediv < 12 * HZ_PER_MHZ) {
            break;
        }
    }

    p->pll_mult = (VD56G3_TARGET_PLL * p->pll_prediv + xclk_hz / 2) / xclk_hz;
    uint32_t pll_out = xclk_hz * p->pll_mult / p->pll_prediv;
    p->pixel_clock = pll_out / VD56G3_VT_CLOCK_DIV;

    /*
     * 2-lane CSI mapping: logical lanes 1,2 -> physical 0,1.
     * Trial profile: invert clock and both data lane polarities (STM32 middleware uses swap enables).
     */
    p->oif_ctrl = 2u | (1u << 3) | (0u << 4) | (1u << 6) | (1u << 7) | (1u << 9);

    return ESP_OK;
}

static esp_err_t vd66gy_hw_reset(esp_cam_sensor_device_t *dev)
{
    if (dev->reset_pin >= 0) {
        gpio_set_level(dev->reset_pin, 0);
        vd66gy_delay_ms(10);
        gpio_set_level(dev->reset_pin, 1);
        vd66gy_delay_ms(10);
    }
    return ESP_OK;
}

static esp_err_t vd66gy_power_on(esp_cam_sensor_device_t *dev)
{
    esp_err_t ret = ESP_OK;

    if (dev->xclk_pin >= 0) {
        VD66GY_ENABLE_OUT_CLOCK(dev->xclk_pin, dev->priv ? ((vd66gy_priv_t *)dev->priv)->xclk_hz : 24000000);
    }

    if (dev->pwdn_pin >= 0) {
        gpio_config_t conf = {0};
        conf.pin_bit_mask = 1ULL << dev->pwdn_pin;
        conf.mode = GPIO_MODE_OUTPUT;
        ret = gpio_config(&conf);
        ESP_RETURN_ON_FALSE(ret == ESP_OK, ret, TAG, "pwdn pin config failed");
        gpio_set_level(dev->pwdn_pin, 1);
        vd66gy_delay_ms(10);
        gpio_set_level(dev->pwdn_pin, 0);
        vd66gy_delay_ms(10);
    }

    if (dev->reset_pin >= 0) {
        gpio_config_t conf = {0};
        conf.pin_bit_mask = 1ULL << dev->reset_pin;
        conf.mode = GPIO_MODE_OUTPUT;
        ret = gpio_config(&conf);
        ESP_RETURN_ON_FALSE(ret == ESP_OK, ret, TAG, "reset pin config failed");
        gpio_set_level(dev->reset_pin, 0);
        vd66gy_delay_ms(4);
        gpio_set_level(dev->reset_pin, 1);
        vd66gy_delay_ms(10);
    }

    /*
     * Linux vd56g3: after regulators + MCLK enable, ~3.5–4 ms before polling SYSTEM_FSM.
     * When the host does not drive XCLK/reset (pins -1), the module may still need settle time.
     */
    esp_rom_delay_us(3750);

    return vd66gy_wait_rom_ready(dev->sccb_handle, 2000);
}

static esp_err_t vd66gy_power_off(esp_cam_sensor_device_t *dev)
{
    if (dev->xclk_pin >= 0) {
        VD66GY_DISABLE_OUT_CLOCK(dev->xclk_pin);
    }
    if (dev->pwdn_pin >= 0) {
        gpio_set_level(dev->pwdn_pin, 0);
        vd66gy_delay_ms(10);
        gpio_set_level(dev->pwdn_pin, 1);
        vd66gy_delay_ms(10);
    }
    if (dev->reset_pin >= 0) {
        gpio_set_level(dev->reset_pin, 1);
        vd66gy_delay_ms(10);
        gpio_set_level(dev->reset_pin, 0);
        vd66gy_delay_ms(10);
    }
    return ESP_OK;
}

static esp_err_t vd66gy_boot_cmd(esp_cam_sensor_device_t *dev)
{
    esp_err_t ret = vd66gy_cci_write(dev->sccb_handle, REG_BOOT, VD56G3_CMD_BOOT);
    ESP_RETURN_ON_ERROR(ret, TAG, "boot cmd");
    ret = vd66gy_poll_reg8(dev->sccb_handle, REG_BOOT, VD56G3_CMD_ACK, 500);
    ESP_RETURN_ON_ERROR(ret, TAG, "boot ack");
    ret = vd66gy_wait_fsm(dev->sccb_handle, VD56G3_SYSTEM_FSM_SW_STBY, 3000);
    return ret;
}

static esp_err_t vd66gy_patch_cut2(esp_cam_sensor_device_t *dev)
{
    const uint8_t *patch = patch_cut2;
    size_t patch_size = vd66gy_patch_cut2_size;
    uint8_t patch_major = patch[3];
    uint8_t patch_minor = patch[2];
    esp_err_t ret = vd66gy_write_array(dev->sccb_handle, 0x2000, patch_size, patch);
    ESP_RETURN_ON_ERROR(ret, TAG, "patch array");

    ret = vd66gy_cci_write(dev->sccb_handle, REG_BOOT, VD56G3_CMD_PATCH_SETUP);
    ESP_RETURN_ON_ERROR(ret, TAG, "patch setup");
    ret = vd66gy_poll_reg8(dev->sccb_handle, REG_BOOT, VD56G3_CMD_ACK, 500);
    ESP_RETURN_ON_ERROR(ret, TAG, "patch setup ack");

    uint32_t cur = 0;
    ret = vd66gy_cci_read(dev->sccb_handle, REG_FWPATCH_REVISION, &cur);
    ESP_RETURN_ON_ERROR(ret, TAG, "fwpatch rev");
    if ((uint16_t)cur != (uint16_t)((patch_major << 8) + patch_minor)) {
        ESP_LOGE(TAG, "patch version mismatch exp %u.%u got 0x%04" PRIx32, patch_major, patch_minor, cur);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Cut2 patch applied (%u.%u)", patch_major, patch_minor);
    return ESP_OK;
}

static esp_err_t vd66gy_vtpatch_apply(esp_cam_sensor_device_t *dev)
{
    esp_err_t ret = vd66gy_cci_write(dev->sccb_handle, REG_VTPATCHING, VD56G3_CMD_START_VTRAM_UPDATE);
    ESP_RETURN_ON_ERROR(ret, TAG, "vtpatch start");
    ret = vd66gy_poll_reg8(dev->sccb_handle, REG_VTPATCHING, VD56G3_CMD_ACK, 500);
    ESP_RETURN_ON_ERROR(ret, TAG, "vtpatch start ack");
    ret = vd66gy_wait_fsm(dev->sccb_handle, VD56G3_SYSTEM_FSM_SW_STBY, 3000);
    ESP_RETURN_ON_ERROR(ret, TAG, "vtpatch fsm");

    int vtpatch_offset = 0;
    for (int i = 0; i < vtpatch_area_nb; i++) {
        uint16_t base = (uint16_t)vtpatch_desc[i].offset;
        int sz = vtpatch_desc[i].size;
        ret = vd66gy_write_array(dev->sccb_handle, base, (size_t)sz, vtpatch + vtpatch_offset);
        ESP_RETURN_ON_ERROR(ret, TAG, "vtpatch chunk");
        vtpatch_offset += sz;
    }

    ret = vd66gy_cci_write(dev->sccb_handle, REG_VTPATCHING, VD56G3_CMD_END_VTRAM_UPDATE);
    ESP_RETURN_ON_ERROR(ret, TAG, "vtpatch end");
    ret = vd66gy_poll_reg8(dev->sccb_handle, REG_VTPATCHING, VD56G3_CMD_ACK, 500);
    ESP_RETURN_ON_ERROR(ret, TAG, "vtpatch end ack");
    ret = vd66gy_wait_fsm(dev->sccb_handle, VD56G3_SYSTEM_FSM_SW_STBY, 3000);
    ESP_RETURN_ON_ERROR(ret, TAG, "vtpatch end fsm");

    uint32_t cur = 0;
    ret = vd66gy_cci_read(dev->sccb_handle, REG_VTPATCH_ID, &cur);
    ESP_RETURN_ON_ERROR(ret, TAG, "vtpatch id");
    if ((uint8_t)cur != VT_REVISION_EXPECT) {
        ESP_LOGE(TAG, "vtpatch version expected %u got %" PRIu32, VT_REVISION_EXPECT, cur);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "VT patch %d applied", VT_REVISION_EXPECT);
    return ESP_OK;
}

static esp_err_t vd66gy_power_patch(esp_cam_sensor_device_t *dev)
{
    vd66gy_priv_t *p = dev->priv;
    ESP_RETURN_ON_FALSE(p != NULL, ESP_ERR_INVALID_STATE, TAG, "priv");

    esp_err_t ret = vd66gy_power_on(dev);
    ESP_RETURN_ON_ERROR(ret, TAG, "power on");

    if (!p->is_fastboot) {
        ret = vd66gy_patch_cut2(dev);
        ESP_RETURN_ON_ERROR(ret, TAG, "patch");
    }

    uint32_t fsm_pp = 0;
    ret = vd66gy_cci_read(dev->sccb_handle, REG_SYSTEM_FSM, &fsm_pp);
    ESP_RETURN_ON_ERROR(ret, TAG, "read fsm for power_patch");

    if ((uint8_t)fsm_pp != VD56G3_SYSTEM_FSM_SW_STBY) {
        ret = vd66gy_boot_cmd(dev);
        ESP_RETURN_ON_ERROR(ret, TAG, "boot");
    } else {
        ESP_LOGI(TAG, "FSM already SW_STBY after power (Cut3 fastboot path); skip ROM boot cmd");
    }

    if (!p->is_fastboot) {
        ret = vd66gy_vtpatch_apply(dev);
        ESP_RETURN_ON_ERROR(ret, TAG, "vtpatch");
    }

    p->patch_loaded = true;
    return ESP_OK;
}

static esp_err_t vd66gy_write_gpio_defaults(esp_cam_sensor_device_t *dev)
{
    for (int io = 0; io < 8; io++) {
        esp_err_t e = vd66gy_cci_write(dev->sccb_handle, CCI_REG8(0x0467 + (uint32_t)io), VD56G3_GPIOX_GPIO_IN);
        if (e != ESP_OK) {
            return e;
        }
    }
    return ESP_OK;
}

static esp_err_t vd66gy_apply_ae_defaults(esp_cam_sensor_device_t *dev)
{
    esp_err_t ret = vd66gy_cci_write(dev->sccb_handle, REG_EXP_MODE, VD56G3_EXP_MODE_AUTO);
    ESP_RETURN_ON_ERROR(ret, TAG, "exp mode");
    ret = vd66gy_cci_write(dev->sccb_handle, REG_AE_COLDSTART_COARSE_EXPOSURE, VD56G3_EXPOSURE_DEFAULT);
    ESP_RETURN_ON_ERROR(ret, TAG, "ae cold expo");
    ret = vd66gy_cci_write(dev->sccb_handle, REG_AE_COLDSTART_ANALOG_GAIN, 0);
    ESP_RETURN_ON_ERROR(ret, TAG, "ae cold again");
    ret = vd66gy_cci_write(dev->sccb_handle, REG_AE_COLDSTART_DIGITAL_GAIN, 0x100);
    ESP_RETURN_ON_ERROR(ret, TAG, "ae cold dgain");
    ret = vd66gy_cci_write(dev->sccb_handle, REG_FRAME_LENGTH, VD56G3_FRAME_LENGTH_DEF_60FPS);
    ESP_RETURN_ON_ERROR(ret, TAG, "frame length");
    return ESP_OK;
}

#define VD66GY_RET_IO(expr, step) \
    do { \
        ret = (expr); \
        if (ret != ESP_OK) { \
            ESP_LOGE(TAG, "VID66GY_STREAM_FAIL: stream_on %s: %s", step, esp_err_to_name(ret)); \
            return ret; \
        } \
    } while (0)

static esp_err_t vd66gy_stream_on(esp_cam_sensor_device_t *dev)
{
    vd66gy_priv_t *p = dev->priv;
    ESP_RETURN_ON_FALSE(p != NULL, ESP_ERR_INVALID_STATE, TAG, "priv");
    ESP_RETURN_ON_FALSE(p->patch_loaded, ESP_ERR_INVALID_STATE, TAG, "call set_format first");
    const esp_cam_sensor_format_t *fmt = dev->cur_format;
    ESP_RETURN_ON_FALSE(fmt != NULL, ESP_ERR_INVALID_STATE, TAG, "format");

    unsigned lanes = fmt->mipi_info.lane_num ? fmt->mipi_info.lane_num : 2;
    uint32_t link_hz = (lanes == 2) ? VD56G3_LINK_FREQ_DEF_2LANES : 750000000UL;
    unsigned csi_mbps = (unsigned)(link_hz * 2 / HZ_PER_MHZ);

    esp_err_t ret = ESP_OK;

    ESP_LOGI(TAG, "stream_on: %ux%u @ %" PRIu32 " fps, %" PRIu32 " Hz xclk, lanes=%u CSI~%u Mbps oif_ctrl=0x%04x",
             (unsigned)fmt->width, (unsigned)fmt->height, (uint32_t)fmt->fps, p->xclk_hz, lanes, csi_mbps, p->oif_ctrl);

    VD66GY_RET_IO(vd66gy_cci_write(dev->sccb_handle, REG_EXT_CLOCK, p->xclk_hz), "REG_EXT_CLOCK");
    VD66GY_RET_IO(vd66gy_cci_write(dev->sccb_handle, REG_CLK_PLL_PREDIV, p->pll_prediv), "REG_CLK_PLL_PREDIV");
    VD66GY_RET_IO(vd66gy_cci_write(dev->sccb_handle, REG_CLK_SYS_PLL_MULT, p->pll_mult), "REG_CLK_SYS_PLL_MULT");

    VD66GY_RET_IO(vd66gy_cci_write(dev->sccb_handle, REG_FORMAT_CTRL, 8), "REG_FORMAT_CTRL");
    VD66GY_RET_IO(vd66gy_cci_write(dev->sccb_handle, REG_OIF_CTRL, p->oif_ctrl), "REG_OIF_CTRL");
    VD66GY_RET_IO(vd66gy_cci_write(dev->sccb_handle, REG_OIF_CSI_BITRATE, csi_mbps), "REG_OIF_CSI_BITRATE");
    VD66GY_RET_IO(vd66gy_cci_write(dev->sccb_handle, REG_OIF_IMG_CTRL, MIPI_CSI2_DT_RAW8), "REG_OIF_IMG_CTRL");
    VD66GY_RET_IO(vd66gy_cci_write(dev->sccb_handle, REG_ISL_ENABLE, 0), "REG_ISL_ENABLE");

    VD66GY_RET_IO(vd66gy_cci_write(dev->sccb_handle, REG_READOUT_CTRL, READOUT_NORMAL), "REG_READOUT_CTRL");

    uint16_t crop_left = (VD56G3_NATIVE_WIDTH - fmt->width) / 2;
    uint16_t crop_top = (VD56G3_NATIVE_HEIGHT - fmt->height) / 2;

    VD66GY_RET_IO(vd66gy_cci_write(dev->sccb_handle, REG_Y_START, crop_top), "REG_Y_START");
    VD66GY_RET_IO(vd66gy_cci_write(dev->sccb_handle, REG_Y_END, crop_top + fmt->height - 1), "REG_Y_END");
    VD66GY_RET_IO(vd66gy_cci_write(dev->sccb_handle, REG_OUT_ROI_X_START, crop_left), "REG_OUT_ROI_X_START");
    VD66GY_RET_IO(vd66gy_cci_write(dev->sccb_handle, REG_OUT_ROI_X_END, crop_left + fmt->width - 1), "REG_OUT_ROI_X_END");
    VD66GY_RET_IO(vd66gy_cci_write(dev->sccb_handle, REG_OUT_ROI_Y_START, 0), "REG_OUT_ROI_Y_START");
    VD66GY_RET_IO(vd66gy_cci_write(dev->sccb_handle, REG_OUT_ROI_Y_END, fmt->height - 1), "REG_OUT_ROI_Y_END");
    VD66GY_RET_IO(vd66gy_cci_write(dev->sccb_handle, REG_AE_ROI_START_H, crop_left), "REG_AE_ROI_START_H");
    VD66GY_RET_IO(vd66gy_cci_write(dev->sccb_handle, REG_AE_ROI_END_H, crop_left + fmt->width - 1), "REG_AE_ROI_END_H");
    VD66GY_RET_IO(vd66gy_cci_write(dev->sccb_handle, REG_AE_ROI_START_V, 0), "REG_AE_ROI_START_V");
    VD66GY_RET_IO(vd66gy_cci_write(dev->sccb_handle, REG_AE_ROI_END_V, fmt->height - 1), "REG_AE_ROI_END_V");

    VD66GY_RET_IO(vd66gy_cci_write(dev->sccb_handle, REG_ORIENTATION, 0), "REG_ORIENTATION");
    VD66GY_RET_IO(vd66gy_cci_write(dev->sccb_handle, REG_DUSTER_CTRL, VD56G3_DUSTER_ENABLE_DEF_MODULES), "REG_DUSTER_CTRL");
    VD66GY_RET_IO(vd66gy_cci_write(dev->sccb_handle, REG_DARKCAL_CTRL, VD56G3_DARKCAL_ENABLE), "REG_DARKCAL_CTRL");
    VD66GY_RET_IO(vd66gy_cci_write(dev->sccb_handle, REG_PATGEN_CTRL, 0), "REG_PATGEN_CTRL");

    ret = vd66gy_apply_ae_defaults(dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "VID66GY_STREAM_FAIL: stream_on apply_ae_defaults: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = vd66gy_write_gpio_defaults(dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "VID66GY_STREAM_FAIL: stream_on write_gpio_defaults: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Linux reference sequence: stream start command is written to REG_STBY (0x0201). */
    VD66GY_RET_IO(vd66gy_cci_write(dev->sccb_handle, REG_STBY, VD56G3_CMD_START_STREAM), "REG_STBY START_STREAM");
    VD66GY_RET_IO(vd66gy_poll_reg8(dev->sccb_handle, REG_STBY, VD56G3_CMD_ACK, 500), "REG_STBY ack");
    VD66GY_RET_IO(vd66gy_wait_fsm(dev->sccb_handle, VD56G3_SYSTEM_FSM_STREAMING, 3000), "FSM STREAMING");

    dev->stream_status = 1;
    ESP_LOGI(TAG, "stream_on: sensor MIPI streaming (FSM=STREAMING)");
    return ESP_OK;
}
#undef VD66GY_RET_IO

/** Stop MIPI streaming and wait for SW_STBY (used from stream_off and from detect when FSM is stuck in STREAMING). */
static esp_err_t vd66gy_stop_stream_to_sw_stby(esp_cam_sensor_device_t *dev)
{
    esp_err_t ret = vd66gy_cci_write(dev->sccb_handle, REG_STREAMING, VD56G3_CMD_STOP_STREAM);
    ESP_RETURN_ON_ERROR(ret, TAG, "stop stream");
    ret = vd66gy_poll_reg8(dev->sccb_handle, REG_STREAMING, VD56G3_CMD_ACK, 500);
    ESP_RETURN_ON_ERROR(ret, TAG, "stop ack");
    ret = vd66gy_wait_fsm(dev->sccb_handle, VD56G3_SYSTEM_FSM_SW_STBY, 3000);
    ESP_RETURN_ON_ERROR(ret, TAG, "stby fsm");
    return ESP_OK;
}

static esp_err_t vd66gy_stream_off(esp_cam_sensor_device_t *dev)
{
    esp_err_t ret = vd66gy_stop_stream_to_sw_stby(dev);
    if (ret == ESP_OK) {
        dev->stream_status = 0;
    }
    return ret;
}

static const esp_cam_sensor_isp_info_t vd66gy_isp_info_default = {
    .isp_v1_info = {
        .version = SENSOR_ISP_INFO_VERSION_DEFAULT,
        .pclk = 160800000,
        .vts = VD56G3_FRAME_LENGTH_DEF_60FPS,
        .hts = VD56G3_LINE_LENGTH_MIN,
        /*
         * Phase-2 color tuning: try GBRG Bayer order.
         * Prior BGGR and RGGB trials produced purple cast / grid artifacts.
         */
        .bayer_type = ESP_CAM_SENSOR_BAYER_GBRG,
    }
};

static const esp_cam_sensor_format_t vd66gy_formats[] = {
    {
        .name = "MIPI_2lane_24Minput_RAW8_1120x1360_60fps",
        .format = ESP_CAM_SENSOR_PIXFORMAT_RAW8,
        .port = ESP_CAM_SENSOR_MIPI_CSI,
        .xclk = 24000000,
        .width = 1120,
        .height = 1360,
        .regs = NULL,
        .regs_size = 0,
        .fps = 60,
        .isp_info = &vd66gy_isp_info_default,
        .mipi_info = {
            .mipi_clk = 804000000ULL,
            .lane_num = 2,
            .line_sync_en = false,
        },
        .reserved = NULL,
    },
};

static esp_err_t vd66gy_get_sensor_id(esp_cam_sensor_device_t *dev, esp_cam_sensor_id_t *id)
{
    uint32_t model = 0;
    esp_err_t ret = vd66gy_cci_read(dev->sccb_handle, REG_MODEL_ID, &model);
    ESP_RETURN_ON_FALSE(ret == ESP_OK, ret, TAG, "read model");
    id->pid = (uint16_t)model;
    return ESP_OK;
}

static esp_err_t vd66gy_query_para_desc(esp_cam_sensor_device_t *dev, esp_cam_sensor_param_desc_t *qdesc)
{
    (void)dev;
    (void)qdesc;
    return ESP_ERR_NOT_SUPPORTED;
}

static esp_err_t vd66gy_get_para_value(esp_cam_sensor_device_t *dev, uint32_t id, void *arg, size_t size)
{
    (void)dev;
    (void)id;
    (void)arg;
    (void)size;
    return ESP_ERR_NOT_SUPPORTED;
}

static esp_err_t vd66gy_set_para_value(esp_cam_sensor_device_t *dev, uint32_t id, const void *arg, size_t size)
{
    (void)dev;
    (void)id;
    (void)arg;
    (void)size;
    return ESP_ERR_NOT_SUPPORTED;
}

static esp_err_t vd66gy_query_support_formats(esp_cam_sensor_device_t *dev, esp_cam_sensor_format_array_t *formats)
{
    (void)dev;
    ESP_CAM_SENSOR_NULL_POINTER_CHECK(TAG, formats);
    formats->count = 1;
    formats->format_array = &vd66gy_formats[0];
    return ESP_OK;
}

static esp_err_t vd66gy_query_support_capability(esp_cam_sensor_device_t *dev, esp_cam_sensor_capability_t *cap)
{
    (void)dev;
    ESP_CAM_SENSOR_NULL_POINTER_CHECK(TAG, cap);
    cap->fmt_raw = 1;
    return ESP_OK;
}

static esp_err_t vd66gy_set_format(esp_cam_sensor_device_t *dev, const esp_cam_sensor_format_t *format)
{
    ESP_CAM_SENSOR_NULL_POINTER_CHECK(TAG, dev);
    vd66gy_priv_t *p = dev->priv;
    ESP_RETURN_ON_FALSE(p != NULL, ESP_ERR_INVALID_STATE, TAG, "priv");

    if (format == NULL) {
        format = &vd66gy_formats[CONFIG_CAMERA_VD66GY_MIPI_IF_FORMAT_INDEX_DEFAULT];
    }

    if (!p->patch_loaded) {
        esp_err_t ret = vd66gy_power_patch(dev);
        ESP_RETURN_ON_ERROR(ret, TAG, "power_patch");
    }

    dev->cur_format = format;
    return ESP_OK;
}

static esp_err_t vd66gy_get_format(esp_cam_sensor_device_t *dev, esp_cam_sensor_format_t *format)
{
    ESP_CAM_SENSOR_NULL_POINTER_CHECK(TAG, dev);
    ESP_CAM_SENSOR_NULL_POINTER_CHECK(TAG, format);
    if (dev->cur_format != NULL) {
        memcpy(format, dev->cur_format, sizeof(esp_cam_sensor_format_t));
        return ESP_OK;
    }
    return ESP_FAIL;
}

static esp_err_t vd66gy_priv_ioctl(esp_cam_sensor_device_t *dev, uint32_t cmd, void *arg)
{
    ESP_CAM_SENSOR_NULL_POINTER_CHECK(TAG, dev);
    esp_err_t ret = ESP_FAIL;
    uint8_t regval;
    esp_cam_sensor_reg_val_t *sensor_reg;

    VD66GY_IO_MUX_LOCK(mux);
    switch (cmd) {
    case ESP_CAM_SENSOR_IOC_HW_RESET:
        ret = vd66gy_hw_reset(dev);
        break;
    case ESP_CAM_SENSOR_IOC_S_REG:
        sensor_reg = (esp_cam_sensor_reg_val_t *)arg;
        ret = esp_sccb_transmit_reg_a16v8(dev->sccb_handle, sensor_reg->regaddr, sensor_reg->value);
        break;
    case ESP_CAM_SENSOR_IOC_S_STREAM:
        ret = *(int *)arg ? vd66gy_stream_on(dev) : vd66gy_stream_off(dev);
        break;
    case ESP_CAM_SENSOR_IOC_G_REG:
        sensor_reg = (esp_cam_sensor_reg_val_t *)arg;
        ret = esp_sccb_transmit_receive_reg_a16v8(dev->sccb_handle, sensor_reg->regaddr, &regval);
        if (ret == ESP_OK) {
            sensor_reg->value = regval;
        }
        break;
    case ESP_CAM_SENSOR_IOC_G_CHIP_ID:
        ret = vd66gy_get_sensor_id(dev, arg);
        break;
    default:
        ret = ESP_ERR_INVALID_ARG;
        break;
    }
    VD66GY_IO_MUX_UNLOCK(mux);
    return ret;
}

static esp_err_t vd66gy_delete(esp_cam_sensor_device_t *dev)
{
    if (dev) {
        if (dev->priv) {
            free(dev->priv);
            dev->priv = NULL;
        }
        free(dev);
    }
    return ESP_OK;
}

static const esp_cam_sensor_ops_t vd66gy_ops = {
    .query_para_desc = vd66gy_query_para_desc,
    .get_para_value = vd66gy_get_para_value,
    .set_para_value = vd66gy_set_para_value,
    .query_support_formats = vd66gy_query_support_formats,
    .query_support_capability = vd66gy_query_support_capability,
    .set_format = vd66gy_set_format,
    .get_format = vd66gy_get_format,
    .priv_ioctl = vd66gy_priv_ioctl,
    .del = vd66gy_delete,
};

esp_cam_sensor_device_t *vd66gy_detect(esp_cam_sensor_config_t *config)
{
    if (config == NULL) {
        return NULL;
    }

    esp_cam_sensor_device_t *dev = calloc(1, sizeof(esp_cam_sensor_device_t));
    if (dev == NULL) {
        ESP_LOGE(TAG, "no mem");
        return NULL;
    }

    vd66gy_priv_t *priv = calloc(1, sizeof(vd66gy_priv_t));
    if (priv == NULL) {
        free(dev);
        return NULL;
    }
    dev->priv = priv;
    dev->name = (char *)VD66GY_SENSOR_NAME;
    dev->sccb_handle = config->sccb_handle;
    dev->xclk_pin = config->xclk_pin;
    dev->reset_pin = config->reset_pin;
    dev->pwdn_pin = config->pwdn_pin;
    dev->sensor_port = config->sensor_port;
    dev->ops = &vd66gy_ops;

    if (config->sensor_port == ESP_CAM_SENSOR_MIPI_CSI) {
        dev->cur_format = &vd66gy_formats[CONFIG_CAMERA_VD66GY_MIPI_IF_FORMAT_INDEX_DEFAULT];
    } else {
        ESP_LOGE(TAG, "DVP not supported");
        goto err;
    }

    uint32_t xclk_hz = (config->xclk_freq_hz > 0) ? (uint32_t)config->xclk_freq_hz : 24000000;
    if (vd66gy_prepare_clock_tree(priv, xclk_hz) != ESP_OK) {
        goto err;
    }

    if (vd66gy_power_on(dev) != ESP_OK) {
        ESP_LOGE(TAG, "power on failed");
        goto err;
    }

    uint32_t fsm_probe = 0;
    if (vd66gy_cci_read(dev->sccb_handle, REG_SYSTEM_FSM, &fsm_probe) != ESP_OK) {
        ESP_LOGE(TAG, "read FSM after power failed");
        goto err;
    }
    if ((uint8_t)fsm_probe == VD56G3_SYSTEM_FSM_STREAMING) {
        ESP_LOGW(TAG, "FSM streaming at detect; stop stream to reach SW_STBY");
        if (vd66gy_stop_stream_to_sw_stby(dev) != ESP_OK) {
            ESP_LOGE(TAG, "stop stream before boot failed");
            goto err;
        }
        if (vd66gy_cci_read(dev->sccb_handle, REG_SYSTEM_FSM, &fsm_probe) != ESP_OK) {
            ESP_LOGE(TAG, "read FSM after stop stream failed");
            goto err;
        }
    }
    if ((uint8_t)fsm_probe == VD56G3_SYSTEM_FSM_READY_TO_BOOT) {
        if (vd66gy_boot_cmd(dev) != ESP_OK) {
            ESP_LOGE(TAG, "initial boot failed");
            goto err;
        }
    } else if ((uint8_t)fsm_probe == VD56G3_SYSTEM_FSM_SW_STBY) {
        ESP_LOGI(TAG, "FSM already SW_STBY; skip ROM boot cmd (warm / resumed)");
    } else {
        ESP_LOGE(TAG, "unexpected FSM 0x%" PRIx32 " after power", fsm_probe & 0xffu);
        goto err;
    }

    esp_cam_sensor_id_t id = {0};
    if (vd66gy_get_sensor_id(dev, &id) != ESP_OK) {
        ESP_LOGE(TAG, "read id failed");
        goto err;
    }
    if (id.pid != VD56G3_MODEL_ID) {
        ESP_LOGE(TAG, "wrong model 0x%x", id.pid);
        goto err;
    }

    uint32_t rev = 0;
    if (vd66gy_cci_read(dev->sccb_handle, REG_REVISION, &rev) != ESP_OK) {
        ESP_LOGE(TAG, "read revision failed");
        goto err;
    }
    if (((rev >> 8) & 0xff) == VD56G3_REVISION_CUT3) {
        priv->is_fastboot = true;
    } else if (((rev >> 8) & 0xff) == VD56G3_REVISION_CUT2) {
        priv->is_fastboot = false;
    } else {
        ESP_LOGE(TAG, "unsupported cut 0x%" PRIx32, rev);
        goto err;
    }

    uint32_t opt = 0;
    if (vd66gy_cci_read(dev->sccb_handle, REG_OPTICAL_REVISION, &opt) != ESP_OK) {
        ESP_LOGE(TAG, "read optical revision failed");
        goto err;
    }
    if ((uint8_t)opt != VD56G3_OPTICAL_REVISION_BAYER) {
        ESP_LOGE(TAG, "optical revision %" PRIu32 " (expected Bayer module)", opt);
        goto err;
    }

    dev->id = id;
    ESP_LOGI(TAG, "Detected VD66GY PID=0x%x rev=0x%" PRIx32, id.pid, rev);
    return dev;

err:
    vd66gy_power_off(dev);
    free(priv);
    free(dev);
    return NULL;
}

#if CONFIG_CAMERA_VD66GY_AUTO_DETECT_MIPI_INTERFACE_SENSOR
ESP_CAM_SENSOR_DETECT_FN(vd66gy_detect, ESP_CAM_SENSOR_MIPI_CSI, VD66GY_SCCB_ADDR)
{
    ((esp_cam_sensor_config_t *)config)->sensor_port = ESP_CAM_SENSOR_MIPI_CSI;
    return vd66gy_detect(config);
}
#endif
