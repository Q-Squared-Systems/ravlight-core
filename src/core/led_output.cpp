#include "core/led_output.h"
#include "esp_log.h"
#if ESP_IDF_VERSION_MAJOR >= 5
#include "soc/soc_caps.h"
#endif
#include <stdlib.h>
#include <string.h>

static const char* TAG = "LED";

// WS2812B-compatible 800 kHz protocol (clk_div=4, APB=80 MHz → 50 ns/tick)
// FastLED's WS2811 type on ESP32 also uses these 800 kHz timings.
// The original WS2811 400 kHz spec (T0H=500 ns) sits above this strip's
// ~600 ns threshold and is misread as '1', causing low-brightness white glitch.
#define WS_T0H  8   //  400 ns  — safely below the ~600 ns '0'/'1' threshold
#define WS_T0L 17   //  850 ns
#define WS_T1H 16   //  800 ns
#define WS_T1L  9   //  450 ns
// Period: T0H+T0L = T1H+T1L = 25 ticks = 1250 ns (800 kHz) ✓

#if ESP_IDF_VERSION_MAJOR < 5
// RMT translator callback — converts raw RGB(W) bytes to RMT items in ISR context.
// Placed in IRAM to avoid cache misses during ISR refill of the hardware FIFO.
// Called multiple times per strip per frame (once per mem_block worth of items).
static void IRAM_ATTR ws2812_to_rmt(const void* src, rmt_item32_t* dest, size_t src_size,
                                     size_t wanted_num, size_t* translated_size, size_t* item_num) {
    const uint8_t* p = (const uint8_t*)src;
    size_t size = 0, num = 0;
    while (size < src_size && num + 8 <= wanted_num) {
        uint8_t byte = *p++;
        for (int bit = 7; bit >= 0; bit--) {
            if (byte & (1 << bit)) {
                dest->level0 = 1; dest->duration0 = WS_T1H;
                dest->level1 = 0; dest->duration1 = WS_T1L;
            } else {
                dest->level0 = 1; dest->duration0 = WS_T0H;
                dest->level1 = 0; dest->duration1 = WS_T0L;
            }
            dest++;
            num++;
        }
        size++;
    }
    *translated_size = size;
    *item_num = num;
}

#endif

esp_err_t led_output_init(led_output_t* out, int gpio_num, uint16_t n_pixels,
                           uint8_t channel, uint8_t mem_blocks, uint8_t channels) {
    out->channel  = channel;
    out->n_pixels = n_pixels;
    out->channels = channels;

    out->buf = (uint8_t*)calloc(n_pixels * channels, 1);
    if (!out->buf) {
        ESP_LOGE(TAG, "buf alloc failed ch%d", channel);
        return ESP_ERR_NO_MEM;
    }

#if ESP_IDF_VERSION_MAJOR >= 5
    out->tx_pending = false;
    out->tx_channel = nullptr;
    out->encoder = nullptr;
    out->tx_buf = static_cast<uint8_t*>(malloc(static_cast<size_t>(n_pixels) * channels));

    rmt_tx_channel_config_t cfg{};
    cfg.gpio_num = static_cast<gpio_num_t>(gpio_num);
    cfg.clk_src = RMT_CLK_SRC_DEFAULT;
    cfg.resolution_hz = 20000000;
    cfg.mem_block_symbols = mem_blocks * SOC_RMT_MEM_WORDS_PER_CHANNEL;
    cfg.trans_queue_depth = 1;

    rmt_bytes_encoder_config_t encoding{};
    encoding.bit0.duration0 = WS_T0H;
    encoding.bit0.level0 = 1;
    encoding.bit0.duration1 = WS_T0L;
    encoding.bit1.duration0 = WS_T1H;
    encoding.bit1.level0 = 1;
    encoding.bit1.duration1 = WS_T1L;
    encoding.flags.msb_first = true;

    esp_err_t err = out->tx_buf ? rmt_new_tx_channel(&cfg, &out->tx_channel) : ESP_ERR_NO_MEM;
    if (err == ESP_OK) err = rmt_new_bytes_encoder(&encoding, &out->encoder);
    if (err == ESP_OK) err = rmt_enable(out->tx_channel);
    if (err != ESP_OK) {
        if (out->encoder) rmt_del_encoder(out->encoder);
        if (out->tx_channel) rmt_del_channel(out->tx_channel);
        free(out->tx_buf);
        free(out->buf);
        *out = {};
        ESP_LOGE(TAG, "ch%d RMT initialization failed: %s", channel, esp_err_to_name(err));
        return err;
    }
#else
    const auto hardware_channel = static_cast<rmt_channel_t>(channel);
    rmt_config_t cfg = RMT_DEFAULT_CONFIG_TX((gpio_num_t)gpio_num, hardware_channel);
    cfg.clk_div       = 4;          // 80 MHz / 4 = 20 MHz → 50 ns/tick
    cfg.mem_block_num = mem_blocks;

    esp_err_t err = rmt_config(&cfg);
    if (err != ESP_OK) { ESP_LOGE(TAG, "rmt_config err %d", err); return err; }

    err = rmt_driver_install(hardware_channel, 0, 0);
    if (err != ESP_OK) { ESP_LOGE(TAG, "rmt_driver_install err %d", err); return err; }

    // Translator callback enables rmt_write_sample() for non-blocking parallel output.
    // Also harmless for channels using the blocking led_output_flush() path (Veyron).
    rmt_translator_init(hardware_channel, ws2812_to_rmt);
#endif

    ESP_LOGI(TAG, "ch%d gpio%d n=%d mem=%d", channel, gpio_num, n_pixels, mem_blocks);
    return ESP_OK;
}

void led_output_set_pixel(led_output_t* out, uint16_t idx, uint8_t r, uint8_t g, uint8_t b) {
    if (idx >= out->n_pixels) return;
    out->buf[idx * out->channels]     = r;
    out->buf[idx * out->channels + 1] = g;
    out->buf[idx * out->channels + 2] = b;
}

void led_output_write_raw(led_output_t* out, uint16_t idx, const uint8_t* src) {
    if (idx >= out->n_pixels) return;
    memcpy(out->buf + idx * out->channels, src, out->channels);
}

static volatile uint32_t s_tx_timeouts = 0;

void led_output_flush(led_output_t* out) {
    led_output_flush_async(out);
    led_output_wait_done(out);
}

void led_output_flush_async(led_output_t* out) {
#if ESP_IDF_VERSION_MAJOR >= 5
    if (!out->tx_channel) return;
    if (out->tx_pending && rmt_tx_wait_all_done(out->tx_channel, 0) != ESP_OK)
        return;

    const size_t bytes = static_cast<size_t>(out->n_pixels) * out->channels;
    memcpy(out->tx_buf, out->buf, bytes);
    rmt_transmit_config_t cfg{};
    cfg.flags.queue_nonblocking = true;
    const esp_err_t err = rmt_transmit(out->tx_channel, out->encoder, out->tx_buf, bytes, &cfg);
    out->tx_pending = (err == ESP_OK);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "ch%d RMT submit failed: %s", out->channel, esp_err_to_name(err));
    }
#else
    rmt_write_sample(static_cast<rmt_channel_t>(out->channel), out->buf,
                     (size_t)out->n_pixels * out->channels, false);
#endif
}

void led_output_wait_done(led_output_t* out) {
#if ESP_IDF_VERSION_MAJOR >= 5
    if (!out->tx_pending) return;
    const esp_err_t werr = rmt_tx_wait_all_done(out->tx_channel, 100);
    if (werr == ESP_OK) out->tx_pending = false;
#else
    const esp_err_t werr = rmt_wait_tx_done(static_cast<rmt_channel_t>(out->channel),
                                           pdMS_TO_TICKS(100));
#endif
    if (werr != ESP_OK) {
        s_tx_timeouts = s_tx_timeouts + 1;
        ESP_LOGW(TAG, "ch%d tx wait failed: %s", out->channel, esp_err_to_name(werr));
    }
}

uint32_t led_output_timeout_count(void) { return s_tx_timeouts; }

void led_output_clear(led_output_t* out) {
    memset(out->buf, 0, out->n_pixels * out->channels);
}

void led_output_deinit(led_output_t* out) {
#if ESP_IDF_VERSION_MAJOR >= 5
    if (out->tx_channel) {
        if (rmt_disable(out->tx_channel) != ESP_OK ||
            rmt_del_channel(out->tx_channel) != ESP_OK) {
            ESP_LOGE(TAG, "ch%d RMT teardown failed", out->channel);
            return;
        }
        rmt_del_encoder(out->encoder);
        free(out->tx_buf);
    }
#else
    rmt_driver_uninstall(static_cast<rmt_channel_t>(out->channel));
#endif
    free(out->buf);
    *out = {};
}
