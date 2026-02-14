#include "temperature.h"
#include "esp_adc/adc_oneshot.h"
#include "config.h"
#include "time_utils.h"
#include <cmath>

static bool adc_initialized = false;
static adc_oneshot_unit_handle_t adc_handle = nullptr;

static void init_adc() {
    if (adc_initialized) return;

    adc_oneshot_unit_init_cfg_t unit_cfg = {};
    unit_cfg.unit_id = ADC_UNIT_1;
    unit_cfg.ulp_mode = ADC_ULP_MODE_DISABLE;
    if (adc_oneshot_new_unit(&unit_cfg, &adc_handle) != ESP_OK) {
        return;
    }

    adc_oneshot_chan_cfg_t chan_cfg = {};
    chan_cfg.atten = ADC_ATTEN_DB_11;
    chan_cfg.bitwidth = ADC_BITWIDTH_12;
    adc_oneshot_config_channel(adc_handle, ADC_CHANNEL_0, &chan_cfg); // GPIO36

    adc_initialized = true;
}

int readTemperatureC() {
    init_adc();
    static int lastValidTemp = 20;
    if (!adc_initialized) return lastValidTemp;

    long sum = 0;
    const int samples = 10;
    for (int i = 0; i < samples; i++) {
        int raw = 0;
        if (adc_oneshot_read(adc_handle, ADC_CHANNEL_0, &raw) == ESP_OK) {
            sum += raw;
        }
    }
    float avgAdc = sum / (float)samples;
    float voltage = (avgAdc / 4095.0f) * 3.3f;
    float currentTemp = voltage / 0.01f;

    if (currentTemp < TEMP_MIN_VALID_C) return lastValidTemp;

    struct Sample { float temp; uint32_t ms; };
    static Sample history[64];
    static int head = 0;
    static bool historyInit = false;

    uint32_t nowMs = (uint32_t)time_utils::millis();
    history[head] = {currentTemp, nowMs};
    head = (head + 1) % 64;
    if (!historyInit && head == 0) historyInit = true;

    float temps[64];
    int count = 0;
    int limit = historyInit ? 64 : head;
    for (int i = 0; i < limit; i++) {
        uint32_t age = nowMs - history[i].ms;
        if (age <= TEMP_FILTER_WINDOW_MS) {
            temps[count++] = history[i].temp;
        }
    }
    if (count == 0) return lastValidTemp;

    // Median
    for (int i = 0; i < count - 1; i++) {
        for (int j = 0; j < count - i - 1; j++) {
            if (temps[j] > temps[j + 1]) {
                float tmp = temps[j];
                temps[j] = temps[j + 1];
                temps[j + 1] = tmp;
            }
        }
    }
    float median = (count % 2 == 1)
        ? temps[count / 2]
        : 0.5f * (temps[count / 2 - 1] + temps[count / 2]);

    // Remove outliers from median and average remaining
    float sumFiltered = 0.0f;
    int kept = 0;
    for (int i = 0; i < count; i++) {
        float t = temps[i];
        if (std::fabs(t - median) <= TEMP_OUTLIER_C) {
            sumFiltered += t;
            kept++;
        }
    }
    float filtered = (kept > 0) ? (sumFiltered / kept) : median;

    filtered += TEMP_CALIB_OFFSET_C;
    lastValidTemp = (int)filtered;
    return lastValidTemp;
}
