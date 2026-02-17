#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "freertos/FreeRTOS.h"
#include <math.h>

#define ADC_CHANNEL  ADC_CHANNEL_6
#define ADC_UNIT     ADC_UNIT_1
#define ADC_ATTEN    ADC_ATTEN_DB_12
#define ADC_BITWIDTH ADC_BITWIDTH_12

#define SUP_VOL     3.3f       /* Volts   */
#define VOL_DIV_RES 4640.0f    /* Ohms    */
#define BETA        3977.0f    /* Kelvins */
#define TMP_25      298.15f    /* Kelvins */
#define RES_25      4700.0f    /* Ohms    */

#define MILIVOLTS_TO_VOLTS(mv) ((mv)  / 1000.0f)
#define KEL_TO_DEG_C(kel)      ((kel) - 273.15f)

const char TAG[] = "main";

adc_oneshot_unit_handle_t _hUnit = NULL;
adc_cali_handle_t         _hCali = NULL;

void ADC_Init(void)
{
    adc_cali_line_fitting_config_t caliConfig =
    {
        .unit_id  = ADC_UNIT,
        .atten    = ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH,
    };
    ESP_ERROR_CHECK(adc_cali_create_scheme_line_fitting(&caliConfig, &_hCali));

    adc_oneshot_unit_init_cfg_t unitConfig =
    {
        .unit_id = ADC_UNIT,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unitConfig, &_hUnit));

    adc_oneshot_chan_cfg_t chConfig =
    {
        .atten    = ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(_hUnit, ADC_CHANNEL, &chConfig));
}

float ADC_GetVoltage_V(void)
{
    int adcVal_raw = 0;
    ESP_ERROR_CHECK(adc_oneshot_read(_hUnit, ADC_CHANNEL, &adcVal_raw));

    int vol_mV = 0;
    ESP_ERROR_CHECK(adc_cali_raw_to_voltage(_hCali, adcVal_raw, &vol_mV));

    return MILIVOLTS_TO_VOLTS(vol_mV);
}

float NTC_GetTemperature_degC(void) // From NTCLE100E3472JB0
{
    float vol_V   = ADC_GetVoltage_V();
    float res_ohm = VOL_DIV_RES * vol_V / (SUP_VOL - vol_V);
    float tmp_K   = BETA * TMP_25 / (TMP_25 * logf(res_ohm / RES_25) + BETA);

    return KEL_TO_DEG_C(tmp_K);
}

int app_main()
{
    ADC_Init();

    uint32_t cnt = 0;

    while(true)
    {
        cnt++;

        float tmp_degC = NTC_GetTemperature_degC();

        ESP_LOGI(TAG, "%lu. temperature is %.2f °C", cnt, tmp_degC);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    return 0;
}
