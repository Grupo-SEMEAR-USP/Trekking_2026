#include <stdlib.h>
#include <string.h>
#include "esp_compiler.h"
#include "esp_log.h"
#include "driver/pulse_cnt.h"
#include "rotary_encoder.h"

#define EC11_PCNT_DEFAULT_HIGH_LIMIT (30000)
#define EC11_PCNT_DEFAULT_LOW_LIMIT  (-30000)

typedef struct {
    rotary_encoder_t parent;
    pcnt_unit_handle_t pcnt_unit;
    pcnt_channel_handle_t chan_a;
    pcnt_channel_handle_t chan_b;
    int accumu_count;
} ec11_t;

static esp_err_t ec11_set_glitch_filter(rotary_encoder_t *encoder, uint32_t max_glitch_us)
{
    ec11_t *ec11 = __containerof(encoder, ec11_t, parent);
    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = max_glitch_us * 1000,
    };
    ESP_ERROR_CHECK(pcnt_unit_disable(ec11->pcnt_unit));
    esp_err_t ret = pcnt_unit_set_glitch_filter(ec11->pcnt_unit, &filter_config);
    ESP_ERROR_CHECK(pcnt_unit_enable(ec11->pcnt_unit));
    return ret;
}

static esp_err_t ec11_start(rotary_encoder_t *encoder)
{
    ec11_t *ec11 = __containerof(encoder, ec11_t, parent);
    return pcnt_unit_start(ec11->pcnt_unit);
}

static esp_err_t ec11_stop(rotary_encoder_t *encoder)
{
    ec11_t *ec11 = __containerof(encoder, ec11_t, parent);
    return pcnt_unit_stop(ec11->pcnt_unit);
}

static int ec11_get_counter_value(rotary_encoder_t *encoder)
{
    ec11_t *ec11 = __containerof(encoder, ec11_t, parent);
    int count = 0;
    ESP_ERROR_CHECK(pcnt_unit_get_count(ec11->pcnt_unit, &count));
    return count;
}

static esp_err_t ec11_reset_counter_value(rotary_encoder_t *encoder)
{
    ec11_t *ec11 = __containerof(encoder, ec11_t, parent);
    ec11->accumu_count = 0;
    return pcnt_unit_clear_count(ec11->pcnt_unit);
}

static esp_err_t ec11_del(rotary_encoder_t *encoder)
{
    ec11_t *ec11 = __containerof(encoder, ec11_t, parent);
    pcnt_unit_stop(ec11->pcnt_unit);
    pcnt_unit_disable(ec11->pcnt_unit);
    pcnt_del_channel(ec11->chan_a);
    pcnt_del_channel(ec11->chan_b);
    pcnt_del_unit(ec11->pcnt_unit);
    free(ec11);
    return ESP_OK;
}

esp_err_t rotary_encoder_new_ec11(const rotary_encoder_config_t *config, rotary_encoder_t **ret_encoder)
{
    esp_err_t ret = ESP_OK;
    ec11_t *ec11 = calloc(1, sizeof(ec11_t));
    if (!ec11) {
        return ESP_ERR_NO_MEM;
    }

    // Configuração da Unidade PCNT
    pcnt_unit_config_t unit_config = {
        .high_limit = EC11_PCNT_DEFAULT_HIGH_LIMIT,
        .low_limit = EC11_PCNT_DEFAULT_LOW_LIMIT,
        .flags.accum_count = 1,
    };
    ret = pcnt_new_unit(&unit_config, &ec11->pcnt_unit);
    if (ret != ESP_OK) goto err;

    // Configuração dos Canais (Fases A e B)
    pcnt_chan_config_t chan_a_config = {
        .edge_gpio_num = config->phase_a_gpio_num,
        .level_gpio_num = config->phase_b_gpio_num,
    };
    ret = pcnt_new_channel(ec11->pcnt_unit, &chan_a_config, &ec11->chan_a);
    if (ret != ESP_OK) goto err;

    pcnt_chan_config_t chan_b_config = {
        .edge_gpio_num = config->phase_b_gpio_num,
        .level_gpio_num = config->phase_a_gpio_num,
    };
    ret = pcnt_new_channel(ec11->pcnt_unit, &chan_b_config, &ec11->chan_b);
    if (ret != ESP_OK) goto err;

    // Definição das ações para decodificação em quadratura
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(ec11->chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(ec11->chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(ec11->chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(ec11->chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    // Callbacks para eventos de limite (overflow)
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(ec11->pcnt_unit, EC11_PCNT_DEFAULT_HIGH_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(ec11->pcnt_unit, EC11_PCNT_DEFAULT_LOW_LIMIT));

    ESP_ERROR_CHECK(pcnt_unit_enable(ec11->pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(ec11->pcnt_unit));

    ec11->parent.del = ec11_del;
    ec11->parent.start = ec11_start;
    ec11->parent.stop = ec11_stop;
    ec11->parent.set_glitch_filter = ec11_set_glitch_filter;
    ec11->parent.get_counter_value = ec11_get_counter_value;
    ec11->parent.reset_counter_value = ec11_reset_counter_value;

    *ret_encoder = &(ec11->parent);
    return ESP_OK;

err:
    if (ec11->chan_a) pcnt_del_channel(ec11->chan_a);
    if (ec11->chan_b) pcnt_del_channel(ec11->chan_b);
    if (ec11->pcnt_unit) pcnt_del_unit(ec11->pcnt_unit);
    free(ec11);
    return ret;
}
