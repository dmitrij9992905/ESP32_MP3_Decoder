/*
 * audio_renderer.h
 *
 *  Created on: 12.03.2017
 *      Author: michaelboeckling
 */

#ifndef INCLUDE_AUDIO_RENDERER_H_
#define INCLUDE_AUDIO_RENDERER_H_

#include "freertos/FreeRTOS.h"
#include "driver/i2s.h"
#include "common_component.h"

typedef enum {
    I2S, I2S_MERUS, DAC_BUILT_IN, PDM
} output_mode_t;


typedef struct
{
    output_mode_t output_mode;
    int sample_rate;
    float sample_rate_modifier;
    i2s_bits_per_sample_t bit_depth;
    i2s_port_t i2s_num;
} renderer_config_t;


static i2s_pin_config_t pin_config = {
            .bck_io_num = GPIO_NUM_26,
			//.bck_io_num = I2S_BCK_PIN,
            .ws_io_num = GPIO_NUM_25,
			//.ws_io_num = I2S_LRCK_PIN,
            .data_out_num = GPIO_NUM_22,
			//.data_out_num = I2S_DATA_PIN,
            .data_in_num = I2S_PIN_NO_CHANGE
};

void renderer_set_pin_config(i2s_pin_config_t input_config);

/* ESP32 is Little Endian, I2S is Big Endian.
 *
 * Samples produced by a decoder will probably be LE,
 * and I2S recordings BE.
 */
typedef enum
{
    PCM_INTERLEAVED, PCM_LEFT_RIGHT
} pcm_buffer_layout_t;

typedef enum
{
    PCM_BIG_ENDIAN, PCM_LITTLE_ENDIAN
} pcm_endianness_t;

typedef struct
{
    uint32_t sample_rate;
    i2s_bits_per_sample_t bit_depth;
    uint8_t num_channels;
    pcm_buffer_layout_t buffer_format;
    pcm_endianness_t endianness; // currently unused
} pcm_format_t;


/* generic renderer interface */
void render_samples(char *buf, uint32_t len, pcm_format_t *format);

void renderer_init(renderer_config_t *config);
void renderer_start();
void renderer_stop();
void renderer_destroy();

void renderer_zero_dma_buffer();
renderer_config_t *renderer_get();


#endif /* INCLUDE_AUDIO_RENDERER_H_ */
