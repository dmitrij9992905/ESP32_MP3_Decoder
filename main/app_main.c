
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "http.h"
#include "driver/i2s.h"
#include "driver/i2c.h"

#include "i2c_init.h"
#include "vector.h"
#include "ui.h"
#include "spiram_fifo.h"
#include "audio_renderer.h"
#include "web_radio.h"
#include "playerconfig.h"
#include "wifi.h"
#include "app_main.h"
#ifdef CONFIG_BT_SPEAKER_MODE
#include "bt_speaker.h"
#endif
#include "playlist.h"

#include "fm_radio.h"

#define WIFI_LIST_NUM   10


#define TAG "main"


//Priorities of the reader and the decoder thread. bigger number = higher prio
#define PRIO_READER configMAX_PRIORITIES -3
#define PRIO_MQTT configMAX_PRIORITIES - 3
#define PRIO_CONNECT configMAX_PRIORITIES -1

void fmRSSI(void * parameter) {
	TRDA5807MRegisterFileRead radio;
	uint8_t signal;
	while(1) {
		//getRegisterBulkByFile(&radio);
		signal = getRSSI();
		ESP_LOGI(TAG, "FM RSSI = %d", signal);
		vTaskDelay(500 / portTICK_PERIOD_MS);
		
	}
	
}

void get_fm_state(void * parameter) {
	TRDA5807MRegisterFileRead *radio = malloc(sizeof(TRDA5807MRegisterFileRead));
	//uint8_t signal;
	
	while(1) {
		if(getRegisterBulkByFile(radio)) {
			ESP_LOGI(TAG, "FM RSSI = %d", radio->rssi);
			ESP_LOGI(TAG, "FM ready = %d", radio->ready);
		}
		else {
			ESP_LOGE(TAG, "FM get state error!");		
		}
		//signal = getRSSI();
		//ESP_LOGI(TAG, "FM RSSI = %d", signal);
		vTaskDelay(500 / portTICK_PERIOD_MS);
		
	}
	
}

static void init_hardware()
{
    nvs_flash_init();
	esp_err_t err;
    // init UI
    // ui_init(GPIO_NUM_32);

    //Initialize the SPI RAM chip communications and see if it actually retains some bytes. If it
    //doesn't, warn user.
    if (!spiRamFifoInit()) {
        printf("\n\nSPI RAM chip fail!\n");
        while(1);
    }

	if ((err = i2c_common_init(CONFIG_I2C_NUM, CONFIG_I2C_SCL, CONFIG_I2C_SDA, CONFIG_I2C_FREQ)) != ESP_OK) {
        ESP_LOGE(TAG, "%s I2C initialization error: %s\n", __func__, esp_err_to_name(err));
        //return;
    }
	else ESP_LOGI(TAG, "I2C initialized successfully!");

	#ifdef CONFIG_RDA5807_FM_RADIO
	radio_start(RDA5807M_BAND_WORLD, CONFIG_I2C_NUM);
	setFrequency(10250);
	unMute(true);
	xTaskCreate(get_fm_state, "fm_state", 2048, NULL, 10, NULL);
	#endif

    ESP_LOGI(TAG, "hardware initialized");
}

static void start_wifi()
{
    ESP_LOGI(TAG, "starting network");

    /* FreeRTOS event group to signal when we are connected & ready to make a request */
    EventGroupHandle_t wifi_event_group = xEventGroupCreate();

    /* init wifi */
    ui_queue_event(UI_CONNECTING);
    initialise_wifi(wifi_event_group);

    /* Wait for the callback to set the CONNECTED_BIT in the event group. */
    xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT,
                        false, true, portMAX_DELAY);

    ui_queue_event(UI_CONNECTED);
}

static renderer_config_t *create_renderer_config()
{
    renderer_config_t *renderer_config = calloc(1, sizeof(renderer_config_t));

    renderer_config->bit_depth = I2S_BITS_PER_SAMPLE_16BIT;
    renderer_config->i2s_num = I2S_NUM_0;
    renderer_config->sample_rate = 44100;
    renderer_config->sample_rate_modifier = 1.0;
    renderer_config->output_mode = AUDIO_OUTPUT_MODE;

    if(renderer_config->output_mode == I2S_MERUS) {
        renderer_config->bit_depth = I2S_BITS_PER_SAMPLE_32BIT;
    }

    if(renderer_config->output_mode == DAC_BUILT_IN) {
        renderer_config->bit_depth = I2S_BITS_PER_SAMPLE_16BIT;
    }
	if (renderer_config->output_mode == I2S_MERUS || renderer_config->output_mode == I2S) {
		i2s_pin_config_t pin_config = {
		        //.bck_io_num = GPIO_NUM_26,
				.bck_io_num = CONFIG_I2S_BCK_PIN,
		        //.ws_io_num = GPIO_NUM_25,
				.ws_io_num = CONFIG_I2S_LRCK_PIN,
		        //.data_out_num = GPIO_NUM_22,
				.data_out_num = CONFIG_I2S_DATA_PIN,
		        .data_in_num = I2S_PIN_NO_CHANGE
		};
		renderer_set_pin_config(pin_config);
	}
	
	
    return renderer_config;
}

static void start_web_radio()
{
    // init web radio
    web_radio_t *radio_config = calloc(1, sizeof(web_radio_t));
    radio_config->playlist = playlist_create();
    playlist_load_pls(radio_config->playlist);


    // init player config
    radio_config->player_config = calloc(1, sizeof(player_t));
    radio_config->player_config->command = CMD_NONE;
    radio_config->player_config->decoder_status = UNINITIALIZED;
    radio_config->player_config->decoder_command = CMD_NONE;
    radio_config->player_config->buffer_pref = BUF_PREF_SAFE;
    radio_config->player_config->media_stream = calloc(1, sizeof(media_stream_t));

    // init renderer
    renderer_init(create_renderer_config());

    // start radio
    web_radio_init(radio_config);
    web_radio_start(radio_config);
}

/**
 * entry point
 */
void app_main()
{
    ESP_LOGI(TAG, "starting app_main()");
    ESP_LOGI(TAG, "RAM left: %u", esp_get_free_heap_size());

    init_hardware();

#ifdef CONFIG_BT_SPEAKER_MODE
    bt_speaker_start(create_renderer_config());
#else
    start_wifi();
    start_web_radio();
#endif

    ESP_LOGI(TAG, "RAM left %d", esp_get_free_heap_size());
    // ESP_LOGI(TAG, "app_main stack: %d\n", uxTaskGetStackHighWaterMark(NULL));
}
