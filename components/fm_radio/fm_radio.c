/* Arduino RDA5807M Library
 * See the README file for author and licensing information. In case it's
 * missing from your distribution, use the one here as the authoritative
 * version: https://github.com/csdexter/RDA5807M/blob/master/README
 *
 * This library is for interfacing with a RDA Microelectronics RDA5807M
 * single-chip FM broadcast radio receiver.
 * See the example sketches to learn how to use the library in your code.
 *
 * This is the main code file for the library.
 * See the header file for better function documentation.
 */

#include "fm_radio.h"
#include "fm_radio_private.h"
#include <stdbool.h>

#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define WRITE_BIT I2C_MASTER_WRITE  /*!< I2C master write */
//#define WRITE_BIT 0x00
#define READ_BIT I2C_MASTER_READ    /*!< I2C master read */
//#define READ_BIT 0x01
#define ACK_CHECK_EN 0x1            /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0           /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                 /*!< I2C ack value */
#define NACK_VAL 0x1                /*!< I2C nack value */

static const char *FM_TAG = "FM";

int i2c_port_num;

void radio_start(uint8_t band, i2c_port_t port_num) {
    //Wire.begin();
	i2c_port_num = port_num;
    setRegister(RDA5807M_REG_CONFIG, RDA5807M_FLG_DHIZ | RDA5807M_FLG_DMUTE | 
                RDA5807M_FLG_BASS | RDA5807M_FLG_SEEKUP | RDA5807M_FLG_RDS | 
                RDA5807M_FLG_NEW | RDA5807M_FLG_ENABLE);
    updateRegister(RDA5807M_REG_TUNING, RDA5807M_BAND_MASK, band);
	ESP_LOGI(FM_TAG, "FM radio intialized on I2C port %d", i2c_port_num);
};

void radio_stop(void) {
    setRegister(RDA5807M_REG_CONFIG, 0x00);
};

bool setRegister(uint8_t reg, const uint16_t value) {
	ESP_LOGI(FM_TAG, "Trying to set FM radio register on I2C port %d", i2c_port_num);
	uint8_t h = value>>8;
	uint8_t l = value&(0xFF);
	ESP_LOGI(FM_TAG, "H = 0x%02x ", h);
	ESP_LOGI(FM_TAG, "L = 0x%02x", l);
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
	//i2c_master_write_byte(cmd, RDA5807M_I2C_ADDR_RANDOM | WRITE_BIT, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, RDA5807M_I2C_ADDR_RANDOM | WRITE_BIT, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, h, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, l, ACK_CHECK_EN);
	i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_port_num, cmd, 1500 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
	if (ret == ESP_OK) {
		return true;
	} else {
		return false;
	}
	/*
    Wire.beginTransmission(RDA5807M_I2C_ADDR_RANDOM);
    Wire.write(reg);
    Wire.write(highByte(value));
    Wire.write(lowByte(value));
    Wire.endTransmission(true);
	*/
};

uint16_t getRegister(uint8_t reg) {
	ESP_LOGI(FM_TAG, "Trying to read FM radio register on I2C port %d", i2c_port_num);
    uint16_t result;
	uint8_t *h = malloc(1);
	uint8_t *l = malloc(1);
	//uint8_t *data = malloc(2);
	esp_err_t ret;
	ESP_LOGI(FM_TAG, "I2C number: %d", i2c_port_num);
	//vTaskDelay(500 / portTICK_RATE_MS);
	// Действуем по алгоритму, описанному в документации ESP-IDF
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (RDA5807M_I2C_ADDR_RANDOM | WRITE_BIT), ACK_CHECK_EN);
	//i2c_master_write_byte(cmd, 0b00100010, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
	//i2c_master_stop(cmd);
	//ret = i2c_master_cmd_begin(i2c_port_num, cmd, 500 / portTICK_RATE_MS);
	//i2c_cmd_link_delete(cmd);
	//vTaskDelay(30 / portTICK_RATE_MS);
	//ESP_LOGI(FM_TAG, "Write result: %d", ret);
	//cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (RDA5807M_I2C_ADDR_RANDOM | READ_BIT), ACK_CHECK_EN);
	//i2c_master_write_byte(cmd, 0b00100011, ACK_CHECK_EN);
	ESP_LOGI(FM_TAG, "I2C reading: %d", i2c_port_num);
	i2c_master_read_byte(cmd, h, ACK_VAL);
	i2c_master_read_byte(cmd, l, NACK_VAL);
	//i2c_master_read_byte(cmd, data, ACK_VAL);
	//i2c_master_read_byte(cmd, data+1, NACK_VAL);
	ESP_LOGI(FM_TAG, "I2C end reading: %d", i2c_port_num);
	i2c_master_stop(cmd);
	ret = i2c_master_cmd_begin(i2c_port_num, cmd, 1500 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);
	vTaskDelay(1000 / portTICK_RATE_MS);
	ESP_LOGI(FM_TAG, "H = 0x%02x ", *(h));
	ESP_LOGI(FM_TAG, "L = 0x%02x ", *(l));
	//ESP_LOGI(FM_TAG, "Read result: %d, H: %d, L: %d", ret, &h, &l);
	// Складываем два байта в слово и выдаем
	result = (uint16_t)*(h) << 8;
	result |= *(l);
	free(h);
	free(l);	
    return result;
};

bool setRegisterBulk(uint8_t count, const uint16_t regs[]) {
	/*
    Wire.beginTransmission(RDA5807M_I2C_ADDR_SEQRDA);

    for(byte i=0; i < count; i++) {
        Wire.write(highByte(regs[i]));
        Wire.write(lowByte(regs[i]));
    };

    Wire.endTransmission(true);
	*/
	uint8_t *h = malloc(1);
	uint8_t *l = malloc(1);
	
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
	i2c_master_write_byte(cmd, RDA5807M_I2C_ADDR_SEQRDA | WRITE_BIT, ACK_CHECK_EN);
	// Отправляем все слова, кроме последнего
	for(uint8_t i=0; i < count-1; i++) {
		h = regs[i]>>8;
		l = regs[i]&(0xFF);
		i2c_master_write_byte(cmd, h, ACK_CHECK_EN);
		i2c_master_write_byte(cmd, l, ACK_CHECK_EN);
    };
	// Отправляем последнее слово и прекращаем передачу
	h = regs[count-1]>>8;
	l = regs[count-1]&(0xFF);
	i2c_master_write_byte(cmd, h, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, l, ACK_CHECK_EN);
	i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_port_num, cmd, 50 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
	free(h);
	free(l);
	if (ret == ESP_OK) {
		return true;
	} else {
		return false;
	}
};

bool getRegisterBulk(uint8_t count, uint16_t regs[]) {
	// BUG: regs[count] вместо regs[i]
	/*
    Wire.requestFrom(RDA5807M_I2C_ADDR_SEQRDA, count * 2, true);

    for(byte i=0; i < count; i++) {
        //Don't let gcc play games on us, enforce order of execution.
        regs[count] = (word)Wire.read() << 8;
        regs[count] |= Wire.read();
    };
	*/
	uint8_t *h = malloc(1);
	uint8_t *l = malloc(1);
	// Создаем последовательность команд
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	// Даем модулю запрос на чтение всех регистров
    i2c_master_start(cmd);
	i2c_master_write_byte(cmd, RDA5807M_I2C_ADDR_SEQRDA | READ_BIT, ACK_CHECK_EN);
	// Читаем все слова, кроме последнего
	for(uint8_t i=0; i < count-1; i++) {
        //Don't let gcc play games on us, enforce order of execution.
		i2c_master_read_byte(cmd, h, ACK_CHECK_EN);
        regs[i] = (uint16_t)*(h) << 8;
		i2c_master_read_byte(cmd, l, ACK_CHECK_EN);
        regs[i] |= *(l);
    };
	// Читаем последнее слово
	i2c_master_read_byte(cmd, h, ACK_CHECK_EN);
	regs[count-1] = (uint16_t)*(h) << 8;
	i2c_master_read_byte(cmd, l, ACK_CHECK_DIS);
	regs[count-1] |= *(l);
	// Завершаем последовательность команд и отправляем ее модулю I2C
	i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_port_num, cmd, 50 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
	i2c_cmd_link_delete(cmd);
	free(h);
	free(l);
	if (ret == ESP_OK) {
		return true;
	} else {
		return false;
	}
};

bool setRegisterBulkByFile(const TRDA5807MRegisterFileWrite *regs) {
    const uint8_t * const ptr = (uint8_t *)regs;

	/*
    Wire.beginTransmission(RDA5807M_I2C_ADDR_SEQRDA);

    for(byte i=0; i < sizeof(TRDA5807MRegisterFileWrite); i++)
        Wire.write(ptr[i]);

    Wire.endTransmission(true);
	*/
	
	// Создаем последовательность команд
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	// Даем модулю запрос на запись всех регистров
    i2c_master_start(cmd);
	i2c_master_write_byte(cmd, RDA5807M_I2C_ADDR_SEQRDA | WRITE_BIT, ACK_CHECK_EN);
	// Отправляем структуру побайтово, кроме последнего байта
	for(uint8_t i=0; i < sizeof(TRDA5807MRegisterFileWrite)-1; i++)
        i2c_master_write_byte(cmd, ptr[i], ACK_CHECK_EN);
	// Отправляем последний байт с завершением передачи
	i2c_master_write_byte(cmd, ptr[sizeof(TRDA5807MRegisterFileWrite)-1], ACK_CHECK_DIS);
	i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_port_num, cmd, 50 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
	if (ret == ESP_OK) {
		return true;
	} else {
		return false;
	}
};

bool getRegisterBulkByFile(TRDA5807MRegisterFileRead *regs) {
    uint8_t * const ptr = (uint8_t *)regs;
	/*
    Wire.requestFrom(RDA5807M_I2C_ADDR_SEQRDA,
                     sizeof(TRDA5807MRegisterFileRead), true);

    for(uint8_t i=0; i < sizeof(TRDA5807MRegisterFileRead); i++)
        ptr[i] = Wire.read();
	*/
	// Создаем последовательность команд
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	// Даем модулю запрос на чтение всех регистров побайтово в структуру
    i2c_master_start(cmd);
	i2c_master_write_byte(cmd, RDA5807M_I2C_ADDR_SEQRDA | READ_BIT, ACK_CHECK_EN);
	// Чтение всех байт, кроме последнего
	for(uint8_t i=0; i < sizeof(TRDA5807MRegisterFileRead)-1; i++)
        i2c_master_read_byte(cmd, ptr[i], ACK_CHECK_EN);
	// Чтение последнего байта и завершение
	i2c_master_read_byte(cmd, ptr[sizeof(TRDA5807MRegisterFileRead)-1], ACK_CHECK_DIS);
	i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_port_num, cmd, 50 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
	if (ret == ESP_OK) {
		return true;
	} else {
		ESP_LOGE(FM_TAG, "FM get status error code: %d", ret);
		return false;
	}
};

void updateRegister(uint8_t reg, uint16_t mask, uint16_t value) {
	setRegister(reg, (getRegister(reg) & ~mask) | value);
};

bool volumeUp(void) {
    const uint8_t volume = getRegister(RDA5807M_REG_VOLUME) & RDA5807M_VOLUME_MASK;

    if (volume == RDA5807M_VOLUME_MASK)
        return false;
    else {
        updateRegister(RDA5807M_REG_VOLUME, RDA5807M_VOLUME_MASK, volume + 1);
        return true;
    };
};

bool volumeDown(bool alsoMute) {
    const uint8_t volume = getRegister(RDA5807M_REG_VOLUME) & RDA5807M_VOLUME_MASK;

    if (volume) {
        updateRegister(RDA5807M_REG_VOLUME, RDA5807M_VOLUME_MASK, volume - 1);
        if(!(volume - 1) && alsoMute)
            //If we are to trust the datasheet, this is superfluous as a volume
            //of zero triggers mute & HiZ on its own.
            mute();
        return true;
    } else 
        return false;
};

void seekUp(bool wrap) {
    updateRegister(RDA5807M_REG_CONFIG,
                   (RDA5807M_FLG_SEEKUP | RDA5807M_FLG_SEEK |
                    RDA5807M_FLG_SKMODE), 
                   (RDA5807M_FLG_SEEKUP | RDA5807M_FLG_SEEK |
                    (wrap ? 0x00 : RDA5807M_FLG_SKMODE)));
};

void seekDown(bool wrap) {
    updateRegister(RDA5807M_REG_CONFIG,
                   (RDA5807M_FLG_SEEKUP | RDA5807M_FLG_SEEK |
                    RDA5807M_FLG_SKMODE), 
                   (0x00 | RDA5807M_FLG_SEEK |
                    (wrap ? 0x00 : RDA5807M_FLG_SKMODE)));
};

void mute(void) {
    updateRegister(RDA5807M_REG_CONFIG, RDA5807M_FLG_DMUTE, 0x00);
};

void unMute(bool minVolume) {
    if (minVolume)
        updateRegister(RDA5807M_REG_VOLUME, RDA5807M_VOLUME_MASK, 0x1);
    updateRegister(RDA5807M_REG_CONFIG, RDA5807M_FLG_DMUTE, RDA5807M_FLG_DMUTE);
};

const uint16_t RDA5807M_BandLowerLimits[5] = { 8700, 7600, 7600, 6500, 5000 };
const uint16_t RDA5807M_BandHigherLimits[5] = { 10800, 9100, 10800, 7600, 6500 };
const uint8_t RDA5807M_ChannelSpacings[4] = { 100, 200, 50, 25 };

uint16_t getBandAndSpacing(void) {
    uint8_t band = getRegister(RDA5807M_REG_TUNING) & (RDA5807M_BAND_MASK |
                                                    RDA5807M_SPACE_MASK);
    //Separate channel spacing
    const uint8_t space = band & RDA5807M_SPACE_MASK;

    if ((band&RDA5807M_BAND_MASK) == RDA5807M_BAND_EAST && 
        !(getRegister(RDA5807M_REG_BLEND) & RDA5807M_FLG_EASTBAND65M))
        //Lower band limit is 50MHz
        band = (band >> RDA5807M_BAND_SHIFT) + 1;
    else band >>= RDA5807M_BAND_SHIFT;
	
	uint16_t out;
	out = (uint16_t)space << 8;
	out |= band;
    return out;
};

uint16_t getFrequency(void) {
    const uint16_t spaceandband = getBandAndSpacing();

    return (RDA5807M_BandLowerLimits[spaceandband&0xFF]) +
        (getRegister(RDA5807M_REG_STATUS) & RDA5807M_READCHAN_MASK) *
        (RDA5807M_ChannelSpacings[spaceandband>>8]) / 10;
};

bool setFrequency(uint16_t frequency) {
    const uint16_t spaceandband = getBandAndSpacing();
    const uint16_t origin = (RDA5807M_BandLowerLimits[spaceandband&0xFF]);

    //Check that specified frequency falls within our current band limits
    if (frequency < origin ||
        frequency > (RDA5807M_BandHigherLimits[spaceandband&0xFF]))
        return false;

    //Adjust start offset
    frequency -= origin;

    const uint8_t spacing = (RDA5807M_ChannelSpacings[(spaceandband>>8)]);

    //Check that the given frequency can be tuned given current channel spacing
    if (frequency * 10 % spacing)
        return false;

    //Attempt to tune to the requested frequency
    updateRegister(RDA5807M_REG_TUNING, RDA5807M_CHAN_MASK | RDA5807M_FLG_TUNE,
                   ((frequency * 10 / spacing) << RDA5807M_CHAN_SHIFT) |
                   RDA5807M_FLG_TUNE);

    return true;
};



uint8_t getRSSI(void) {
    return (getRegister(RDA5807M_REG_RSSI) & RDA5807M_RSSI_MASK) >> RDA5807M_RSSI_SHIFT;
};
