#include <stdio.h>
#include <inttypes.h>
#include <string.h>

#include <esp_log.h>
#include "esp_err.h"
#include "esp_adc/adc_continuous.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"

/* GPIO definitions*/

#define MOSI    23
#define SCK     18
#define CS       5
#define RST     17
#define DC      16

#define SSD1322_WIDTH   256
#define SSD1322_HEIGHT   64

#define MAXCOLS 240  
#define MAXROWS 64


enum
{
    LO,
    HI
};

static spi_device_handle_t spi;

/************************************************************
 * SPI callback
 ************************************************************/
static void IRAM_ATTR pre_cb(spi_transaction_t *t)
{
    gpio_set_level(DC, (int)(intptr_t)t->user);
}

static void spi_bus_init(void) // wrapper function for spi_bus_config and spi_bus_initialize, 
                                //needed as scope of buscfg is local
{
    spi_bus_config_t buscfg = {
        .mosi_io_num = MOSI,
        .miso_io_num = -1,
        .sclk_io_num = SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 8192 + 32
    };

    ESP_ERROR_CHECK(
        spi_bus_initialize(
            SPI2_HOST,
            &buscfg,
            SPI_DMA_CH_AUTO)
    );
}

static void config_gpio(void)  //wrapper function for gpio_config and bus_init
{
    gpio_config_t output_conf = {
        .pin_bit_mask = (1ULL << RST) | (1ULL << DC),

        .mode = GPIO_MODE_OUTPUT,
        .intr_type = GPIO_INTR_DISABLE
    };

    ESP_ERROR_CHECK(
        gpio_config(&output_conf)
    );
}

static void spi_interface_conf(void) //wrapper function for spi_device_interface_config and spi_bus_add_device, needed as scope of devcfg is local
{
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 10000000,
        .mode = 0,
        .spics_io_num = CS,
        .queue_size = 7,
        .pre_cb = pre_cb
    };

    ESP_ERROR_CHECK(
        spi_bus_add_device(
            SPI2_HOST,
            &devcfg,
            &spi)
    );
}

/************************************************************
 * Send command
 ************************************************************/
static void ssd1322_cmd(uint8_t cmd)
{
    spi_transaction_t t = {
        .length = 8,
        .tx_buffer = &cmd,
        .user = (void *)0
    };

    ESP_ERROR_CHECK(
        spi_device_transmit(spi, &t)
    );
}

/************************************************************
 * Send data
 ************************************************************/
static void ssd1322_data(uint8_t data)
{
    spi_transaction_t t = {
        .length = 8,
        .tx_buffer = &data,
        .user = (void *)1
    };

    ESP_ERROR_CHECK(
        spi_device_transmit(spi, &t)
    );
}

/************************************************************
 * Reset display
 ************************************************************/
static void ssd1322_reset(void)
{
    gpio_set_level(RST, LO);
    vTaskDelay(pdMS_TO_TICKS(20));

    gpio_set_level(RST, HI);
    vTaskDelay(pdMS_TO_TICKS(100));
}

/**************************************
 * initialise SSD1322
 **************************************/
void ssd1322_init(void)
{
    ssd1322_reset();

    ssd1322_cmd(0xFD);
    ssd1322_data(0x12);

    ssd1322_cmd(0xB3);
    ssd1322_data(0xD0);

    ssd1322_cmd(0xCA);
    ssd1322_data(0x3F);

    ssd1322_cmd(0xA2);
    ssd1322_data(0x00);

    ssd1322_cmd(0xA1);
    ssd1322_data(0x00);

    ssd1322_cmd(0xA0);
    ssd1322_data(0x14);
    ssd1322_data(0x11);

    ssd1322_cmd(0xB5);
    ssd1322_data(0x00);

    ssd1322_cmd(0xAB);
    ssd1322_data(0x01);

    ssd1322_cmd(0xB4);
    ssd1322_data(0xA0);
    ssd1322_data(0xB5);

    ssd1322_cmd(0xC1);
    ssd1322_data(0x7F);

    ssd1322_cmd(0xC7);
    ssd1322_data(0x0F);

    ssd1322_cmd(0xB9);

    ssd1322_cmd(0xB1);
    ssd1322_data(0xE2);

    ssd1322_cmd(0xD1);
    ssd1322_data(0xA2);
    ssd1322_data(0x20);

    ssd1322_cmd(0xBB);
    ssd1322_data(0x1F);

    ssd1322_cmd(0xB6);
    ssd1322_data(0x08);

    ssd1322_cmd(0xBE);
    ssd1322_data(0x07);

    ssd1322_cmd(0xA6);

    ssd1322_cmd(0xA9);

    ssd1322_cmd(0xAF);

    vTaskDelay(pdMS_TO_TICKS(10));
}

// Address functions
void set_column_address(uint8_t start, uint8_t end)
{
    ssd1322_cmd(0x15);
    ssd1322_data(start);
    ssd1322_data(end);
}

void set_row_address(uint8_t start, uint8_t end)
{
    ssd1322_cmd(0x75);
    ssd1322_data(start);
    ssd1322_data(end);
}

void set_write_RAM(void)
{
    ssd1322_cmd(0x5C);
}

// clear display
void clearDisplay(void)
{
    ssd1322_cmd(0xA4);

    set_column_address(0x00, 0x77);
    set_row_address(0x00, 0x3F);
    set_write_RAM();

    for(int i = 0; i < 64 * 120; i++)
    {
        ssd1322_data(0x00);
        ssd1322_data(0x00);
    }

    ssd1322_cmd(0xA6);
}



/* 16 x 8 glyphs from 0 to 9*/

const uint8_t cero[16] = {
    0x00, 0x18, 0x24, 0x42, 0x42, 0x42, 0x42, 0x42,
    0x42, 0x42, 0x42, 0x42, 0x42, 0x24, 0x18, 0x00
};
const uint8_t uno[16] = {
  0x00, 0x18, 0x38, 0x58, 0x18, 0x18, 0x18, 0x18,
  0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x7E, 0x00
};

const uint8_t dos[16] = {
  0x00, 0x1C, 0x22, 0x42, 0x42, 0x06, 0x08, 0x10,
  0x20, 0x40, 0x40, 0x40, 0x40, 0x3E, 0x00, 0x00
};

const uint8_t treis[16] = {
  0x00, 0x38, 0x44, 0x02, 0x02, 0x02, 0x02, 0x3E,
  0x02, 0x02, 0x02, 0x02, 0x02, 0x44, 0x38, 0x00
};

const uint8_t cuatro[16] = {
  0x00, 0x04, 0x0C, 0x14, 0x24, 0x44, 0x44, 0x44,
  0x3E, 0x04, 0x04, 0x04, 0x04, 0x04, 0x0E, 0x00
};

const uint8_t cinco[16] = {
  0x00, 0x3E, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40,
  0x3C, 0x02, 0x02, 0x02, 0x02, 0x02, 0x7C, 0x00
};

const uint8_t seis[16] = {
  0x00, 0x3C, 0x42, 0x40, 0x40, 0x40, 0x40, 0x7C,
  0x42, 0x42, 0x42, 0x42, 0x42, 0x3C, 0x00, 0x00
};

const uint8_t siete[16] = {
  0x00, 0x3C, 0x42, 0x02, 0x02, 0x02, 0x02, 0x04,
  0x04, 0x08, 0x08, 0x10, 0x10, 0x20, 0x20, 0x00
};

const uint8_t ocho[16] = {
  0x00, 0x18, 0x24, 0x42, 0x42, 0x42, 0x42, 0x3E,
  0x42, 0x42, 0x42, 0x42, 0x42, 0x26, 0x18, 0x00
};

const uint8_t nueve[16] = {
  0x00, 0x1C, 0x22, 0x42, 0x42, 0x42, 0x42, 0x22,
  0x1C, 0x02, 0x02, 0x02, 0x02, 0x04, 0x38, 0x00
};

/*How the SSD1322 takes pixel data: each pixel is 4 bits of grey (0x0 to 0xF), 
and one column address covers 4 pixels, which is 2 bytes. An 8-pixel-wide glyph 
therefore takes 2 column addresses, or 4 bytes per row, and 64 bytes for 16 rows.*/

 #define COL_OFFSET 0x1C   // use the same offset your clear screen uses

 void draw_glyph(uint8_t x, uint8_t y, const uint8_t *glyph, uint8_t fg, uint8_t bg)
{
/* Function parameters:

uint8_t x: the horizontal position on the screen, in pixels, of the glyph's left edge. 0 is the left side. 
It should be a multiple of 4 (0, 4, 8, 12...), because the display writes columns in groups of 4 pixels.

uint8_t y: the vertical position, in pixels, of the glyph's top edge. 0 is the top row. 
The glyph occupies rows y to y + 15, since it is 16 rows tall.

const uint8_t *glyph: a pointer to the glyph's 16 bytes, one byte per row. 
When you write glyph_4, you are passing the address of that array. const means the function promises not to change it, 
and uint8_t * means "pointer to 8-bit unsigned values". 
Inside the function, glyph[0] is the top row and glyph[15] is the bottom row.

uint8_t fg: the foreground grey level, used for the pixels that are on (the shape of the digit). 
It can be 0x0 (black) to 0xF (white).

uint8_t bg: the background grey level, used for the pixels that are off (the space around the digit). 
It has the same range, 0x0 to 0xF.*/

int row;
    int col;
    uint8_t bits;
    uint8_t mask;
    uint8_t pixel1;
    uint8_t pixel2;
    uint8_t out;

    // tell the display where to draw (2 column addresses = 8 pixels wide)
    set_column_address(COL_OFFSET + x / 4, COL_OFFSET + x / 4 + 1);
    set_row_address(y, y + 15);
    set_write_RAM();

    for (row = 0; row < 16; row++)
    {
        bits = glyph[row];      // one row of the glyph (8 pixels)
        mask = 0x80;            // start with the leftmost pixel

        for (col = 0; col < 4; col++)
        {
            // first pixel of the pair
            if (bits & mask)
                pixel1 = fg;
            else
                pixel1 = bg;
            mask = mask / 2;    // move to the next pixel

            // second pixel of the pair
            if (bits & mask)
                pixel2 = fg;
            else
                pixel2 = bg;
            mask = mask / 2;

            // pack two 4-bit pixels into one byte and send it
            out = pixel1 * 16 + pixel2;
            ssd1322_data(out);
        }
    }
}

#define glyph_x 8
#define glyph_y 16

uint8_t result[256];
uint32_t bytes_read;
float raw;

static const char *TAG = "ADC result";/*The static keyword limits the 
                                    symbol’s visibility to the current file, 
                                    while const ensures the string is stored 
                                    in read-only memory instead of consuming RAM.*/

static adc_continuous_handle_t handle = NULL;  

    

void adc_create_handle(void) /*create an ADC continuous mode driver handle, 
                                set up the required configuration structure 
                                adc_continuous_handle_cfg_t:*/
{
  static  adc_continuous_handle_cfg_t adc_config = 
    {
        .max_store_buf_size = 8 * 32 * 4,/* = 1024 - When initializing the continuous mode driver via 
                                        adc_continuous_new_handle(), your conv_frame_size
                                         parameter inside adc_continuous_handle_cfg_t must be 
                                         a multiple of 4 bytes 
                                         (SOC_ADC_DIGI_DATA_BYTES_PER_CONV). 
                                         This defines the number of conversion samples 
                                         processed before triggering a DMA interrupt.*/
        .conv_frame_size = 256,/*Stores 128 conversion results (256 bytes / 2 bytes per result)
                                 before triggering a completion interrupt. 
                                This is a multiple of 4 and matches Espressif's default 
                                recommendation*/
        .flags = {
        .flush_pool = 0,    /*when the buffer fills, older buffered results are discarded 
                            so newer results can be stored.*/
    }
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &handle));
}

void adc_configure(void) //a pointer member means I need a separate object and pass its address (adc pattern)
{
   static adc_digi_pattern_config_t pattern = {
    .atten = ADC_ATTEN_DB_0, // no attenuation
    .channel = ADC_CHANNEL_6, //INPUT ONLY ADC1_CH6, GPIO 34
    .unit = ADC_UNIT_1,//ADC 1
    .bit_width = ADC_BITWIDTH_12 //ADC output width is 12 bit

};

static adc_continuous_config_t dig_cfg = {
    .sample_freq_hz = 20000,
    .conv_mode = ADC_CONV_SINGLE_UNIT_1, // ADC 1
    .pattern_num = 1, //Number of ADC channels that will be used
    .adc_pattern = &pattern, // pointer to the above structure
    .format = ADC_DIGI_OUTPUT_FORMAT_TYPE1
};

    ESP_ERROR_CHECK(adc_continuous_config(handle, &dig_cfg));
}
uint8_t horiz, vert = 0;

/*function which accepts a number to write to ssd1322 display*/
void write_num(uint8_t num, uint8_t horiz, uint8_t vert)
{
    switch (num)
    {
        case 0:
        draw_glyph(horiz,vert, cero,0x01,0x00);

        break;

        case 1:
        draw_glyph(horiz,vert, uno,0x01,0x00);
        break;

        case 2:
        draw_glyph(horiz,vert, dos,0x01,0x00);
        break;

        case 3:
        draw_glyph(horiz,vert, treis,0x01,0x00);
        break;

        case 4:
        draw_glyph(horiz,vert, cuatro,0x01,0x00);
        break;

        case 5:
        draw_glyph(horiz,vert, cinco,0x01,0x00);
        break;

        case 6:
        draw_glyph(horiz,vert, seis,0x01,0x00);
        break;

        case 7:
        draw_glyph(horiz,vert, siete,0x01,0x00);
        break;

        case 8:
        draw_glyph(horiz,vert, ocho,0x01,0x00);
        break;

        case 9:
        draw_glyph(horiz,vert, nueve,0x01,0x00);
        break;

        default:
        break;
    }
}

    
void app_main(void)
{
    /*local variables to split temp into units, tens and hundreds*/
    uint8_t units, tens, hundreds = 0;
    adc_create_handle();
    adc_configure();

    adc_continuous_start(handle);/*Calling adc_continuous_start(handle) makes the ADC start to measure 
                                analog signals from the configured ADC channels, and generate the conversion results*/
    
    config_gpio();
    spi_bus_init();
    spi_interface_conf();
    ssd1322_init();

    clearDisplay();
   //glyph is x 8, y 16

   while (1)
   {

    esp_err_t ret = adc_continuous_read(
        handle,
        result,
        sizeof(result),
        &bytes_read,
        1000);

 if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "Bytes read = %" PRIu32, bytes_read);

        adc_digi_output_data_t *adc_data =
            (adc_digi_output_data_t *)result;

        raw = adc_data[0].type1.data;

        float voltage = raw * 1.0f / 4095.0f;
        float temp = voltage * 100.0f;

        ESP_LOGI(TAG,
                 "Raw = %.0f Voltage = %.3f V Temp = %.1f C", raw, voltage, temp);
                 //temp = temp * 10;
                 tens = (uint8_t)temp/10;
                 units = (uint8_t)temp %10;
                 ESP_LOGI(TAG, "tens = %d, units = %d\n",  tens, units);
                 write_num(tens,116,32);
                 write_num(units,128,32);
    }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
