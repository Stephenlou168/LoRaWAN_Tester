#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "esp_timer.h"
#include "esp_err.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "lvgl.h"
#include "lv_examples.h"
#include "esp_lcd_ili9341.h"

#include "esp_log.h"
#include "esp_event.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "nvs_flash.h"

#include "ttn.h"
#include "../components/ttn-esp32/src/lmic/lmic.h"         
#include "nmea_parser.h"

static const char *BTN = "Button";
static const char *GNSS = "GNSS";
static const char *Display = "Display";

// // EUI Key for synchronize data to AWS1
// const char *appEui = "dca632fffe7bc6e5";
// const char *devEui = "3a614d3bd634bd38";
// const char *appKey = "eb9c00e0bf36b742feae322fbe8ac06c";

// EUI Key for synchronize data to AWS2
const char *appEui = "dca632fffe7bc6e5";
const char *devEui = "a68acfc1d6f217d2";
const char *appKey = "770629e019d2fff79ca62c0ede3e22d3";

// Pins and other Resources for LoRa
#define TTN_SPI_HOST      SPI3_HOST
#define TTN_SPI_DMA_CHAN  SPI_DMA_CH_AUTO
#define TTN_PIN_SPI_SCLK  5
#define TTN_PIN_SPI_MOSI  27
#define TTN_PIN_SPI_MISO  19
#define TTN_PIN_NSS       18
#define TTN_PIN_RXTX      TTN_NOT_CONNECTED
#define TTN_PIN_RST       14
#define TTN_PIN_DIO0      26
#define TTN_PIN_DIO1      35

// Pin and other Resources for ST7735s
#define LCD_HOST  SPI2_HOST
#define EXAMPLE_LCD_PIXEL_CLOCK_HZ     (40 * 1000 * 1000)
#define PIN_NUM_SCLK           13 
#define PIN_NUM_MOSI           15
#define PIN_NUM_LCD_DC         2
#define PIN_NUM_LCD_RST        4
#define PIN_NUM_LCD_CS         14
#define Horizontal_Resolution          128
#define Vertical_Resolution            160
#define Buf_Display_Size ((Horizontal_Resolution * Vertical_Resolution)/ 10)

// Button Confure for Sending Message
#define BUTTON_Send (GPIO_NUM_33)
#define BUTTON_RESET_Count (GPIO_NUM_25)
#define DEBOUCNE_DELAY_TICK (1)
#define DEBOUNCE_COUNT_FLIP (5)
#define DEBOUNCE_COUNT_MAX (2 * DEBOUNCE_COUNT_FLIP)

// RTC Define for GNSS and LoRa
#define TX_INTERVAL 5       // default 5
#define TIME_ZONE (+7)
#define YEAR_BASE (2000)
#define EARTH_RADIUS 6371

// Bit Representation for ST7735s
#define EXAMPLE_LCD_CMD_BITS           8
#define EXAMPLE_LCD_PARAM_BITS         8
#define EXAMPLE_LVGL_TICK_PERIOD_MS    3        // Default 2        // 3 is okay

// #define LED (GPIO_NUM_21)
#define LED (GPIO_NUM_12)
#define LED_Receive (GPIO_NUM_32)

// Global Variable for ST7735s
static lv_disp_draw_buf_t disp_buf; 
static lv_disp_drv_t disp_drv;      
static lv_obj_t *scr;

//Important variable for Harversine
double a, c, distance;
gps_t *gps = NULL;

// RTOS Handle_t
QueueHandle_t send_msg_queue;
QueueHandle_t display_msg_queue;
SemaphoreHandle_t semp_btn;

///////////////////////////////////////////////////////////////
/* This function should be called whenever you are ready to flush the display */
static bool notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
    lv_disp_drv_t *disp_driver = (lv_disp_drv_t *)user_ctx;
    lv_disp_flush_ready(disp_driver);
    return false;
}
/*It's part of the display driver configuration and is called by LVGL whenever it needs to update the display contents.*/
static void lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t) drv->user_data;
    int offsetx1 = area->x1;
    int offsetx2 = area->x2;
    int offsety1 = area->y1;
    int offsety2 = area->y2;
    // copy a buffer's content to a specific area of the display
    esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, color_map);
}
/* Rotate display and touch, when rotated screen in LVGL. Called when driver parameters are updated. */
static void lvgl_port_update_callback(lv_disp_drv_t *drv)
{
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t) drv->user_data;

    switch (drv->rotated)
    {
        case LV_DISP_ROT_NONE:
            // Rotate LCD display
            esp_lcd_panel_swap_xy(panel_handle, true);
            esp_lcd_panel_mirror(panel_handle, false, false);
            break;
        case LV_DISP_ROT_90:
            // Rotate LCD display
            esp_lcd_panel_swap_xy(panel_handle, true);
            esp_lcd_panel_mirror(panel_handle, true, true);
            break;
        case LV_DISP_ROT_180:
            // Rotate LCD display
            esp_lcd_panel_swap_xy(panel_handle, false);
            esp_lcd_panel_mirror(panel_handle, false, true);
            break;
        case LV_DISP_ROT_270:
            // Rotate LCD display
            esp_lcd_panel_swap_xy(panel_handle, true);
            esp_lcd_panel_mirror(panel_handle, false, false);
            break;
    }
}
static void example_increase_lvgl_tick(void *arg)
{
    /* Tell LVGL how many milliseconds has elapsed */
    lv_tick_inc(EXAMPLE_LVGL_TICK_PERIOD_MS);
}
void Buf_Allocation()
{
    // alloc draw buffers used by LVGL
    // it's recommended to choose the size of the draw buffer(s) to be at least 1/10 screen sized
    // lv_color_t *buf1 = heap_caps_malloc(Horizontal_Resolution * 20 * sizeof(lv_color_t), MALLOC_CAP_DMA);
    // assert(buf1);
    // lv_color_t *buf2 = heap_caps_malloc(Horizontal_Resolution * 20 * sizeof(lv_color_t), MALLOC_CAP_DMA);
    // assert(buf2);
    // // initialize LVGL draw buffers
    // lv_disp_draw_buf_init(&disp_buf, buf1, buf2, Horizontal_Resolution * 20);

    lv_color_t *buf1 = heap_caps_malloc(Buf_Display_Size * 7 * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf1);
    lv_color_t *buf2 = heap_caps_malloc(Buf_Display_Size * 7 * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf2);
    // initialize LVGL draw buffers
    lv_disp_draw_buf_init(&disp_buf, buf1, buf2, Buf_Display_Size * 7);


    ESP_LOGI(Display, "Install LVGL tick timer");
    const esp_timer_create_args_t lvgl_tick_timer_args =
    {
        .callback = &example_increase_lvgl_tick,
        .name = "lvgl_tick"
    };
    
    esp_timer_handle_t lvgl_tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, EXAMPLE_LVGL_TICK_PERIOD_MS * 1000));
}
void Init_LCD_panel()
{
    ESP_LOGI(Display, "Install panel IO");
    esp_lcd_panel_io_handle_t io_handle = NULL;

    esp_lcd_panel_io_spi_config_t io_config =
    {
        .dc_gpio_num = PIN_NUM_LCD_DC,
        .cs_gpio_num = PIN_NUM_LCD_CS,
        .pclk_hz = EXAMPLE_LCD_PIXEL_CLOCK_HZ,
        .lcd_cmd_bits = EXAMPLE_LCD_CMD_BITS,
        .lcd_param_bits = EXAMPLE_LCD_PARAM_BITS,
        .spi_mode = 0,
        .trans_queue_depth = 10,
        .on_color_trans_done = notify_lvgl_flush_ready,
        .user_ctx = &disp_drv,
    };
    // Attach the LCD to the SPI bus
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config, &io_handle));

    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_panel_dev_config_t panel_config =
    {
        .reset_gpio_num = PIN_NUM_LCD_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR,
        .bits_per_pixel = 16,
    };
    ESP_LOGI(Display, "Install ILI9341 panel driver");
    ESP_ERROR_CHECK(esp_lcd_new_panel_ili9341(io_handle, &panel_config, &panel_handle));

    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, true, true));            // To mirror the screen according to our preference

    // user can flush pre-defined pattern to the screen before we turn on the screen or backlight
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));             // On: To turn the preview of the display, else false.

    ESP_LOGI(Display, "Register display driver to LVGL");
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = Horizontal_Resolution;
    disp_drv.ver_res = Vertical_Resolution;
    disp_drv.flush_cb = lvgl_flush_cb;
    disp_drv.drv_update_cb = lvgl_port_update_callback;
    disp_drv.draw_buf = &disp_buf;
    disp_drv.user_data = panel_handle;
}
////////////////////////////////////////////////

int count_button = -1;
enum eButtonState
{
    BUTTON_PRESSED,
    BUTTON_RELEASED,
    BUTTON_STATE_INVALID = -1
};

int ping_pong_debounce (bool button_state_1, bool button_state_2)
{
    static bool last_button_state_1 = false;
    static bool last_button_state_2 = false;

    int action = BUTTON_STATE_INVALID;

    if (button_state_1 != last_button_state_1 && button_state_1 == true)
    {
        count_button++;
        action = BUTTON_PRESSED;
    }
    else if (button_state_2 != last_button_state_2 && button_state_2 == true)
    {
        count_button=0;
        action = BUTTON_PRESSED;
    }

    last_button_state_1 = button_state_1;
    last_button_state_2 = button_state_2;

    return action;
}


void Button_MSG(void)
{
    gpio_config_t io_config = {0};
    io_config.mode = GPIO_MODE_INPUT;
    io_config.pin_bit_mask = (1ULL << BUTTON_Send) | (1ULL << BUTTON_RESET_Count);
    io_config.pull_up_en = GPIO_PULLUP_ENABLE;

    gpio_config(&io_config);

    bool button_state_1;
    bool button_state_2;
    int debounce_state;
    while (1)
    {
        button_state_1 = gpio_get_level(BUTTON_Send);
        button_state_2 = gpio_get_level(BUTTON_RESET_Count);
        debounce_state = ping_pong_debounce(button_state_1, button_state_2);

        if(debounce_state == BUTTON_PRESSED)
        {
            if(button_state_1)
            {
                ESP_LOGI(BTN, "Sending Count: %d", count_button);
                xSemaphoreGive(semp_btn);
            }
            else if(button_state_2)
            {
                ESP_LOGI(BTN, "Reset Count: %d", count_button);
            }
        }
        vTaskDelay(DEBOUCNE_DELAY_TICK);
    }
}

float hav(double lat1, double lon1, double lat2, double lon2)
{
    // Convert the Geographic Location from degree to Radian.
    lon1 = (lon1 * M_PI) / 180;
    lon2 = (lon2 * M_PI) / 180;
    lat1 = (lat1 * M_PI) / 180;
    lat2 = (lat2 * M_PI) / 180;

    // Geographical Location Differences
    float diff_lon = lon2 - lon1;
    float diff_lat = lat2 - lat1;

    float sin2_lat = sin(diff_lat/2) * sin(diff_lat/2);
    float sin2_lon = sin(diff_lon/2) * sin(diff_lon/2);

    a = sin2_lat + cos(lat1)*cos(lat2) * sin2_lon;

    c = 2*atan2(sqrt(a), sqrt(1-a));

    distance = c * EARTH_RADIUS;

    return distance;
}

static void gps_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    switch (event_id)
    {
        case GPS_UPDATE:
            gps = (gps_t *)event_data;

            float longitude = gps->longitude;
            float latitude = gps->latitude;
            distance = hav(11.571006, 104.897023,gps->latitude, gps->longitude);

            int snr_data = LMIC.snr;
            int n_sat = gps->sats_in_use;

            float *coordinate = malloc(5 * sizeof(float_t));

            coordinate[0] = latitude;
            coordinate[1] = longitude;
            coordinate[2] = distance;
            coordinate[3] = (float)snr_data;
            coordinate[4] = (float)n_sat;

            if(coordinate == NULL)
            {
                ESP_LOGE(GNSS, "Failed to allocate memory for coordinate");
                break;
            }

            xQueueSend(send_msg_queue, &coordinate, portMAX_DELAY);
            xQueueSend(display_msg_queue, &coordinate, portMAX_DELAY);

            ESP_LOGI(GNSS, " latitude = %.5f °N \r\n\t\t longitude = %.5f °E \r\n\t\t Distance = %.3f km \r\n\t\t Satellite In Use = %d satellite \r\n\t\t RSSI = %d \r\n\t\t SNR = %d \r\n\t\t SNR LoRa = %d" ,
                                                                    coordinate[0],
                                                                    coordinate[1],
                                                                    coordinate[2],
                                                                    gps->sats_in_use,
                                                                    ttn_rssi(),
                                                                    gps->sats_desc_in_view->snr,
                                                                    LMIC.snr);
            break;
        case GPS_UNKNOWN:
            /* print unknown statements */
            ESP_LOGW(GNSS, "Unknown statement:%s", (char *)event_data);
            break;
        default:
            break;
    }
}

void sendMessages(void* pvParameter)
{
    char buf[1028];
    float *coordinate;

    bool led_state = gpio_get_level(LED);

    // Initialize NMEA parser and configure GPS event handler
    nmea_parser_config_t config = NMEA_PARSER_CONFIG_DEFAULT();
    nmea_parser_handle_t nmea_hdl = nmea_parser_init(&config);
    nmea_parser_add_handler(nmea_hdl, gps_event_handler, NULL);

    vTaskDelay(1000 / portTICK_PERIOD_MS);

    while (1)
    {
        xSemaphoreTake(semp_btn, portMAX_DELAY);
        printf("Sending message...\n");

        // Check for GPS data availability or any other conditions required for sending
        if (gps == NULL)
        {
            vTaskDelay(pdMS_TO_TICKS(5000));
            continue;
        }

        // Check for queue data availability
        if(xQueueReceive(send_msg_queue, &coordinate, portMAX_DELAY))
        {
            float latitude2 = coordinate[0];
            float longitude2 = coordinate[1];
            float distance2 = coordinate[2];

            // snprintf(buf, sizeof(buf), "{\"lat\":%.5f,\n \"long\":%.5f,\n \"Dis\":%.3f\n}", latitude2, longitude2, distance2);

            char gps_data[128];
            snprintf(gps_data, sizeof(gps_data), "\"lat\":%.5f,\"long\":%.5f", latitude2, longitude2);
            char additional_data[128];
            snprintf(additional_data, sizeof(additional_data), "\"M\":%d,\"dis\":%.3f", count_button, distance2);
            snprintf(buf, sizeof(buf), "{%s,%s}", gps_data, additional_data);

            ttn_response_code_t res = ttn_transmit_message((const uint8_t*)buf, strlen(buf), 1, false);

            printf(res == TTN_SUCCESSFUL_TRANSMISSION ? "Message sent Successful.\n" : "Transmission failed.\n");  

            // Apply it when outdoor test
            gpio_set_level(LED, !led_state);
            vTaskDelay(pdMS_TO_TICKS(500));
            gpio_set_level(LED, led_state);

            // nmea_parser_remove_handler(nmea_hdl, gps_event_handler);    
            // nmea_parser_deinit(nmea_hdl);    
        }
        vTaskDelay(TX_INTERVAL * pdMS_TO_TICKS(1000)); // Delay before next transmission        
    }
}

void messageReceived(const uint8_t* message, size_t length, ttn_port_t port)
{
    bool led_state_receive = gpio_get_level(LED_Receive);
    printf("Message of %d bytes received on port %d:\n", length, port);
    for (int i = 0; i < length; i++)
    {
        // printf(" %02x\n", message[i]);
        // printf("\n");
        printf("%c", message[i]);
    }
    printf("\n");
    printf("RSSI: %d dBm\n", ttn_rssi());   ///
    printf("SNR: %d dB\n", LMIC.snr);  

    // Apply it when outdoor test
    gpio_set_level(LED_Receive, !led_state_receive);
    vTaskDelay(pdMS_TO_TICKS(500));
    gpio_set_level(LED_Receive, led_state_receive);
}

void Display_LVGL(void * param)
{
    float *receiver_coord;

    lv_obj_t *title_container = lv_obj_create(scr);
    lv_obj_remove_style_all(title_container);
    lv_obj_set_width(title_container, 126);
    lv_obj_set_height(title_container, 20);
    lv_obj_set_style_text_color(title_container, lv_color_hex(0xFFFF00), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(title_container, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(title_container, lv_color_hex(0x00FFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(title_container, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_outline_color(title_container, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_outline_opa(title_container, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_outline_width(title_container, 1, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_outline_pad(title_container, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_align(title_container, LV_ALIGN_TOP_LEFT, 1, 0);

    lv_obj_t *container_Dis = lv_obj_create(scr);
    lv_obj_remove_style_all(container_Dis);
    lv_obj_set_width(container_Dis, 66);
    lv_obj_set_height(container_Dis, 55);
    lv_obj_align(container_Dis, LV_ALIGN_CENTER, -32, 9);
    lv_obj_set_style_border_color(container_Dis, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(container_Dis, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(container_Dis, 1, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_t *container_RSSI_Sec = lv_obj_create(scr);
    lv_obj_remove_style_all(container_RSSI_Sec);
    lv_obj_set_width(container_RSSI_Sec, 65);
    lv_obj_set_height(container_RSSI_Sec, 55);
    lv_obj_align(container_RSSI_Sec, LV_ALIGN_CENTER, 32, 9);
    lv_obj_set_style_border_color(container_RSSI_Sec, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(container_RSSI_Sec, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(container_RSSI_Sec, 1, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_t *status = lv_label_create(scr);
    char data_joined[30] = "Joined";

    char text_status[50];
    snprintf(text_status, sizeof(text_status), "status: %s", data_joined);
    lv_label_set_text(status, text_status);
    lv_obj_align(status, LV_ALIGN_TOP_MID, 0,2);

    ESP_LOGI(Display, "Geographical Location");

    lv_obj_t *data_longitude= lv_label_create(scr);
    lv_obj_align(data_longitude, LV_ALIGN_TOP_LEFT, 5, 22);

    lv_obj_t *data_latitude = lv_label_create(scr);
    lv_obj_align(data_latitude, LV_ALIGN_TOP_LEFT,20 ,41);

    ESP_LOGI(Display, "Distance");
    lv_obj_t *Distance_label = lv_label_create(scr);
    lv_label_set_text(Distance_label, "Distance");
    lv_obj_align(Distance_label, LV_ALIGN_LEFT_MID, 0 , -8);

    lv_obj_t *Dis_data = lv_label_create(scr);
    lv_obj_align(Dis_data, LV_ALIGN_LEFT_MID,13,10);

    lv_obj_t *Dis_SI = lv_label_create(scr);
    lv_label_set_text(Dis_SI, "KM");
    lv_obj_align(Dis_SI, LV_ALIGN_CENTER, -35, 28);

    ESP_LOGI(Display, "RSSI");

    lv_obj_t *rssi = lv_label_create(scr);
    lv_label_set_text(rssi, "RSSI");
    lv_obj_align(rssi, LV_ALIGN_LEFT_MID, 80,-8);

    lv_obj_t *rssi_data = lv_label_create(scr);
    lv_obj_align(rssi_data, LV_ALIGN_LEFT_MID, 85,10);

    lv_obj_t *rssi_SI = lv_label_create(scr);
    lv_label_set_text(rssi_SI, "dBm");
    lv_obj_align(rssi_SI, LV_ALIGN_CENTER, 30, 28);

    ESP_LOGI(Display, "SNR");

    lv_obj_t *snr= lv_label_create(scr);
    lv_obj_align(snr, LV_ALIGN_BOTTOM_MID, 0, -25);

    ESP_LOGI(Display, "Downlink");

    lv_obj_t *Messgae_ID = lv_label_create(scr);
    lv_label_set_text(Messgae_ID, "MSG ID: ");
    lv_obj_align(Messgae_ID, LV_ALIGN_BOTTOM_MID, -10 , -10);

    // lv_obj_t *n_satellite = lv_label_create(scr);
    // lv_obj_align(n_satellite, LV_ALIGN_BOTTOM_MID, 40, -10);
    
    lv_obj_t *Message_ID_Data = lv_label_create(scr);
    lv_obj_align(Message_ID_Data, LV_ALIGN_BOTTOM_MID, 40, -10);

    while (1)
    {
        if (xQueueReceive(display_msg_queue, &receiver_coord, portMAX_DELAY) == pdPASS)
        {
            float latitude2 = receiver_coord[0];
            float longitude2 = receiver_coord[1];
            float distance2 = receiver_coord[2];
            int snr_data_c = (int)receiver_coord[3];
            int n_sat_c = (int)receiver_coord[4];

            printf("lon = %.5f, lat = %.5f, dis = %.3f, snr = %.2f, n_sat = %.2f\n", receiver_coord[0], receiver_coord[1], receiver_coord[2], receiver_coord[3], receiver_coord[4]);

            char text_rssi[10];
            snprintf(text_rssi, sizeof(text_rssi), "-%d",ttn_rssi());
            lv_label_set_text(rssi_data, text_rssi);

            float data_long = longitude2;
            char text_long[30];
            snprintf(text_long, sizeof(text_long), "Long: %.5f°", data_long);
            lv_label_set_text(data_longitude, text_long);

            float data_lat = latitude2;
            char text_lat[30];
            snprintf(text_lat, sizeof(text_lat), "Lat: %.5f°",data_lat);
            lv_label_set_text(data_latitude, text_lat);

            float data_dis = distance2;
            char text_dis[30];
            snprintf(text_dis, sizeof(text_dis), "%.3f", data_dis);
            lv_label_set_text(Dis_data, text_dis);

            int snr_data = snr_data_c;
            char text_snr[30];
            snprintf(text_snr, sizeof(text_snr), "SNR: %d dB", snr_data);
            lv_label_set_text(snr, text_snr);

            // int n_sat = n_sat_c;
            // char text_sat[30];
            // snprintf(text_sat, sizeof(text_sat), "%d", n_sat);
            // lv_label_set_text(n_satellite, text_sat);

            free(receiver_coord);

            vTaskDelay(pdMS_TO_TICKS(10));
            lv_task_handler();
        }

        char button_count_char[20];
        snprintf(button_count_char, sizeof(button_count_char), "%d", count_button);
        lv_label_set_text(Message_ID_Data, button_count_char);
        vTaskDelay(pdMS_TO_TICKS(10));
        lv_task_handler();
    }
}

void app_main(void)
{
    send_msg_queue = xQueueCreate(5, sizeof(float_t));
    display_msg_queue = xQueueCreate(5, sizeof(float_t));
    semp_btn = xSemaphoreCreateBinary();

    esp_err_t err;
    err = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    ESP_ERROR_CHECK(err);
    err = nvs_flash_init();
    ESP_ERROR_CHECK(err);

    ESP_LOGI(Display, "Initializing Display SPI");
    spi_bus_config_t LCD_config = 
    {
        .sclk_io_num = PIN_NUM_SCLK,
        .mosi_io_num = PIN_NUM_MOSI,
        .quadhd_io_num = -1,
        .quadwp_io_num = -1,
        .max_transfer_sz = Horizontal_Resolution * 80 * sizeof(uint16_t),
    };
    err = spi_bus_initialize(LCD_HOST, &LCD_config, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(err);

    Init_LCD_panel();       //
    Buf_Allocation();       //
    lv_init();
    

    //Initiate SPI configurations for TTN
    spi_bus_config_t LoRa_Conf = {
        .miso_io_num = TTN_PIN_SPI_MISO,
        .mosi_io_num = TTN_PIN_SPI_MOSI,
        .sclk_io_num = TTN_PIN_SPI_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 0
    }; 
    err = spi_bus_initialize(TTN_SPI_HOST, &LoRa_Conf, TTN_SPI_DMA_CHAN);
    ESP_ERROR_CHECK(err);


    printf("Size of transactin size of SPI is %d", LoRa_Conf.max_transfer_sz);
    
    gpio_config_t led_conf = 
    {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT_OUTPUT,
        .pin_bit_mask = (1ULL << LED)
    };
    gpio_config(&led_conf);

    gpio_config_t led_conf_receive = 
    {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT_OUTPUT,
        .pin_bit_mask = (1ULL << LED_Receive)
    };
    gpio_config(&led_conf_receive);
    
    lv_disp_t * disp = lv_disp_drv_register(&disp_drv);
    scr = lv_disp_get_scr_act(disp);
    scr = lv_obj_create(NULL);
    lv_scr_load(scr);

    //Initiate TTN
    ttn_init();
    ttn_configure_pins(TTN_SPI_HOST, TTN_PIN_NSS, TTN_PIN_RXTX, TTN_PIN_RST, TTN_PIN_DIO0, TTN_PIN_DIO1);
    ttn_provision(devEui, appEui, appKey);
    ttn_on_message(messageReceived);

    printf("Joining...\n");

    if (ttn_join())
    {
        printf("Joined.\n");
        xTaskCreate(sendMessages, "send_messages",10 * 1024, (void* )0, 3, NULL);   
        xTaskCreate((TaskFunction_t)Button_MSG, "BUTTON", 4 * 1024, (void *)0, 5, NULL);
        xTaskCreate(Display_LVGL, "Display", 8 * 1024, (void *)0, 3, NULL);
    }
    else
    {
        printf("Join failed. Goodbye\n");
    }
}