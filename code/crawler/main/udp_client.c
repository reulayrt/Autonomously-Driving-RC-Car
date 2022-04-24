/* BSD Socket API Example
   This example code is in the Public Domain (or CC0 licensed, at your option.)
   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"
#include <string.h>

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>
#include "addr_from_stdin.h"

#if defined(CONFIG_EXAMPLE_IPV4)
#define HOST_IP_ADDR CONFIG_EXAMPLE_IPV4_ADDR
#elif defined(CONFIG_EXAMPLE_IPV6)
#define HOST_IP_ADDR CONFIG_EXAMPLE_IPV6_ADDR
#else
#define HOST_IP_ADDR ""
#endif

#define PORT CONFIG_EXAMPLE_PORT

static const char *TAG = "example";
static const char *payload = "Message from ESP32 ";

bool powerToggle = false;

int lidarDistance;


static void udp_client_task(void *pvParameters)
{
    char rx_buffer[128];
    char host_ip[] = HOST_IP_ADDR;
    int addr_family = 0;
    int ip_protocol = 0;

    while (1) {

#if defined(CONFIG_EXAMPLE_IPV4)
        struct sockaddr_in dest_addr;
        dest_addr.sin_addr.s_addr = inet_addr(HOST_IP_ADDR);
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(PORT);
        addr_family = AF_INET;
        ip_protocol = IPPROTO_IP;
#elif defined(CONFIG_EXAMPLE_IPV6)
        struct sockaddr_in6 dest_addr = { 0 };
        inet6_aton(HOST_IP_ADDR, &dest_addr.sin6_addr);
        dest_addr.sin6_family = AF_INET6;
        dest_addr.sin6_port = htons(PORT);
        dest_addr.sin6_scope_id = esp_netif_get_netif_impl_index(EXAMPLE_INTERFACE);
        addr_family = AF_INET6;
        ip_protocol = IPPROTO_IPV6;
#elif defined(CONFIG_EXAMPLE_SOCKET_IP_INPUT_STDIN)
        struct sockaddr_storage dest_addr = { 0 };
        ESP_ERROR_CHECK(get_addr_from_stdin(PORT, SOCK_DGRAM, &ip_protocol, &addr_family, &dest_addr));
#endif

        int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket created, sending to %s:%d", HOST_IP_ADDR, PORT);

        while (1) {

            int err = sendto(sock, payload, strlen(payload), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
            if (err < 0) {
                ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                break;
            }
            ESP_LOGI(TAG, "Message sent");

            struct sockaddr_storage source_addr; // Large enough for both IPv4 or IPv6
            socklen_t socklen = sizeof(source_addr);
            int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);

            // Error occurred during receiving
            if (len < 0) {
                ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
                break;
            }
            // Data received
            else {
                rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string
                ESP_LOGI(TAG, "Received %d bytes from %s:", len, host_ip);
                ESP_LOGI(TAG, "%s", rx_buffer);
                if (strcmp(rx_buffer, "ON") == 0) powerToggle = true;
                else powerToggle = false;
                if (strncmp(rx_buffer, "OK: ", 4) == 0) {
                    ESP_LOGI(TAG, "Received expected message, reconnecting");
                    break;
                }
            }

            vTaskDelay(2000 / portTICK_PERIOD_MS);
        }

        if (sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
    }
    vTaskDelete(NULL);
}


#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_attr.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"
#include "esp_log.h"

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/mcpwm.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"

#include "driver/i2c.h"
#include "driver/gpio.h"
#include "display.h"

// You can get these value from the datasheet of servo you use, in general pulse width varies between 1000 to 2000 mocrosecond
#define SERVO_MIN_PULSEWIDTH_US (900) // Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH_US (1900) // Maximum pulse width in microsecond
#define SERVO_MAX_DEGREE        (180)   // Maximum angle in degree upto which servo can rotate

#define STEERING_MIN_PULSEWIDTH 700  //Minimum pulse width in microsecond
#define STEERING_MAX_PULSEWIDTH 2100 //Maximum pulse width in microsecond
#define STEERING_MAX_DEGREE 180      //Maximum angle in degree upto which servo can rotate

#define DRIVE_WHEELS_GPIO       (18)   // GPIO connects to the PWM signal line
#define SERVO_PULSE_GPIO        (19)   // GPIO connects to the PWM signal line

#define CIRCUM_METERS (0.25)

// ADC
#define DEFAULT_VREF (1100)
#define NO_OF_SAMPLES (64)

// Alphanumeric Stuff //
// 14-Segment Display
#define SLAVE_ADDR                         0x70 // alphanumeric address
#define SLAVE_ADDR2                        0x62 // alphanumeric address
#define OSC                                0x21 // oscillator cmd
#define HT16K33_BLINK_DISPLAYON            0x01 // Display on cmd
#define HT16K33_BLINK_OFF                  0    // Blink off cmd
#define HT16K33_BLINK_CMD                  0x80 // Blink cmd
#define HT16K33_CMD_BRIGHTNESS             0xE0 // Brightness cmd

// Master I2C
#define I2C_EXAMPLE_MASTER_SCL_IO          22   // gpio number for i2c clk
#define I2C_EXAMPLE_MASTER_SDA_IO          23   // gpio number for i2c data
#define I2C_EXAMPLE_MASTER_NUM             I2C_NUM_0  // i2c port
#define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE  0    // i2c master no buffer needed
#define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE  0    // i2c master no buffer needed
#define I2C_EXAMPLE_MASTER_FREQ_HZ         100000     // i2c master clock freq
#define WRITE_BIT                          I2C_MASTER_WRITE // i2c master write
#define READ_BIT                           I2C_MASTER_READ  // i2c master read
#define ACK_CHECK_EN                       true // i2c master will check ack
#define ACK_CHECK_DIS                      false// i2c master will not check ack
#define ACK_VAL                            0x00 // i2c ack value
#define NACK_VAL                           0xFF // i2c nack value

//////////////////////////////////////////

static esp_adc_cal_characteristics_t *adc_chars;
static const adc_channel_t channel = ADC_CHANNEL_6; //GPIO34 if ADC1 (A2)
static const adc_atten_t atten = ADC_ATTEN_DB_0;
static const adc_unit_t unit = ADC_UNIT_1;

static const adc_channel_t channel2 = ADC_CHANNEL_3; //ADC2_GPIO26_CHANNEL;  // A0
static const adc_atten_t atten2 = ADC_ATTEN_DB_11;
static const adc_unit_t unit2 = ADC_UNIT_2;

int counter = 0;

char sensor_data[50];
float sdata_array[3];

// bool startCrawler = false;
bool startCrawler = true;

int range;

float wheelSpeed = 0.00;

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"

static const int RX_BUF_SIZE = 1024;
static const adc_bits_width_t width = ADC_WIDTH_BIT_12;


#define TXD_PIN (GPIO_NUM_4)
#define RXD_PIN (GPIO_NUM_5)

void rxtx_init(void) {
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

int sendData(const char* logName, const char* data)
{
    const int len = strlen(data);
    const int txBytes = uart_write_bytes(UART_NUM_1, data, len);
    // ESP_LOGI(logName, "Wrote %d bytes", txBytes);
    return txBytes;
}

static void tx_task(void *arg)
{
    static const char *TX_TASK_TAG = "";
    esp_log_level_set(TX_TASK_TAG, ESP_LOG_INFO);
    while (1) {
        sendData(TX_TASK_TAG, "");
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

static inline uint32_t example_convert_servo_angle_to_duty_us(int angle)
{
    return (STEERING_MIN_PULSEWIDTH + (((STEERING_MAX_PULSEWIDTH - STEERING_MIN_PULSEWIDTH) * (angle)) / (STEERING_MAX_DEGREE)));
}

static void steeringPID() {
    uint32_t count = (0.5 * SERVO_MAX_DEGREE) + 4;
    uint32_t center = (0.5 * SERVO_MAX_DEGREE) + 4;
    uint32_t angle = example_convert_servo_angle_to_duty_us(center);
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, angle);  // straighten

    int ds_complete = 0;
    int ds = 100;
    float setpoint = 25;
    float previous_errorS = 0.00;
    float previous_output = 0.00;
    float integralS = 0.00; 
    float derivativeS;
    float outputS;
    int errorS;

    while (1) {
        // mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, center);
        errorS = sdata_array[1] - setpoint;
        integralS = integralS + errorS * ds;
        derivativeS = (errorS - previous_errorS) / ds;
        outputS = (0.3 * errorS) + (0.35 * integralS) + (0.35 * derivativeS);
        printf("STEER PID OUTPUT: %f\n", outputS);
        //printf("ERRORS: %d\n", errorS);
        

        if (outputS > previous_output) { // steer right
            count -= 20;
            if (count < 67) count = 67;
            angle = example_convert_servo_angle_to_duty_us(count);
            mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, angle);

        }
        else if (outputS < previous_output) { // steer left
            count += 20;
            if (count > 97) count = 97;
            angle = example_convert_servo_angle_to_duty_us(count);
            mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, angle);
        }
        else {
            angle = example_convert_servo_angle_to_duty_us(center);
            mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, angle);
        }
        vTaskDelay(ds);
        angle = example_convert_servo_angle_to_duty_us(center);
        mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, angle);
        previous_errorS = errorS;
        previous_output = outputS;
    }
}

static void parser(char str[]) {
    int i = 0;
    char *ptr;
    ptr = strtok(str, ",");
    while (ptr && i < 3) {
        float a = atof(ptr);
        sdata_array[i] = a;
        ptr = strtok(NULL, ",");
        i++;
    }
}

static void rx_task(void *arg)
{
    static const char *RX_TASK_TAG = "RX_TASK";
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
    uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZE+1);
    while (1) {
        const int rxBytes = uart_read_bytes(UART_NUM_1, data, RX_BUF_SIZE, 500 / portTICK_RATE_MS);
        if (rxBytes > 0) {
            data[rxBytes] = 0;
            ESP_LOGI(RX_TASK_TAG, "Read %d bytes: '%s'", rxBytes, data);
            ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, rxBytes, ESP_LOG_INFO);
            sprintf(sensor_data, "%s", data);
            printf("SENSOR_DATA: %s\n", sensor_data);
            parser(sensor_data);
            printf("%f %f %f\n", sdata_array[0], sdata_array[1], sdata_array[2]);
        }
    }
    free(data);
}

// ALPHANUMERIC FUNCTIONS
// Function to initiate i2c -- note the MSB declaration!
// Function to initiate i2c -- note the MSB declaration!
static void i2c_example_master_init(){
    // Debug
    printf("\n>> i2c Config\n");
    int err;

    // Port configuration
    int i2c_master_port = I2C_EXAMPLE_MASTER_NUM;

    /// Define I2C configurations
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;                              // Master mode
    conf.sda_io_num = I2C_EXAMPLE_MASTER_SDA_IO;              // Default SDA pin
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;                  // Internal pullup
    conf.scl_io_num = I2C_EXAMPLE_MASTER_SCL_IO;              // Default SCL pin
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;                  // Internal pullup
    conf.master.clk_speed = I2C_EXAMPLE_MASTER_FREQ_HZ;       // CLK frequency
    conf.clk_flags = 0;
    err = i2c_param_config(i2c_master_port, &conf);           // Configure
    if (err == ESP_OK) {printf("- parameters: ok\n");}

    // Install I2C driver
    err = i2c_driver_install(i2c_master_port, conf.mode,
                       I2C_EXAMPLE_MASTER_RX_BUF_DISABLE,
                       I2C_EXAMPLE_MASTER_TX_BUF_DISABLE, 0);
    // i2c_set_data_mode(i2c_master_port,I2C_DATA_MODE_LSB_FIRST,I2C_DATA_MODE_LSB_FIRST);
    if (err == ESP_OK) {printf("- initialized: yes\n\n");}

    // Dat in MSB mode
    i2c_set_data_mode(i2c_master_port, I2C_DATA_MODE_MSB_FIRST, I2C_DATA_MODE_MSB_FIRST);
}

// Utility  Functions //////////////////////////////////////////////////////////

// Utility function to test for I2C device address -- not used in deploy
int testConnection(uint8_t devAddr, int32_t timeout) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (devAddr << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    int err = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 100 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return err;
}

// Utility function to scan for i2c device
static void i2c_scanner() {
    int32_t scanTimeout = 1000;
    printf("\n>> I2C scanning ..."  "\n");
    uint8_t count = 0;
    for (uint8_t i = 1; i < 127; i++) {
        // printf("0x%X%s",i,"\n");
        if (testConnection(i, scanTimeout) == ESP_OK) {
            printf( "- Device found at address: 0x%X%s", i, "\n");
            count++;
        }
    }
    if (count == 0)
        printf("- No I2C devices found!" "\n");
    printf("\n");
}

////////////////////////////////////////////////////////////////////////////////

// Alphanumeric Functions //////////////////////////////////////////////////////

// Turn on oscillator for alpha display
int alpha_oscillator() {
  int ret;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, OSC, ACK_CHECK_EN);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 100 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  vTaskDelay(200 / portTICK_RATE_MS);
  return ret;
}

// Set blink rate to off
int no_blink() {
  int ret;
  i2c_cmd_handle_t cmd2 = i2c_cmd_link_create();
  i2c_master_start(cmd2);
  i2c_master_write_byte(cmd2, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd2, HT16K33_BLINK_CMD | HT16K33_BLINK_DISPLAYON | (HT16K33_BLINK_OFF << 1), ACK_CHECK_EN);
  i2c_master_stop(cmd2);
  ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd2, 100 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd2);
  vTaskDelay(200 / portTICK_RATE_MS);
  return ret;
}

// Set Brightness
int set_brightness_max(uint8_t val) {
  int ret;
  i2c_cmd_handle_t cmd3 = i2c_cmd_link_create();
  i2c_master_start(cmd3);
  i2c_master_write_byte(cmd3, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd3, HT16K33_CMD_BRIGHTNESS | val, ACK_CHECK_EN);
  i2c_master_stop(cmd3);
  ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd3, 100 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd3);
  vTaskDelay(200 / portTICK_RATE_MS);
  return ret;
}

////////////////////////////////////////////////////////////////////////////////

static void test_alpha_display() {

    char str[100];
    sprintf(str, "%01.1f", wheelSpeed);
    printf("SPEED: %s\n", str);

    // Debug
    int ret;
    printf(">> Test Alphanumeric Display: \n");

    // Set up routines
    // Turn on alpha oscillator
    ret = alpha_oscillator();
    if(ret == ESP_OK) {printf("- oscillator: ok \n");}
    // Set display blink off
    ret = no_blink();
    if(ret == ESP_OK) {printf("- blink: off \n");}
    ret = set_brightness_max(0xF);
    if(ret == ESP_OK) {printf("- brightness: max \n");}

    // Continually writes the same command
    while (1) {
      int i;  

     sprintf(str, "%02.1f", wheelSpeed);
      uint16_t displaybuffer[8];

      for (i = 0; i < 4; i++) { // i think this is needed otherwise text wouldnt change
          displaybuffer[i] = alphafonttable[' '];
      }

      i = 0;
      while (i < 4 && str[i] != '\0') { // go until 4 chars or end of string
        displaybuffer[i] = alphafonttable[(int) str[i]];
        i++;
      }


      // Send commands characters to display over I2C
      i2c_cmd_handle_t cmd4 = i2c_cmd_link_create();
      i2c_master_start(cmd4);
      i2c_master_write_byte(cmd4, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
      i2c_master_write_byte(cmd4, (uint8_t)0x00, ACK_CHECK_EN);
      for (uint8_t i=0; i<8; i++) {
        i2c_master_write_byte(cmd4, displaybuffer[i] & 0xFF, ACK_CHECK_EN);
        i2c_master_write_byte(cmd4, displaybuffer[i] >> 8, ACK_CHECK_EN);
      }
      i2c_master_stop(cmd4);
      ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd4, 100 / portTICK_RATE_MS);
      i2c_cmd_link_delete(cmd4);

    //   for (int i = 0; i < 8; i++) {
    //       printf("%04x\n", displaybuffer[i]);
    //   }

      if(ret == ESP_OK) {
      }
    }


}



static void check_efuse(void)
{
    //Check TP is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK)
    {
        printf("eFuse Two Point: Supported\n");
    }
    else
    {
        printf("eFuse Two Point: NOT supported\n");
    }

    //Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK)
    {
        printf("eFuse Vref: Supported\n");
    }
    else
    {
        printf("eFuse Vref: NOT supported\n");
    }
}

static void print_char_val_type(esp_adc_cal_value_t val_type)
{
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP)
    {
        printf("Characterized using Two Point Value\n");
    }
    else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF)
    {
        printf("Characterized using eFuse Vref\n");
    }
    else
    {
        printf("Characterized using Default Vref\n");
    }
}

static void calibrateESC()
{
    printf("CALIBRATING...\n");
    vTaskDelay(3000 / portTICK_PERIOD_MS);  // Give yourself time to turn on crawler (3s)
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1200); // NEUTRAL signal in microseconds
    vTaskDelay(3100 / portTICK_PERIOD_MS); // Do for at least 3s, and leave in neutral state
    printf("CALIBRATION COMPLETE.\n");
}

static void pwm_gpio_init() {
    printf("PWM GPIO INIT\n");
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, DRIVE_WHEELS_GPIO);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, SERVO_PULSE_GPIO);
    printf("PWM GPIO INIT COMPLETE\n");
}

static void pwm_init() {
    printf("PWM INIT\n");
    pwm_gpio_init();

    mcpwm_config_t pwm_config;
    pwm_config.frequency = 50; //frequency = 50Hz, i.e. for every servo motor time period should be 20ms
    pwm_config.cmpr_a = 0;     //duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;     //duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
    printf("PWM INIT COMPLETE\n");
}

static void optical_sensor(void) {
    counter = 0;

    bool hasPulsed = false;

    while(1) {
        uint32_t adc_reading = 0;

        // multisampling
        for (int i = 0; i < NO_OF_SAMPLES; i++) {
            if (unit == ADC_UNIT_1)
                adc_reading += adc1_get_raw((adc1_channel_t) channel);
            else {
                int raw;
                adc2_get_raw((adc2_channel_t) channel, ADC_WIDTH_BIT_12, &raw);
                adc_reading += raw;
            }
        }

        adc_reading /= NO_OF_SAMPLES;

        if (adc_reading < 4095 && hasPulsed == false) {
            counter += 1;
            hasPulsed = true;
        }
        else if (adc_reading == 4095)
            hasPulsed = false;

        // vTaskDelay(10 / portTICK_RATE_MS);
    }
}

static void calculate_speed(void) {
    while (1) {
        counter = 0;
        vTaskDelay(100);
        printf("Counter: %d\n", counter);
        wheelSpeed = (counter*1.0 / 12.0) * (CIRCUM_METERS);
        // printf("wheelspeed: %f \n", wheelSpeed);   
        counter = 0;
    }
}

#include "./ADXL343.h"
// ADXL343 Functions ///////////////////////////////////////////////////////////
// Get Device ID
int getDeviceID(uint8_t *data)
{
  int ret;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (SLAVE_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, ADXL343_REG_DEVID, ACK_CHECK_EN);
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (SLAVE_ADDR << 1) | READ_BIT, ACK_CHECK_EN);
  i2c_master_read_byte(cmd, data, ACK_CHECK_DIS);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 100 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  return ret;
}

// Write one byte to register
void writeRegister(uint8_t reg, uint8_t data)
{
  // int ret;
  // printf("--Writing %d to reg %d!--\n", data, reg);
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  //start command
  i2c_master_start(cmd);
  //slave address followed by write bit
  i2c_master_write_byte(cmd, (SLAVE_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
  //register pointer sent
  i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
  //data sent
  i2c_master_write_byte(cmd, data, ACK_CHECK_EN);
  //stop command
  i2c_master_stop(cmd);
  i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 100 / portTICK_RATE_MS);
  // if (ret == ESP_OK)
  // {
  //   printf("I2C SUCCESSFUL \n");
  // }
  i2c_cmd_link_delete(cmd);
}

// Read register
uint8_t readRegister(uint8_t reg)
{
  uint8_t value;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  //start command
  i2c_master_start(cmd);
  //slave followed by write bit
  i2c_master_write_byte(cmd, (SLAVE_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
  //register pointer sent
  i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
  //stop command
  // i2c_master_stop(cmd);

  //repeated start command
  i2c_master_start(cmd);
  //slave followed by read bit
  i2c_master_write_byte(cmd, (SLAVE_ADDR << 1) | READ_BIT, ACK_CHECK_EN);
  //place data from register into bus
  i2c_master_read_byte(cmd, &value, ACK_CHECK_DIS);
  //stop command
  i2c_master_stop(cmd);
  i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 100 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  return value;
}

static void speedPID() {
    int count;

    for (count = 1200; count < 1376; count += 2) {
        mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, count);
        vTaskDelay(100 / portTICK_RATE_MS);
    }

  int ds_complete = 0;
  int ds = 100;
  float setpointS = 0.25;
  float previous_errorS = 0.00; // Set up PID loop
  float integralS = 0.00;
  float derivativeS;
  float outputS;
  int errorS;
  int lidarCount = 0;

  while(1) {
    if ( (lidarDistance < 120 && lidarDistance > 1) || !powerToggle) {
        lidarCount++;
        printf("SLOW DOWN STARTED !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
        // if (lidarCount > 2) {
            mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 0);
            break;
        // }
    }
        errorS = wheelSpeed - setpointS;
        integralS = integralS + errorS * ds;
        derivativeS = (errorS - previous_errorS) / ds;
        outputS = 0.34 * errorS + 0.33 * integralS + 0.33 * derivativeS;
        printf("\nOutput: %f\n", outputS);

        if (wheelSpeed > 0.28) {
            // Slow Down
            count -= 2;
            mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, count);
        }
        else if (wheelSpeed < 0.21) {
            // Speed up
            count += 2;
            mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, count);
        }
        else {
            count += 1;
            mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, count);
            count -= 1;
            mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, count);
        }

        previous_errorS = errorS;
        vTaskDelay(ds);
  
  }
  printf("STOOOOOOOOOOOOOOOOOOP!!!!!!!!!!!!\n");
  vTaskDelete(NULL);
}

static void IRsensor() {
    //Check if Two Point or Vref are burned into eFuse
    //check_efuse();

    //Configure ADC
    if (unit2 == ADC_UNIT_2) {
        adc1_config_width(width);
        adc1_config_channel_atten(channel2, atten2);
    } else {
        adc2_config_channel_atten((adc2_channel_t)channel2, atten2);
    }

    //Characterize ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit2, atten2, width, DEFAULT_VREF, adc_chars);
    print_char_val_type(val_type);

    float A = 0.0;

    //Continuously sample ADC1
    while (1) {
        uint32_t adc_reading = 0;
        //Multisampling
        for (int i = 0; i < NO_OF_SAMPLES; i++) {
            if (unit2 == ADC_UNIT_2) {
                adc_reading += adc1_get_raw((adc1_channel_t)channel2);
            } else {
                int raw;
                adc2_get_raw((adc2_channel_t)channel2, width, &raw);
                adc_reading += raw;
            }
        }
        adc_reading /= NO_OF_SAMPLES;
        //Convert adc_reading to voltage in mV
        uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
        //printf("Raw: %d\tVoltage: %dmV\n", adc_reading, voltage);
        A = 48401.54 / (voltage - 79.923);
        A = A * 1.16;
        // printf("Right Side Distance: %fcm\n", A);
        sdata_array[1] = A;
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

static void i2c_master_init()
{
  // Debug
  printf("\n>> i2c Config\n");
  int err;

  // Port configuration
  int i2c_master_port = I2C_EXAMPLE_MASTER_NUM;

  /// Define I2C configurations
  i2c_config_t conf;
  conf.mode = I2C_MODE_MASTER;                        // Master mode
  conf.sda_io_num = I2C_EXAMPLE_MASTER_SDA_IO;        // Default SDA pin
  conf.sda_pullup_en = GPIO_PULLUP_ENABLE;            // Internal pullup
  conf.scl_io_num = I2C_EXAMPLE_MASTER_SCL_IO;        // Default SCL pin
  conf.scl_pullup_en = GPIO_PULLUP_ENABLE;            // Internal pullup
  conf.master.clk_speed = I2C_EXAMPLE_MASTER_FREQ_HZ; // CLK frequency
  err = i2c_param_config(i2c_master_port, &conf);     // Configure
  if (err == ESP_OK)
  {
    printf("- parameters: ok\n");
  }

  // Install I2C driver
  err = i2c_driver_install(i2c_master_port, conf.mode,
                           I2C_EXAMPLE_MASTER_RX_BUF_DISABLE,
                           I2C_EXAMPLE_MASTER_TX_BUF_DISABLE, 0);
  if (err == ESP_OK)
  {
    printf("- initialized: yes\n");
  }

  // Data in MSB mode
  i2c_set_data_mode(i2c_master_port, I2C_DATA_MODE_MSB_FIRST, I2C_DATA_MODE_MSB_FIRST);
}

int getDeviceID2(uint8_t *data)
{
  int ret;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (SLAVE_ADDR2 << 1) | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, ADXL343_REG_DEVID, ACK_CHECK_EN);
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (SLAVE_ADDR2 << 1) | READ_BIT, ACK_CHECK_EN);
  i2c_master_read_byte(cmd, data, ACK_CHECK_DIS);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  return ret;
}

// Write one byte to register
void writeRegister2(uint8_t reg, uint8_t data)
{
  // int ret;
  // printf("--Writing %d to reg %d!--\n", data, reg);
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  //start command
  i2c_master_start(cmd);
  //slave address followed by write bit
  i2c_master_write_byte(cmd, (SLAVE_ADDR2 << 1) | WRITE_BIT, ACK_CHECK_EN);
  //register pointer sent
  i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
  //data sent
  i2c_master_write_byte(cmd, data, ACK_CHECK_EN);
  //stop command
  i2c_master_stop(cmd);
  i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
  // if (ret == ESP_OK)
  // {
  //   printf("I2C SUCCESSFUL \n");
  // }
  i2c_cmd_link_delete(cmd);
}

// Read register
uint8_t readRegister2(uint8_t reg)
{
  uint8_t value;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  //start command
  i2c_master_start(cmd);
  //slave followed by write bit
  i2c_master_write_byte(cmd, (SLAVE_ADDR2 << 1) | WRITE_BIT, ACK_CHECK_EN);
  //register pointer sent
  i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
  //stop command
  // i2c_master_stop(cmd);

  //repeated start command
  i2c_master_start(cmd);
  //slave followed by read bit
  i2c_master_write_byte(cmd, (SLAVE_ADDR2 << 1) | READ_BIT, ACK_CHECK_EN);
  //place data from register into bus
  i2c_master_read_byte(cmd, &value, ACK_CHECK_DIS);
  //stop command
  i2c_master_stop(cmd);
  i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  return value;
}

static void lidarRead()
{
  uint8_t initReg = 0x00;
  uint8_t initData = 0x04;
  uint8_t data = 0x06;
  vTaskDelay(22);
  while (1)
  {
    // Initialize lidar device
    writeRegister2(initReg, initData);
    data = readRegister2(0x01);
    while ((data & 1) != 0x00)
    {
      data = readRegister(0x01);
      vTaskDelay(10);
    }

    // Read high and low distance bits off device
    uint8_t distHigh = readRegister2(0x11);
    uint8_t distLow = readRegister2(0x10);

    // Print high and low distance data
    printf("High Data is: %x \n", distHigh);
    printf("Low Data is: %x \n", distLow);

    int distance = (distHigh << 8) + distLow;
    lidarDistance = distance;

    printf("Distance is: %d \n", distance);
    printf("lidarDistance is: %d \n", lidarDistance);
    vTaskDelay(100);
  }
}

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    ESP_ERROR_CHECK(example_connect());

    xTaskCreate(udp_client_task, "udp_client", 4096, NULL, 5, NULL);
    calibrateESC();

    pwm_init();
    i2c_example_master_init();
    i2c_master_init();
    i2c_scanner();
    rxtx_init();

    check_efuse();

    //Configure ADC
    if (unit == ADC_UNIT_1)
    {
        adc1_config_width(ADC_WIDTH_BIT_12);
        adc1_config_channel_atten(channel, atten);
    }
    else
    {
        adc2_config_channel_atten((adc2_channel_t)channel2, atten2);
    }

    //Characterize ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
    print_char_val_type(val_type);

    xTaskCreate(speedPID, "speedPID", 4096, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(test_alpha_display, "test_alpha_display", 4096, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(calculate_speed, "calculate_speed", 4096, NULL, configMAX_PRIORITIES-2, NULL);
    xTaskCreate(optical_sensor, "optical_sensor", 4096, NULL, 5, NULL);
    xTaskCreate(steeringPID, "steeringPID", 4096, NULL, configMAX_PRIORITIES-1, NULL);
    xTaskCreate(IRsensor, "IRsensor", 4096, NULL, configMAX_PRIORITIES-1, NULL);
    xTaskCreate(lidarRead, "lidarRead", 4096, NULL, configMAX_PRIORITIES-2, NULL);
    // xTaskCreate(tx_task, "tx_task", 2048, NULL, configMAX_PRIORITIES-3, NULL);
    // xTaskCreate(rx_task,"rx_task", 2048, NULL, configMAX_PRIORITIES-3, NULL);
}