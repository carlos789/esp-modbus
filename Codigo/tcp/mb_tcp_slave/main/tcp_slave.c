/*
 * SPDX-FileCopyrightText: 2016-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include "esp_err.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "mdns.h"
#include "esp_netif.h"
#include <math.h> // para usar rounf

#if __has_include("esp_mac.h")
#include "esp_mac.h"
#endif

#include "protocol_examples_common.h"

#include "mbcontroller.h"       // for mbcontroller defines and api
#include "modbus_params.h"      // for modbus parameters structures

#define MB_TCP_PORT_NUMBER      (CONFIG_FMB_TCP_PORT_DEFAULT)
/**************Max31865***************** */
#include <max31865.h>
#include "esp_idf_lib_helpers.h"
max31865_t ConfigMax31865(void);// Declaro la funcion para configurar el Max31865
#define HOST HELPER_SPI_HOST_DEFAULT

#if CONFIG_EXAMPLE_FILTER_50
#define FILTER MAX31865_FILTER_50HZ
#endif
#if CONFIG_EXAMPLE_FILTER_60
#define FILTER MAX31865_FILTER_60HZ
#endif
#if CONFIG_EXAMPLE_SCALE_ITS90
#define RTD_STANDARD MAX31865_ITS90
#endif
#if CONFIG_EXAMPLE_SCALE_DIN43760
#define RTD_STANDARD MAX31865_DIN43760
#endif
#if CONFIG_EXAMPLE_SCALE_US
#define RTD_STANDARD MAX31865_US_INDUSTRIAL
#endif

#if CONFIG_EXAMPLE_CONN_2WIRE
#define RTD_CONNECTION MAX31865_2WIRE
#endif
#if CONFIG_EXAMPLE_CONN_3WIRE
#define RTD_CONNECTION MAX31865_3WIRE
#endif
#if CONFIG_EXAMPLE_CONN_4WIRE
#define RTD_CONNECTION MAX31865_4WIRE
#endif
/**************************************** */
#include "string.h"
#define GPIO_COIL_0 GPIO_NUM_2 // Ejemplo: usar el GPIO 2 
#define GPIO_COIL_1 GPIO_NUM_4 // Ejemplo: usar el GPIO 4

// ************DS18B20*********** */
#include "ds18b20.h"  //Header sensor temperatura 
#include "onewire_bus.h"
#include "driver/gpio.h"
static void sensor_detect(void);// Declaro la funcion para detectar  el DS18B20
void sensor_read(void);// Declaro la funcion para leer el DS18B20
// Temp Sensors are on GPIO26
#define EXAMPLE_ONEWIRE_BUS_GPIO    1 //GPIO 1
#define EXAMPLE_ONEWIRE_MAX_DS18B20 2  
#define LED 2
#define HIGH 1
#define digitalWrite gpio_set_level
static int s_ds18b20_device_num = 0;
static float s_temperature = 0.0;
static ds18b20_device_handle_t s_ds18b20s[EXAMPLE_ONEWIRE_MAX_DS18B20];
static int H_temperature = 0;
int16_t TMax =0 ;// Regidtro para Modbus del Max31865
/********************************* */

// Defines below are used to define register start address for each type of Modbus registers
#define HOLD_OFFSET(field) ((uint16_t)(offsetof(holding_reg_params_t, field) >> 1))
#define INPUT_OFFSET(field) ((uint16_t)(offsetof(input_reg_params_t, field) >> 1))
#define MB_REG_DISCRETE_INPUT_START         (0x0000)
#define MB_REG_COILS_START                  (0x0000)
#define MB_REG_INPUT_START_AREA0            (INPUT_OFFSET(input_data0)) // register offset input area 0
#define MB_REG_INPUT_START_AREA1            (INPUT_OFFSET(input_data4)) // register offset input area 1
#define MB_REG_HOLDING_START_AREA0          (HOLD_OFFSET(holding_data0))
#define MB_REG_HOLDING_START_AREA0_SIZE     ((size_t)((HOLD_OFFSET(holding_data4) - HOLD_OFFSET(holding_data0)) << 1))
#define MB_REG_HOLDING_START_AREA1          (HOLD_OFFSET(holding_data4))
#define MB_REG_HOLDING_START_AREA1_SIZE     ((size_t)((HOLD_OFFSET(holding_area1_end) - HOLD_OFFSET(holding_data4)) << 1) + 4)
#define MB_REG_HOLDING_START_AREA2          (HOLD_OFFSET(holding_u8_a))
#define MB_REG_HOLDING_START_AREA2_SIZE     ((size_t)((HOLD_OFFSET(holding_area2_end) - HOLD_OFFSET(holding_u8_a)) << 1))

#define MB_PAR_INFO_GET_TOUT                (10) // Timeout for get parameter info
#define MB_CHAN_DATA_MAX_VAL                (60)
#define MB_CHAN_DATA_OFFSET                 (10.1f)

#define MB_READ_MASK                        (MB_EVENT_INPUT_REG_RD \
                                                | MB_EVENT_HOLDING_REG_RD \
                                                | MB_EVENT_DISCRETE_RD \
                                                | MB_EVENT_COILS_RD)
#define MB_WRITE_MASK                       (MB_EVENT_HOLDING_REG_WR \
                                                | MB_EVENT_COILS_WR)
#define MB_READ_WRITE_MASK                  (MB_READ_MASK | MB_WRITE_MASK)
#define MB_TEST_VALUE                       (12345.0)
#define MB_SLAVE_ADDR                       (CONFIG_MB_SLAVE_ADDR)
#define MB_CUST_DATA_MAX_LEN                (100)

static const char *TAG = "SLAVE_TEST";

static void *slave_handle = NULL;

// Set register values into known state
static void setup_reg_data(void)
{
    // Define initial state of parameters
    discrete_reg_params.discrete_input0 = 1;
    discrete_reg_params.discrete_input1 = 0;
    discrete_reg_params.discrete_input2 = 1;
    discrete_reg_params.discrete_input3 = 0;
    discrete_reg_params.discrete_input4 = 1;
    discrete_reg_params.discrete_input5 = 0;
    discrete_reg_params.discrete_input6 = 1;
    discrete_reg_params.discrete_input7 = 0;

    holding_reg_params.holding_data0 = 1.34F;
    holding_reg_params.holding_data1 = 2.56F;
    holding_reg_params.holding_data2 = 3.78F;
    holding_reg_params.holding_data3 = 4.90F;

    holding_reg_params.holding_data4 = 5.67F;
    holding_reg_params.holding_data5 = 6.78F;
    holding_reg_params.holding_data6 = 7.79F;
    holding_reg_params.holding_data7 = 8.80F;

#if CONFIG_FMB_EXT_TYPE_SUPPORT           //KCONFIG Name: FMB_EXT_TYPE_SUPPORT  Habilitar
    mb_set_uint8_a((val_16_arr *)&holding_reg_params.holding_u8_a[0], (uint8_t)0x55);
    mb_set_uint8_a((val_16_arr *)&holding_reg_params.holding_u8_a[1], (uint8_t)0x55);
    mb_set_uint8_b((val_16_arr *)&holding_reg_params.holding_u8_b[0], (uint8_t)0x55);
    mb_set_uint8_b((val_16_arr *)&holding_reg_params.holding_u8_b[1], (uint8_t)0x55);
    mb_set_uint16_ab((val_16_arr *)&holding_reg_params.holding_u16_ab[1], (uint16_t)MB_TEST_VALUE);
    mb_set_uint16_ab((val_16_arr *)&holding_reg_params.holding_u16_ab[0], (uint16_t)MB_TEST_VALUE);
    mb_set_uint16_ba((val_16_arr *)&holding_reg_params.holding_u16_ba[0], (uint16_t)MB_TEST_VALUE);
    mb_set_uint16_ba((val_16_arr *)&holding_reg_params.holding_u16_ba[1], (uint16_t)MB_TEST_VALUE);

    mb_set_float_abcd((val_32_arr *)&holding_reg_params.holding_float_abcd[0], (float)MB_TEST_VALUE);
    mb_set_float_abcd((val_32_arr *)&holding_reg_params.holding_float_abcd[1], (float)MB_TEST_VALUE);
    mb_set_float_cdab((val_32_arr *)&holding_reg_params.holding_float_cdab[0], (float)MB_TEST_VALUE);
    mb_set_float_cdab((val_32_arr *)&holding_reg_params.holding_float_cdab[1], (float)MB_TEST_VALUE);
    mb_set_float_badc((val_32_arr *)&holding_reg_params.holding_float_badc[0], (float)MB_TEST_VALUE);
    mb_set_float_badc((val_32_arr *)&holding_reg_params.holding_float_badc[1], (float)MB_TEST_VALUE);
    mb_set_float_dcba((val_32_arr *)&holding_reg_params.holding_float_dcba[0], (float)MB_TEST_VALUE);
    mb_set_float_dcba((val_32_arr *)&holding_reg_params.holding_float_dcba[1], (float)MB_TEST_VALUE);

    mb_set_uint32_abcd((val_32_arr *)&holding_reg_params.holding_uint32_abcd[0], (uint32_t)MB_TEST_VALUE);
    mb_set_uint32_abcd((val_32_arr *)&holding_reg_params.holding_uint32_abcd[1], (uint32_t)MB_TEST_VALUE);
    mb_set_uint32_cdab((val_32_arr *)&holding_reg_params.holding_uint32_cdab[0], (uint32_t)MB_TEST_VALUE);
    mb_set_uint32_cdab((val_32_arr *)&holding_reg_params.holding_uint32_cdab[1], (uint32_t)MB_TEST_VALUE);
    mb_set_uint32_badc((val_32_arr *)&holding_reg_params.holding_uint32_badc[0], (uint32_t)MB_TEST_VALUE);
    mb_set_uint32_badc((val_32_arr *)&holding_reg_params.holding_uint32_badc[1], (uint32_t)MB_TEST_VALUE);
    mb_set_uint32_dcba((val_32_arr *)&holding_reg_params.holding_uint32_dcba[0], (uint32_t)MB_TEST_VALUE);
    mb_set_uint32_dcba((val_32_arr *)&holding_reg_params.holding_uint32_dcba[1], (uint32_t)MB_TEST_VALUE);

    mb_set_double_abcdefgh((val_64_arr *)&holding_reg_params.holding_double_abcdefgh[0], (double)MB_TEST_VALUE);
    mb_set_double_abcdefgh((val_64_arr *)&holding_reg_params.holding_double_abcdefgh[1], (double)MB_TEST_VALUE);
    mb_set_double_hgfedcba((val_64_arr *)&holding_reg_params.holding_double_hgfedcba[0], (double)MB_TEST_VALUE);
    mb_set_double_hgfedcba((val_64_arr *)&holding_reg_params.holding_double_hgfedcba[1], (double)MB_TEST_VALUE);
    mb_set_double_ghefcdab((val_64_arr *)&holding_reg_params.holding_double_ghefcdab[0], (double)MB_TEST_VALUE);
    mb_set_double_ghefcdab((val_64_arr *)&holding_reg_params.holding_double_ghefcdab[1], (double)MB_TEST_VALUE);
    mb_set_double_badcfehg((val_64_arr *)&holding_reg_params.holding_double_badcfehg[0], (double)MB_TEST_VALUE);
    mb_set_double_badcfehg((val_64_arr *)&holding_reg_params.holding_double_badcfehg[1], (double)MB_TEST_VALUE);
#endif

    coil_reg_params.coils_port0 = 0x00;
    coil_reg_params.coils_port1 = 0x00;

    input_reg_params.input_data0 = 1.12F;
    input_reg_params.input_data1 = 2.34F;
    input_reg_params.input_data2 = 3.56F;
    input_reg_params.input_data3 = 4.78F;
    input_reg_params.input_data4 = 1.12F;
    input_reg_params.input_data5 = 2.34F;
    input_reg_params.input_data6 = 3.56F;
    input_reg_params.input_data7 = 4.78F;
}

static void slave_operation_func(void *arg)
{
    mb_param_info_t reg_info; // keeps the Modbus registers access information

    ESP_LOGI(TAG, "Modbus slave stack initialized.");
    ESP_LOGI(TAG, "Start modbus test...");
    sensor_detect();
    max31865_t dev = ConfigMax31865( ); // Configuracion de SPI y Max
    // The cycle below will be terminated when parameter holding_data0
    // incremented each access cycle reaches the CHAN_DATA_MAX_VAL value.
    for(;holding_reg_params.holding_data0 < MB_CHAN_DATA_MAX_VAL;) {
        // Check for read/write events of Modbus master for certain events
        (void)mbc_slave_check_event(slave_handle, MB_READ_WRITE_MASK);
        ESP_ERROR_CHECK_WITHOUT_ABORT(mbc_slave_get_param_info(slave_handle, &reg_info, MB_PAR_INFO_GET_TOUT));
        const char* rw_str = (reg_info.type & MB_READ_MASK) ? "READ" : "WRITE";
        // Filter events and process them accordingly
        if(reg_info.type & (MB_EVENT_HOLDING_REG_WR | MB_EVENT_HOLDING_REG_RD)) {
            // Get parameter information from parameter queue
            ESP_LOGI(TAG, "HOLDING %s (%u us), ADDR:%u, TYPE:%u, INST_ADDR:0x%.4x, SIZE:%u",
                    rw_str,
                    (unsigned)reg_info.time_stamp,
                    (unsigned)reg_info.mb_offset,
                    (unsigned)reg_info.type,
                    (int)reg_info.address,
                    (unsigned)reg_info.size);
                    sensor_read();
		            if (s_temperature < 100.0)//para evitar falsas lecturas
		                    {
                            printf("Temperatura exterior : %0.1fC\n", s_temperature);
		                    printf("Temperatura Hexa : %4X \n", H_temperature);
                            
                            //holding_reg_params.holding_u16_ab[0] = 0x1089;
                            holding_reg_params.holding_u16_ab[1] = H_temperature;
                            }
                            float temperature;
                            esp_err_t res = max31865_measure(&dev, &temperature);
                            printf("Temperatura interior : %0.2fC\n", temperature);
                            temperature=temperature * 100.0;//es para no perder las centesimas
                            TMax= (int16_t)roundf (temperature)  ;
                            holding_reg_params.holding_u16_ab[0] = TMax; 
            if (reg_info.address == (uint8_t*)&holding_reg_params.holding_data0)
            {
                (void)mbc_slave_lock(slave_handle);
                holding_reg_params.holding_data0 += MB_CHAN_DATA_OFFSET;
                if (holding_reg_params.holding_data0 >= (MB_CHAN_DATA_MAX_VAL - MB_CHAN_DATA_OFFSET)) {
                    coil_reg_params.coils_port1 = 0xFF;
                    ESP_LOGI(TAG, "Riched maximum value");
                }
                (void)mbc_slave_unlock(slave_handle);
            }
        } else if (reg_info.type & MB_EVENT_INPUT_REG_RD) {
            ESP_LOGI(TAG, "INPUT READ (%" PRIu32 " us), ADDR:%u, TYPE:%u, INST_ADDR:0x%" PRIx32 ", SIZE:%u",
                            reg_info.time_stamp,
                            (unsigned)reg_info.mb_offset,
                            (unsigned)reg_info.type,
                            (uint32_t)reg_info.address,
                            (unsigned)reg_info.size);
        } else if (reg_info.type & MB_EVENT_DISCRETE_RD) {
            ESP_LOGI(TAG, "DISCRETE READ (%" PRIu32 " us): ADDR:%u, TYPE:%u, INST_ADDR:0x%" PRIx32 ", SIZE:%u",
                            reg_info.time_stamp,
                            (unsigned)reg_info.mb_offset,
                            (unsigned)reg_info.type,
                            (uint32_t)reg_info.address,
                            (unsigned)reg_info.size);
        } else if (reg_info.type & (MB_EVENT_COILS_RD | MB_EVENT_COILS_WR)) {
            ESP_LOGI(TAG, "COILS %s (%" PRIu32 " us), ADDR:%u, TYPE:%u, INST_ADDR:0x%" PRIx32 ", SIZE:%u",
                            rw_str,
                            reg_info.time_stamp,
                            (unsigned)reg_info.mb_offset,
                            (unsigned)reg_info.type,
                            (uint32_t)reg_info.address,
                            (unsigned)reg_info.size);

                            uint16_t coil_address = reg_info.mb_offset;// Captura el byte completo que contiene las bobinas
                            uint8_t nuevo_estado = coil_reg_params.coils_port0;
                            printf  ("Acceso a la bobina %u. Nuevo estado: %2X\n", 
                            coil_address, nuevo_estado  );
                            if (nuevo_estado & (1 << 0)) { 
                                gpio_set_level(GPIO_COIL_0, 0); // Enciende relé (Nivel ALTO)
                             } else {
                                gpio_set_level(GPIO_COIL_0, 1); // Apaga relé (Nivel BAJO)
                            }
                            // --- Lógica para la Bobina 1 (GPIO 4) ---
                            // Usamos operador bit a bit (&) para verificar si el BIT 1 (valor 2) está activo
                            if (nuevo_estado & (1 << 1)) { 
                                gpio_set_level(GPIO_COIL_1,0); // Enciende relé (Nivel ALTO)
                             } else {
                                gpio_set_level(GPIO_COIL_1, 1); // Apaga relé (Nivel BAJO)
                             }

    
            if (coil_reg_params.coils_port1 == 0xFF) {
                ESP_LOGI(TAG, "Stop polling.");
                break;
            }
        }
    }
    // Destroy of Modbus controller on alarm
    ESP_LOGI(TAG,"Modbus controller destroyed.");
    vTaskDelay(100);
}

static esp_err_t init_services(void)
{
    esp_err_t result = nvs_flash_init();
    if (result == ESP_ERR_NVS_NO_FREE_PAGES || result == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      result = nvs_flash_init();
    }
    MB_RETURN_ON_FALSE((result == ESP_OK), ESP_ERR_INVALID_STATE,
                            TAG,
                            "nvs_flash_init fail, returns(0x%x).",
                            (int)result);
    result = esp_netif_init();
    MB_RETURN_ON_FALSE((result == ESP_OK), ESP_ERR_INVALID_STATE,
                            TAG,
                            "esp_netif_init fail, returns(0x%x).",
                            (int)result);
    result = esp_event_loop_create_default();
    MB_RETURN_ON_FALSE((result == ESP_OK), ESP_ERR_INVALID_STATE,
                            TAG,
                            "esp_event_loop_create_default fail, returns(0x%x).",
                            (int)result);
    // This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
    // Read "Establishing Wi-Fi or Ethernet Connection" section in
    // examples/protocols/README.md for more information about this function.
    result = example_connect();
    MB_RETURN_ON_FALSE((result == ESP_OK), ESP_ERR_INVALID_STATE,
                                TAG,
                                "example_connect fail, returns(0x%x).",
                                (int)result);
#if CONFIG_EXAMPLE_CONNECT_WIFI
    result = esp_wifi_set_ps(WIFI_PS_NONE);
    MB_RETURN_ON_FALSE((result == ESP_OK), ESP_ERR_INVALID_STATE,
                                   TAG,
                                   "esp_wifi_set_ps fail, returns(0x%x).",
                                   (int)result);
#endif
    return ESP_OK;
}

static esp_err_t destroy_services(void)
{
    esp_err_t err = ESP_OK;

    err = example_disconnect();
    MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE,
                                   TAG,
                                   "example_disconnect fail, returns(0x%x).",
                                   (int)err);
    err = esp_event_loop_delete_default();
    MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE,
                                       TAG,
                                       "esp_event_loop_delete_default fail, returns(0x%x).",
                                       (int)err);
    err = esp_netif_deinit();
    MB_RETURN_ON_FALSE((err == ESP_OK || err == ESP_ERR_NOT_SUPPORTED), ESP_ERR_INVALID_STATE,
                                        TAG,
                                        "esp_netif_deinit fail, returns(0x%x).",
                                        (int)err);
    err = nvs_flash_deinit();
    MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE,
                                TAG,
                                "nvs_flash_deinit fail, returns(0x%x).",
                                (int)err);
    return err;
}

// This is a simple custom function handler for the command.
// The handler is executed from the context of modbus controller event task and should be as simple as possible.
// Parameters: frame_ptr - the pointer to the incoming ADU request frame from master starting from function code,
// len - the pointer to length of the frame. The handler body can override the buffer and return the length of data.
// After return from the handler the modbus object will handle the end of transaction according to the exception returned,
// then builds the response frame and send it back to the master. If the whole transaction time including the response
// latency exceeds the configured slave response time set in the master configuration the master will ignore the transaction.
mb_exception_t my_custom_fc_handler(void *inst, uint8_t *frame_ptr, uint16_t *len)
{
    char *str_append = ":Slave";
    MB_RETURN_ON_FALSE((frame_ptr && len && *len < (MB_CUST_DATA_MAX_LEN - strlen(str_append))), MB_EX_ILLEGAL_DATA_VALUE, TAG,
                            "incorrect custom frame");
    frame_ptr[*len] = '\0';
    strcat((char *)&frame_ptr[1], str_append);
    *len = (strlen(str_append) + *len); // the length of (response + command)
    return MB_EX_NONE; // Set the exception code for modbus object appropriately
}

// Modbus slave initialization
static esp_err_t slave_init(mb_communication_info_t *pcomm_info)
{
    mb_register_area_descriptor_t reg_area = {0}; // Modbus register area descriptor structure

    // Initialization of Modbus controller
    esp_err_t err = mbc_slave_create_tcp(pcomm_info, &slave_handle);
    MB_RETURN_ON_FALSE((err == ESP_OK && slave_handle != NULL), ESP_ERR_INVALID_STATE,
                                TAG,
                                "mb controller create fail.");

    uint8_t custom_command = 0x41;
    // Delete the handler for specified command, if available.
    err = mbc_delete_handler(slave_handle, custom_command);
    MB_RETURN_ON_FALSE((err == ESP_OK  || err == ESP_ERR_INVALID_STATE), ESP_ERR_INVALID_STATE, TAG,
                        "could not reset handler, returned (0x%x).", (int)err);
    err = mbc_set_handler(slave_handle, custom_command, my_custom_fc_handler);
    MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE, TAG,
                        "could not set or override handler, returned (0x%x).", (int)err);
    mb_fn_handler_fp handler = NULL;
    err = mbc_get_handler(slave_handle, custom_command, &handler);
    MB_RETURN_ON_FALSE((err == ESP_OK && handler == my_custom_fc_handler), ESP_ERR_INVALID_STATE, TAG,
                        "could not get handler for command %d, returned (0x%x).", (int)custom_command, (int)err);

    // The code below initializes Modbus register area descriptors
    // for Modbus Holding Registers, Input Registers, Coils and Discrete Inputs
    // Initialization should be done for each supported Modbus register area according to register map.
    // When external master trying to access the register in the area that is not initialized
    // by mbc_slave_set_descriptor() API call then Modbus stack
    // will send exception response for this register area.
    reg_area.type = MB_PARAM_HOLDING; // Set type of register area
    reg_area.start_offset = MB_REG_HOLDING_START_AREA0; // Offset of register area in Modbus protocol
    reg_area.address = (void*)&holding_reg_params.holding_data0; // Set pointer to storage instance
    reg_area.size = (MB_REG_HOLDING_START_AREA1 - MB_REG_HOLDING_START_AREA0) << 1; // Set the size of register storage instance
    reg_area.access = MB_ACCESS_RW;
    err = mbc_slave_set_descriptor(slave_handle, reg_area);
    MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE,
                                    TAG,
                                    "mbc_slave_set_descriptor fail, returns(0x%x).",
                                    (int)err);

    reg_area.type = MB_PARAM_HOLDING; // Set type of register area
    reg_area.start_offset = MB_REG_HOLDING_START_AREA1; // Offset of register area in Modbus protocol
    reg_area.address = (void*)&holding_reg_params.holding_data4; // Set pointer to storage instance
    reg_area.size = MB_REG_HOLDING_START_AREA1_SIZE; // Set the size of register storage instance
    reg_area.access = MB_ACCESS_RW;
    err = mbc_slave_set_descriptor(slave_handle, reg_area);
    MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE,
                                    TAG,
                                    "mbc_slave_set_descriptor fail, returns(0x%x).",
                                    (int)err);

#if CONFIG_FMB_EXT_TYPE_SUPPORT
    // The extended parameters register area
    reg_area.type = MB_PARAM_HOLDING;
    reg_area.start_offset = MB_REG_HOLDING_START_AREA2;
    reg_area.address = (void*)&holding_reg_params.holding_u8_a;
    reg_area.size = MB_REG_HOLDING_START_AREA2_SIZE;
    reg_area.access = MB_ACCESS_RW;
    err = mbc_slave_set_descriptor(slave_handle, reg_area);
    MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE,
                                        TAG,
                                        "mbc_slave_set_descriptor fail, returns(0x%x).",
                                        (int)err);
#endif

    // Initialization of Input Registers area
    reg_area.type = MB_PARAM_INPUT;
    reg_area.start_offset = MB_REG_INPUT_START_AREA0;
    reg_area.address = (void*)&input_reg_params.input_data0;
    reg_area.size = sizeof(float) << 2;
    reg_area.access = MB_ACCESS_RW;
    err = mbc_slave_set_descriptor(slave_handle, reg_area);
    MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE,
                                        TAG,
                                        "mbc_slave_set_descriptor fail, returns(0x%x).",
                                        (int)err);
    reg_area.type = MB_PARAM_INPUT;
    reg_area.start_offset = MB_REG_INPUT_START_AREA1;
    reg_area.address = (void*)&input_reg_params.input_data4;
    reg_area.size = sizeof(float) << 2;
    reg_area.access = MB_ACCESS_RW;
    err = mbc_slave_set_descriptor(slave_handle, reg_area);
    MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE,
                                        TAG,
                                        "mbc_slave_set_descriptor fail, returns(0x%x).",
                                        (int)err);

    // Initialization of Coils register area
    reg_area.type = MB_PARAM_COIL;
    reg_area.start_offset = MB_REG_COILS_START;
    reg_area.address = (void*)&coil_reg_params;
    reg_area.size = sizeof(coil_reg_params);
    reg_area.access = MB_ACCESS_RW;
    err = mbc_slave_set_descriptor(slave_handle, reg_area);
    MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE,
                                    TAG,
                                    "mbc_slave_set_descriptor fail, returns(0x%x).",
                                    (int)err);

    // Initialization of Discrete Inputs register area
    reg_area.type = MB_PARAM_DISCRETE;
    reg_area.start_offset = MB_REG_DISCRETE_INPUT_START;
    reg_area.address = (void*)&discrete_reg_params;
    reg_area.size = sizeof(discrete_reg_params);
    reg_area.access = MB_ACCESS_RW;
    err = mbc_slave_set_descriptor(slave_handle, reg_area);
    MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE,
                                    TAG,
                                    "mbc_slave_set_descriptor fail, returns(0x%x).",
                                    (int)err);

    // Set values into known state
    setup_reg_data();

    // Starts of modbus controller and stack
    err = mbc_slave_start(slave_handle);
    MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE,
                                        TAG,
                                        "mbc_slave_start fail, returns(0x%x).",
                                        (int)err);
    vTaskDelay(5);
    return err;
}

static esp_err_t slave_destroy(void)
{
    esp_err_t err = mbc_slave_delete(slave_handle);
    MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE,
                                TAG,
                                "mbc_slave_destroy fail, returns(0x%x).",
                                (int)err);
    return err;
}
//*******************Funciones DS18B20********************* */
static void sensor_detect(void)
{
    // install 1-wire bus
    onewire_bus_handle_t bus = NULL;
    onewire_bus_config_t bus_config = {
        .bus_gpio_num = EXAMPLE_ONEWIRE_BUS_GPIO,
    };
    onewire_bus_rmt_config_t rmt_config = {
        .max_rx_bytes = 10, // 1byte ROM command + 8byte ROM number + 1byte device command
    };
    ESP_ERROR_CHECK(onewire_new_bus_rmt(&bus_config, &rmt_config, &bus));

    onewire_device_iter_handle_t iter = NULL;
    onewire_device_t next_onewire_device;
    esp_err_t search_result = ESP_OK;

    // create 1-wire device iterator, which is used for device search
    ESP_ERROR_CHECK(onewire_new_device_iter(bus, &iter));
    ESP_LOGI(TAG, "Device iterator created, start searching...");
    do {
        search_result = onewire_device_iter_get_next(iter, &next_onewire_device);
        if (search_result == ESP_OK) { // found a new device, let's check if we can upgrade it to a DS18B20
            ds18b20_config_t ds_cfg = {};
            // check if the device is a DS18B20, if so, return the ds18b20 handle
            if (ds18b20_new_device_from_enumeration(&next_onewire_device, &ds_cfg, &s_ds18b20s[s_ds18b20_device_num]) == ESP_OK) {
                ESP_LOGI(TAG, "Found a DS18B20[%d], address: %016llX", s_ds18b20_device_num, next_onewire_device.address);
                s_ds18b20_device_num++;
            } else {
                ESP_LOGI(TAG, "Found an unknown device, address: %016llX", next_onewire_device.address);
            }
        }
    } while (search_result != ESP_ERR_NOT_FOUND);
    ESP_ERROR_CHECK(onewire_del_device_iter(iter));
    ESP_LOGI(TAG, "Searching done, %d DS18B20 device(s) found", s_ds18b20_device_num);
}

void sensor_read(void)
{
    for (int i = 0; i < s_ds18b20_device_num; i ++) {
        ESP_ERROR_CHECK(ds18b20_trigger_temperature_conversion(s_ds18b20s[i]));
        ESP_ERROR_CHECK(ds18b20_get_temperature(s_ds18b20s[i], &s_temperature, &H_temperature));
        //ESP_LOGI(TAG, "temperature read from DS18B20[%d]: %.2fC", i, s_temperature);
        //printf("Temperatura Hexa : %4X \n", H_temperature);
    }
}

/************************Funciones Max31865 y SPI*********************** */
static max31865_config_t config =
{
    .v_bias = true,
    .filter = FILTER,
    .mode = MAX31865_MODE_SINGLE,
    .connection = RTD_CONNECTION
};

max31865_t ConfigMax31865(void)
{
    // Configure SPI bus
    spi_bus_config_t cfg =
    {
        .mosi_io_num = CONFIG_EXAMPLE_MOSI_GPIO,
        .miso_io_num = CONFIG_EXAMPLE_MISO_GPIO,
        .sclk_io_num = CONFIG_EXAMPLE_SCLK_GPIO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 0,
        .flags = 0
    };
    //ESP_ERROR_CHECK(spi_bus_initialize(HOST, &cfg, 1));
    ESP_ERROR_CHECK(spi_bus_initialize(HOST, &cfg, SPI_DMA_CH_AUTO));// Cambio para la ESP32 S# para que no entre en PANIC

    // Init device
    max31865_t dev =
    {
        .standard = RTD_STANDARD,
        .r_ref = CONFIG_EXAMPLE_RTD_REF,
        .rtd_nominal = CONFIG_EXAMPLE_RTD_NOMINAL,
    };
    ESP_ERROR_CHECK(max31865_init_desc(&dev, HOST, MAX31865_MAX_CLOCK_SPEED_HZ, CONFIG_EXAMPLE_CS_GPIO));

    // Configure device
    ESP_ERROR_CHECK(max31865_set_config(&dev, &config));
   
    return dev;
}
/********************************Fin************************************ */
// An example application of Modbus slave. It is based on esp-modbus stack.
// See deviceparams.h file for more information about assigned Modbus parameters.
// These parameters can be accessed from main application and also can be changed
// by external Modbus master host.
void app_main(void)
{
    ESP_ERROR_CHECK(init_services());
    gpio_set_direction(GPIO_COIL_0, GPIO_MODE_OUTPUT);// Configuro salida COIL 0
    gpio_set_direction(GPIO_COIL_1, GPIO_MODE_OUTPUT);// Configuro salida COIL 1
    gpio_set_level(GPIO_COIL_0, 1);// Apagp RELE 0
    gpio_set_level(GPIO_COIL_1, 1);// Apagp RELE 1
    // Set UART log level
    esp_log_level_set(TAG, ESP_LOG_INFO);

    mb_communication_info_t tcp_slave_config = {
        .tcp_opts.port = MB_TCP_PORT_NUMBER,
        .tcp_opts.mode = MB_TCP,
#if !CONFIG_EXAMPLE_CONNECT_IPV6
        .tcp_opts.addr_type = MB_IPV4,
#else
        .tcp_opts.addr_type = MB_IPV6,
#endif
        .tcp_opts.ip_addr_table = NULL, // Bind to any address
        .tcp_opts.ip_netif_ptr = (void*)get_example_netif(),
        .tcp_opts.uid = MB_SLAVE_ADDR
    };

    ESP_ERROR_CHECK(slave_init(&tcp_slave_config));

    // The Modbus slave logic is located in this function (user handling of Modbus)
    slave_operation_func(NULL);

    ESP_ERROR_CHECK(slave_destroy());
    ESP_ERROR_CHECK(destroy_services());
}
