#include "oled.h"

static oled_params_t current_oled_params;
const char *OLED_LIBRARY_TAG = "oled_library";

void oled_image(const unsigned char *image_data) {
    oled_setXY(0x00, 0x7F, 0x00, 0x07);

    for (int k = 0; k <= 1023; k++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        uint8_t address = OLED_ADDR;
        uint8_t write_buffer[2] = { 0x00 };

        write_buffer[0] = 0x40;
        write_buffer[1] = image_data[k];

        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write(cmd, &write_buffer[0], 1, true);
        i2c_master_write(cmd, &write_buffer[1], 1, true);
        i2c_master_stop(cmd);

        esp_err_t ret = i2c_master_cmd_begin(current_oled_params.i2c_port_num, cmd, 1000 / portTICK_PERIOD_MS);
        oled_check_i2c_transaction(ret, "oled_image");
        i2c_cmd_link_delete(cmd);
    } 
}

void oled_string(char *text) {
    int len = strlen(text);
  
    for (int g = 0; g < len; g++) {    
        for (int index = 0; index < 5; index++) {     
            i2c_cmd_handle_t cmd = i2c_cmd_link_create();
            uint8_t address = OLED_ADDR;
            uint8_t write_buffer[2] = { 0x00 };

            write_buffer[0] = 0x40;
            write_buffer[1] = ASCII[text[g] - 0x20][index];

            i2c_master_start(cmd);
            i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);
            i2c_master_write(cmd, &write_buffer[0], 1, true);
            i2c_master_write(cmd, &write_buffer[1], 1, true);
            i2c_master_stop(cmd);

            esp_err_t ret = i2c_master_cmd_begin(current_oled_params.i2c_port_num, cmd, 1000 / portTICK_PERIOD_MS);
            oled_check_i2c_transaction(ret, "oled_string");
            i2c_cmd_link_delete(cmd);
        }    
    }
}

void oled_command(uint8_t command) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    uint8_t address = OLED_ADDR;
    uint8_t write_buffer[2] = { 0x00 };

    write_buffer[0] = 0x00;
    write_buffer[1] = command;

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, &write_buffer[0], 1, true);
    i2c_master_write(cmd, &write_buffer[1], 1, true);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(current_oled_params.i2c_port_num, cmd, 1000 / portTICK_PERIOD_MS);
    oled_check_i2c_transaction(ret, "oled_command");
    i2c_cmd_link_delete(cmd);
}

void oled_setXY(uint8_t col_start, uint8_t col_end, uint8_t page_start, uint8_t page_end) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    uint8_t address = OLED_ADDR;
    uint8_t write_buffer[7] = { 0x00 };

    write_buffer[0] = 0x00;
    write_buffer[1] = 0x21;
    write_buffer[2] = col_start;
    write_buffer[3] = col_end;
    write_buffer[4] = 0x22;
    write_buffer[5] = page_start;
    write_buffer[6] = col_end;

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, &write_buffer[0], 1, true);
    i2c_master_write(cmd, &write_buffer[1], 1, true);
    i2c_master_write(cmd, &write_buffer[2], 1, true);
    i2c_master_write(cmd, &write_buffer[3], 1, true);
    i2c_master_write(cmd, &write_buffer[4], 1, true);
    i2c_master_write(cmd, &write_buffer[5], 1, true);
    i2c_master_write(cmd, &write_buffer[6], 1, true);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(current_oled_params.i2c_port_num, cmd, 1000 / portTICK_PERIOD_MS);
    oled_check_i2c_transaction(ret, "oled_setXY");
    i2c_cmd_link_delete(cmd);
}

void oled_clear(void) {
    oled_setXY(0x00, 0x7F, 0x00, 0x07);

    for (int k = 0; k <= 1023; k++)
    {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        uint8_t address = OLED_ADDR;
        uint8_t write_buffer[2] = { 0x00 };

        write_buffer[0] = 0x40;
        write_buffer[1] = 0x00;

        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write(cmd, &write_buffer[0], 1, true);
        i2c_master_write(cmd, &write_buffer[1], 1, true);
        i2c_master_stop(cmd);

        esp_err_t ret = i2c_master_cmd_begin(current_oled_params.i2c_port_num, cmd, 1000 / portTICK_PERIOD_MS);
        oled_check_i2c_transaction(ret, "oled_clear");
        i2c_cmd_link_delete(cmd);
    } 
}

void oled_configure(void) {
    oled_command(OLED_CMD_DISPLAY_OFF);
    oled_command(OLED_CMD_DISPLAY_CLK_DIV);
    oled_command(OLED_DISPLAY_CLK_DIV_VAL);
    oled_command(OLED_CMD_MULTIPLEX_RATIO);
    oled_command(OLED_64_COM_LINES_MULIPLEX_RATIO_VAL);
    oled_command(OLED_CMD_DISPLAY_OFFSET);
    oled_command(OLED_DISPLAY_OFFSET_VAL);
    oled_command(OLED_CMD_START_LINE);
    oled_command(OLED_CMD_CHARGE_PUMP);
    oled_command(OLED_CHARGE_PUMP_VAL);
    oled_command(OLED_CMD_MEMORY_ADDRESS);
    oled_command(OLED_MEMORY_ADDRESS_VAL);
    oled_command(OLED_CMD_SET_SEGMENT_REMAP);
    oled_command(OLED_CMD_COM_OUTPUT_SCAN_DIR);
    oled_command(OLED_CMD_COM_PINS_HW);
    oled_command(OLED_COM_PINS_HW_VAL);
    oled_command(OLED_CMD_SET_CONTRAST_CONTROL);
    oled_command(OLED_SET_CONTRAST_CONTROL_VAL);
    oled_command(OLED_CMD_SET_PRE_CHARGE_PERIOD);
    oled_command(OLED_SET_PRE_CHARGE_PERIOD_VAL);
    oled_command(OLED_CMD_SETVCOM);
    oled_command(OLED_SETVCOM_VAL);
    oled_command(OLED_CMD_DISPLAY_ON);
    oled_command(OLED_CMD_DISPLAY_MODE);
    oled_command(OLED_CMD_DEACTIVATE_SCROLL);
    oled_command(OLED_CMD_DISPLAY_ON_NORMAL);
}

void oled_init(oled_params_t params) {
    i2c_config_t config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = params.sda_io_num,
        .scl_io_num = params.scl_io_num,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = params.clk_speed
    };

    i2c_param_config(params.i2c_port_num, &config);
    i2c_driver_install(params.i2c_port_num, config.mode, 0, 0, 0);
    oled_configure();

    current_oled_params = params;
}

void oled_check_i2c_transaction(esp_err_t ret, const char* method_name) {
    if (ret != ESP_OK) {
        ESP_LOGE(OLED_LIBRARY_TAG, "There was an error trying to complete the I2C transaction in method %s, the error was: %s", method_name, esp_err_to_name(ret));
    }
}