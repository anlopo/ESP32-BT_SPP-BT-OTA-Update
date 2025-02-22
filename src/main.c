#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_spp_api.h"
#include "bootloader_common.h"
#include <string.h>
#include <ctype.h>

#define CHUNK_SIZE 1024
#define HASH_SIZE 32

#ifdef DEBUG_BUILD
static bool debug_enabled = true;
#else
static bool debug_enabled = false;
#endif

static const char *TAG = "ESP32_OTA_BT";
static size_t fw_size;
static esp_ota_handle_t ota_handle;
static const esp_partition_t *ota_partition;
static uint32_t spp_connection_handle;
static bool next_spp_data_is_size = false, next_spp_data_is_update = false;
static bool spp_send_ota_status_after_connect = false;

RTC_NOINIT_ATTR int ota_restart; // bool type does not work

static void writeOTAdata(uint8_t *data, int len);
static void sppCallback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param);
static void process_bt_message(char *bt_message);
static void send_bt_message(char *response);
static void trim_whitespace(char *str);
static void bt_init();

void app_main()
{
    // Check if debugging is enabled
    if (!debug_enabled)
        esp_log_level_set("*", ESP_LOG_ERROR); // Set log level to ERROR if debugging is not enabled
    else
        esp_log_level_set("*", ESP_LOG_DEBUG); // Set log level to DEBUG if debugging is enabled

    // Log the start of ESP32 OTA Update over Bluetooth SPP
    ESP_LOGI(TAG, "Starting ESP32 OTA Update over Bluetooth SPP");

    // Check if a restart for OTA is needed
    if (ota_restart == 1)
    {
        ESP_LOGI("OTA", "OTA OK");                // Log that the OTA process is successful
        spp_send_ota_status_after_connect = true; // Set flag to send OTA status after connection
    }
    ota_restart = 0; // Reset the ota_restart flag

    // Attempt to initialize the NVS flash storage
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        // Handle the case where NVS has no free pages or a new version is found
        ESP_ERROR_CHECK(nvs_flash_erase()); // Erase all data in NVS
        ret = nvs_flash_init();             // Reinitialize NVS flash storage
    }
    ESP_ERROR_CHECK(ret); // Ensure there are no errors during NVS initialization

    // Initialize the Bluetooth module for further operations
    bt_init();
}

// SPP callback fucntion
static void sppCallback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
    switch (event)
    {
    case ESP_SPP_SRV_OPEN_EVT: // This event is triggered when a server-mode Bluetooth SPP connection is successfully opened.
    case ESP_SPP_OPEN_EVT:     // This event is triggered when a client-mode Bluetooth SPP connection is successfully opened.
        ESP_LOGI("BT_SPP", "SPP connection opened, handle=%lu", param->open.handle);
        spp_connection_handle = param->open.handle; // Store the connection handle for future reference.

        // If a flag indicates that OTA status should be sent after connecting, send it and then reset the flag.
        if (spp_send_ota_status_after_connect)
        {
            spp_send_ota_status_after_connect = false;
            send_bt_message("\n\nLast OTA OK\r\n");
        }
        break;

    case ESP_SPP_CLOSE_EVT:
        ESP_LOGI("BT_SPP", "SPP connection closed, resetting handle.");
        spp_connection_handle = 0; // Reset the connection handle when the connection is closed to indicate no active connection.
        break;

    case ESP_SPP_DATA_IND_EVT:
        // If spp_connection_handle is not set and data is received, set spp_connection_handle using the handle from the event parameter.
        if (spp_connection_handle == 0)
        {
            spp_connection_handle = param->data_ind.handle;
            ESP_LOGI("BT_SPP", "Manually setting spp_connection_handle = %lu", spp_connection_handle);
        }

        // Check if the next data expected is an OTA update. If so, write the received data to the OTA function.
        if (next_spp_data_is_update)
        {
            writeOTAdata(param->data_ind.data, param->data_ind.len); // Write the data received to the OTA handler.
            break;
        }

        char received_data[128] = {0}; // Buffer to hold incoming data. Ensure it's initialized to zero.

        // Check if the length of the received data exceeds the buffer size.
        if (param->data_ind.len >= sizeof(received_data))
        {
            ESP_LOGE(TAG, "Received data too large, truncating.");
            memcpy(received_data, param->data_ind.data, sizeof(received_data) - 1); // Copy as much data as can fit in the buffer, leaving space for null termination.
            received_data[sizeof(received_data) - 1] = '\0';                        // Null-terminate the string to prevent overflow.
        }
        else
        {
            memcpy(received_data, param->data_ind.data, param->data_ind.len); // Copy exactly the amount of data received.
            received_data[param->data_ind.len] = '\0';                        // Null-terminate the string.
        }

        trim_whitespace(received_data); // Remove any leading or trailing whitespace from the received data.

        ESP_LOGI("BT_SPP", "Received data: '%s', Handle: %lu", received_data, param->data_ind.handle);
        process_bt_message(received_data); // Process the incoming Bluetooth SPP message.
        break;

    case ESP_SPP_CONG_EVT:
        ESP_LOGI(TAG, "SPP Congestion event"); // This event indicates that there is congestion on the SPP channel.
        break;

    default:
        break; // For any other events that are not handled explicitly, take no action.
    }
}

// Write data from SPP to ota, then check it
static void writeOTAdata(uint8_t *data, int len)
{
    static int received = 0; // Keeps track of the total data received for OTA update.
    esp_err_t err;           // Used to store the result of ESP-IDF API calls.

    if (received == 0)
    {
        // Get the next available partition for updating firmware.
        ota_partition = esp_ota_get_next_update_partition(NULL);

        // Start the OTA update process on the selected partition with a specified size.
        err = esp_ota_begin(ota_partition, fw_size, &ota_handle);
        if (err != ESP_OK) // Check if the operation was successful.
        {
            ESP_LOGE(TAG, "OTA begin failed: %s", esp_err_to_name(err));
            return; // Exit function on failure.
        }
    }

    // Write a chunk of data to the OTA partition.
    err = esp_ota_write(ota_handle, data, len);
    if (err != ESP_OK) // Check for errors during writing.
    {
        ESP_LOGE(TAG, "OTA write failed: %s", esp_err_to_name(err));
        return; // Exit function on failure.
    }

    received += len; // Update the total data written.

    // Check if all data has been received.
    if (received >= fw_size)
    {
        next_spp_data_is_update = false;

        err = esp_spp_disconnect(spp_connection_handle);
        if (err != ESP_OK) // Check for errors during hash calculation.
            ESP_LOGE(TAG, "Can't disconnect SPP");
        else
            ESP_LOGD(TAG, "SPP disconnected");

        // Finalize the OTA update process.
        err = esp_ota_end(ota_handle);
        if (err != ESP_OK) // Check for errors during finalization.
        {
            ESP_LOGE(TAG, "esp_ota_end(): %s", esp_err_to_name(err));
            return; // Exit function on failure.
        }

        // Get the description of the OTA partition to log metadata.
        esp_app_desc_t ota_desc;
        err = esp_ota_get_partition_description(ota_partition, &ota_desc);
        if (err != ESP_OK) // Check for errors during retrieval.
        {
            ESP_LOGE(TAG, "Can't get OTA partition description: %s", esp_err_to_name(err));
            return; // Exit function on failure.
        }
        else
        {
            // Log various metadata about the updated firmware.
            ESP_LOGI(TAG, "OTA version='%s'", ota_desc.version);
            ESP_LOGI(TAG, "OTA name='%s'", ota_desc.project_name);
            ESP_LOGI(TAG, "OTA build time='%s'", ota_desc.time);
            ESP_LOGI(TAG, "OTA build date='%s'", ota_desc.date);
            ESP_LOGI(TAG, "OTA idf version='%s'", ota_desc.idf_ver);
            ESP_LOGI(TAG, "OTA part type='%u'", ota_partition->type);
            ESP_LOGI(TAG, "OTA part subtype='%u'", ota_partition->subtype);
            ESP_LOGI(TAG, "OTA part size='0x%08x'", (unsigned int)ota_partition->size);
            ESP_LOGI(TAG, "OTA part address='0x%08x'", (unsigned int)ota_partition->address);
            ESP_LOG_BUFFER_HEX_LEVEL(TAG, ota_desc.app_elf_sha256, HASH_SIZE, ESP_LOG_INFO);
        }

        // Calculate the SHA-256 hash of the OTA partition using bootloader API.
        uint8_t part_hash_metadata[HASH_SIZE], part_hash[HASH_SIZE];
        err = bootloader_common_get_sha256_of_partition(ota_partition->address, fw_size, ota_partition->type, part_hash_metadata);
        if (err != ESP_OK) // Check for errors during hash calculation.
        {
            ESP_LOGE(TAG, "Can't get OTA partition hash: %s", esp_err_to_name(err));
            return; // Exit function on failure.
        }
        else
        {
            ESP_LOG_BUFFER_HEX_LEVEL("bootloader_common_get_sha256()", part_hash_metadata, HASH_SIZE, ESP_LOG_INFO);
        }

        // Calculate the SHA-256 hash of the OTA partition using ESP-IDF API.
        err = esp_partition_get_sha256(ota_partition, part_hash);
        if (err != ESP_OK) // Check for errors during hash calculation.
        {
            ESP_LOGE(TAG, "Can't get OTA APP hash: %s", esp_err_to_name(err));
            return; // Exit function on failure.
        }
        else
        {
            ESP_LOG_BUFFER_HEX_LEVEL("esp_partition_get_sha256()", part_hash_metadata, HASH_SIZE, ESP_LOG_INFO);
        }

        // Compare the two hashes to verify integrity of the firmware update.
        if (memcmp(part_hash_metadata, part_hash, HASH_SIZE) != 0)
        {
            ESP_LOGE(TAG, "Firmware hash mismatch");
            return; // Exit function on failure.
        }

        // Set the boot partition to the newly updated partition.
        esp_ota_set_boot_partition(ota_partition);

        // Log success and restart device after a short delay.
        ESP_LOGI(TAG, "Update successful, restarting...");
        ota_restart = 1;
        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for 1 second.
        esp_restart();                   // Restart the system to boot the new firmware.
    }
}

// Function that handles incoming Bluetooth Serial messages
static void process_bt_message(char *bt_message)
{
    // Echo the received message back to the sender.
    send_bt_message(bt_message);
    send_bt_message("\r");

    // Check if the next expected data is the firmware size.
    if (next_spp_data_is_size)
    {
        next_spp_data_is_size = false;          // Reset flag after processing the size.
        fw_size = strtol(bt_message, NULL, 10); // Convert received message to integer for firmware size.

        // Validate the firmware size: should be greater than 0 and less than 2MB.
        if (fw_size > 0 && fw_size < 2 * 1024 * 1000)
        {
            next_spp_data_is_update = true;                   // Set flag to expect firmware data next.
            send_bt_message("OK SEND FW in RAW BINARY\r\n>"); // Inform sender to start sending firmware data.
        }
        else
        {
            send_bt_message("ERR INVALID SIZE\r\n>"); // Notify sender of invalid size.
        }
        return; // Exit function after handling the size message.
    }

    // Check if the received message is "DBG1" to enable debugging mode.
    if (strcasecmp(bt_message, "DBG1") == 0)
    {
        debug_enabled = true;                  // Enable debugging flag.
        esp_log_level_set("*", ESP_LOG_DEBUG); // Set log level to DEBUG in ESP logging system.
        send_bt_message("OK\r\n");             // Acknowledge the command with OK.
        ESP_LOGI(TAG, "Debugging enabled");    // Log the enabling of debugging.
    }
    // Check if the received message is "DBG0" to disable debugging mode.
    else if (strcasecmp(bt_message, "DBG0") == 0)
    {
        debug_enabled = false;                 // Disable debugging flag.
        ESP_LOGI(TAG, "Debugging disabled");   // Log the disabling of debugging.
        esp_log_level_set("*", ESP_LOG_ERROR); // Set log level to ERROR in ESP logging system.
        send_bt_message("OK\r\n");             // Acknowledge the command with OK.
    }
    // Check if the received message is "FWUPDATE" to initiate firmware update process.
    else if (strcasecmp(bt_message, "FWUPDATE") == 0)
    {
        send_bt_message("OK SEND FW SIZE in BYTES\r\n"); // Inform sender to send firmware size next.
        ESP_LOGW(TAG, "Going to FW Update Mode");        // Log that the device is entering firmware update mode.
        next_spp_data_is_size = true;                    // Set flag to expect firmware size next.
    }
    else
    {
        // If the received message does not match any known commands, send a "?" (unknown command) response.
        send_bt_message("?\r\n");
        ESP_LOGI(TAG, "Received BT message: '%s'", bt_message); // Log the unknown message for debugging purposes.
    }

    // Send prompt character ">" to indicate readiness for receiving the next command from sender.
    send_bt_message(">");
}

// Function that send messages back to BT Serial
static void send_bt_message(char *response)
{
    // Check if there is an active SPP (Serial Port Profile) connection.
    // If the connection handle is 0, it means there is no active connection.
    if (spp_connection_handle == 0)
    {
        ESP_LOGE(TAG, "No active SPP connection");
        return;
    }

    // Attempt to send the response data over the SPP connection.
    // The function esp_spp_write writes data to the specified SPP connection handle.
    esp_err_t write_result = esp_spp_write(spp_connection_handle, strlen(response), (uint8_t *)response);

    // Check if the write operation was successful.
    if (write_result != ESP_OK)
    {
        // If writing failed, log an error message.
        ESP_LOGE(TAG, "Failed to send data via SPP");
    }
    else
    {
        // If writing succeeded, log a debug message with the sent response.
        ESP_LOGD(TAG, "Sent response: %s", response);
    }
}

// Function that trim incoming BT Serial messages
static void trim_whitespace(char *str)
{
    // Check if the input string is NULL or empty. If so, return immediately.
    if (str == NULL || *str == '\0')
        return;

    // Pointer to the start of the string, used to skip leading whitespace characters
    char *start = str;
    while (isspace((unsigned char)*start))
    {
        start++;
    }

    // If only whitespace characters were found and the new start is at the end of the string,
    // set the original string to be empty.
    if (*start == '\0')
    {
        *str = '\0';
        return;
    }

    // Pointer to the end of the string, used to trim trailing whitespace characters
    char *end = start + strlen(start) - 1;
    while (end > start && isspace((unsigned char)*end))
    {
        *end = '\0'; // Overwrite the current character with a null terminator
        end--;       // Move the end pointer backward
    }

    // If there are leading whitespace characters to remove,
    // move the trimmed string content to the beginning of the buffer.
    if (start != str)
    {
        memmove(str, start, strlen(start) + 1);
    }
}

// Initialize Bluetooth module
static void bt_init()
{
    esp_err_t ret; // Variable to store the return value of functions

    esp_bt_controller_mem_release(ESP_BT_MODE_BLE);

    // Initialize the Bluetooth controller with default configuration
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));

    // Enable the Bluetooth controller in classic Bluetooth mode
    ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "BT controller enable failed: %d", ret); // Log error if enabling fails
        return;
    }

    // Initialize Bluedroid stack
    ret = esp_bluedroid_init();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Bluedroid init failed: %d", ret); // Log error if initialization fails
        return;
    }

    // Enable Bluedroid stack
    ret = esp_bluedroid_enable();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Bluedroid enable failed: %d", ret); // Log error if enabling fails
        return;
    }

    // Set the device name for Bluetooth
    ret = esp_bt_gap_set_device_name("ESP32_BT_OTA-test");
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set device name: %d", ret); // Log error if setting device name fails
        return;
    }

    // Set the scan mode of the Bluetooth device to connectable and general discoverable
    ret = esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set scan mode: %d", ret); // Log error if setting scan mode fails
        return;
    }

    // Register the callback function for SPP events
    ret = esp_spp_register_callback(sppCallback);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "SPP register callback failed: %d", ret); // Log error if registration fails
        return;
    }

    // Configure SPP mode and other settings
    esp_spp_cfg_t bt_spp_config = {
        .mode = ESP_SPP_MODE_CB,    // Use callback-based communication
        .enable_l2cap_ertm = false, // Disable L2CAP ERTM (Enhanced Retransmission Mode)
        .tx_buffer_size = 0,        // Use default buffer size for transmission
    };

    // Initialize the SPP module with the configured settings
    ret = esp_spp_enhanced_init(&bt_spp_config);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "SPP init failed: %d", ret); // Log error if initialization fails
        return;
    }

    // Start an SPP server with the specified security level and role
    ret = esp_spp_start_srv(ESP_SPP_SEC_AUTHENTICATE, ESP_SPP_ROLE_SLAVE, 0, "ESP32_OTA");
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to start SPP server: %d", ret); // Log error if starting server fails
        return;
    }
}