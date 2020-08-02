#include "mmc.h"
#define LOG_LOCAL_LEVEL ESP_LOG_INFO
#include <esp_log.h>
#include <dirent.h>


#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"

static const char *TAG = "mmc";

// SSD READER
void ssdtest(void)
{
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024};
    sdmmc_card_t *card;
    const char mount_point[] = MOUNT_POINT;
    ESP_LOGI(TAG, "Initializing SD card");

    ESP_LOGI(TAG, "Using SDMMC peripheral");
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();

    // This initializes the slot without card detect (CD) and write protect (WP) signals.
    // Modify slot_config.gpio_cd and slot_config.gpio_wp if your board has these signals.
    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();

    // To use 1-line SD mode, uncomment the following line:
    slot_config.width = 1;

    // GPIOs 15, 2, 4, 12, 13 should have external 10k pull-ups.
    // Internal pull-ups are not sufficient. However, enabling internal pull-ups
    // does make a difference some boards, so we do that here.
    /*
    gpio_set_pull_mode(15, GPIO_PULLUP_ONLY);   // CMD, needed in 4- and 1- line modes
    gpio_set_pull_mode(2, GPIO_PULLUP_ONLY);    // D0, needed in 4- and 1-line modes
    gpio_set_pull_mode(4, GPIO_PULLUP_ONLY);    // D1, needed in 4-line mode only
    gpio_set_pull_mode(12, GPIO_PULLUP_ONLY);   // D2, needed in 4-line mode only
    gpio_set_pull_mode(13, GPIO_PULLUP_ONLY);   // D3, needed in 4- and 1-line modes
    */

    esp_err_t ret;
    ret = esp_vfs_fat_sdmmc_mount(mount_point, &host, &slot_config, &mount_config, &card);

    if (ret != ESP_OK)
    {
        if (ret == ESP_FAIL)
        {
            ESP_LOGE(TAG, "Failed to mount filesystem. "
                          "If you want the card to be formatted, set the EXAMPLE_FORMAT_IF_MOUNT_FAILED menuconfig option.");
        }
        else
        {
            ESP_LOGE(TAG, "Failed to initialize the card (%s). "
                          "Make sure SD card lines have pull-up resistors in place.",
                     esp_err_to_name(ret));
        }
        return;
    }

    // Card has been initialized, print its properties
    //sdmmc_card_print_info(stdout, card);

    // Stuff
    ESP_LOGI(TAG, "Listing root directory:");

    DIR *d;
    struct dirent *dir;
    d = opendir(MOUNT_POINT "/");
    if (d)
    {
        while ((dir = readdir(d)) != NULL)
        {
            ESP_LOGI(TAG,"%s", dir->d_name);
        }
        closedir(d);
    }

    // All done, unmount partition and disable SDMMC or SPI peripheral
    //    esp_vfs_fat_sdcard_unmount(mount_point, card);
    esp_vfs_fat_sdmmc_unmount();
    ESP_LOGI(TAG, "Card unmounted");
}