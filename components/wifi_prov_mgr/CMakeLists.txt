idf_component_register(SRCS "csrc/wifi_prov_mgr.c"
                    INCLUDE_DIRS "csrc"
                    REQUIRES freertos nvs_flash wifi_provisioning
                    )