#pragma once


typedef struct {
    // Common
    enum {
        DISPLAY_BOOT_PAGE,
        DISPLAY_MAIN_PAGE,
        DISPLAY_OTA_PAGE
    } page;
    
    TaskHandle_t task;

    // MainPage
    enum  { 
        GRBL_IDLE,
        GRBL_RUN,
        GRBL_HOLD,
        GRBL_JOG,
        GRBL_ALARM,
        GRBL_DOOR,
        GRBL_CHECK,
        GRBL_HOME,
        GRBL_SLEEP
    } status;
    float x,y,z;            // Current Position         MPos:
    float wco,yco,zco;      // Work Coordinate Offset   WCO:
    uint16_t bf,bfmax;      // Buffer State             Bf:
    uint32_t ln;            // Line Number              Ln:
    uint16_t fr;            // Feed Rate                F: o FS:
    uint16_t speed;         // Spindle Speed            FS:
    uint8_t pin;            // Pin State  (bitmode)     Pn:
    enum  {                 // Accessory state          A:
        SPINDLE_OFF,
        SPINDLE_CW,
        SPINDLE_CCW
    } spindle;
    bool flood;             // Accessory state          A:
    bool mist;              // Accessory state          A:
    // OTA Page
    uint8_t percentual;     

} info_display_handle_t;

esp_err_t initDisplay(void);
void bootDisplay(void);
void infoDisplay(void);
void otaDisplay(uint8_t perc);
//esp_err_t startInfoDisplay(void);
//void stopInfoDisplay(void);


