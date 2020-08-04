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
        GRBL_UNKNOWN,
        GRBL_IDLE,
        GRBL_RUN,
        GRBL_HOLD,
        GRBL_JOG,
        GRBL_ALARM,
        GRBL_DOOR,
        GRBL_CHECK,
        GRBL_HOME,
        GRBL_SLEEP,
    } status;
    bool status_blink;
    uint32_t lastStatusUpdate;
    float x,y,z;            // Current Position         MPos:
    float xco,yco,zco;      // Work Coordinate Offset   WCO:
    uint16_t bf,bfmax;      // Buffer State             Bf:
    uint32_t ln;            // Line Number              Ln:
    uint16_t fr;            // Feed Rate                F: o FS:
    uint16_t speed;         // Spindle Speed            FS:
    bool    limitXYZ[3];    // Pin State                Pn:XYZ
    bool    limitP;         // Pin State                Pn:P
    bool    pinDoor;        // Pin State                Pn:D
    bool    pinHold;        // Pin State                Pn:H
    bool    pinReset;       // Pin State                Pn:R
    bool    pinStart;       // Pin State                Pn:S
    uint8_t ofeed,orapids,ospeed; // Override value     Ov:
    enum  {                 // Accessory state          A:
        SPINDLE_OFF,
        SPINDLE_CW,
        SPINDLE_CCW
    } spindle;
    bool flood;             // Accessory state          A:
    bool mist;              // Accessory state          A:
    // OTA Page
    uint8_t percentual;  
    // MAIN PAGE
    bool tcp_serial;
    bool bluetooth_serial;   
    bool wifi;
    bool webcam;

} info_display_handle_t;
extern info_display_handle_t info_display_handle;

esp_err_t initDisplay(void);
void bootDisplay(void);
void infoDisplay(void);
void otaDisplay(uint8_t perc);
//esp_err_t startInfoDisplay(void);
//void stopInfoDisplay(void);


