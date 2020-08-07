# TODO
## ![0%](https://progress-bar.dev/0) 1.0 RC

### ![0%](https://progress-bar.dev/0) HW TODO:
 -  [ ] Put on a real PCB

## ![0%](https://progress-bar.dev/0) 1.0 Beta

### ![0%](https://progress-bar.dev/0) HW TODO:
 -  [ ] Test on real Grbl
 -  [ ] Clean Code - Migrate to C++


## ![0%](https://progress-bar.dev/0) 1.0 Alpha

### ![90%](https://progress-bar.dev/90) HW TODO: 
 -  [x] Display
 -  [x] WiFi
 -  [ ] Bluetooth serial
 -  [x] Joystick
 -  [x] Buttons
 -  [x] Cam
 -  [x] HiMem (8Gb)
 -  [ ] Beeper
### ![90%](https://progress-bar.dev/90)  SW TODO: 
 -  [x] Camera streaming 
 -  [x] WiFi configuration (SoftAp provisioning)
   - [ ] Add Display Information on self provisioning
      - [ ] Add QRCode for provisioning (see code for ESP2S app)
 -  [x] Display on own task (autorefresh)
      - [ ] Refactor Display to C++ objects
 -  [x] WiFi Socket serial
 -  [ ] Icon Menu
 -  [x] OTA
    -  [x] Correct watchdog conflict
    -  [x] Ota events!
 -  [x] mDNS
### ![10%](https://progress-bar.dev/10)  GRBL TODO: [Grbl document](https://github.com/gnea/grbl/wiki/Grbl-v1.1-Interface#interacting-with-grbls-systems)
 -  [x] Log out of serial! 
    - [x] GRBL compatible comment? ()
    - [x] UDP log
 -  [ ] Display status
    - [x] Status Parser
    - [x] Display mode (Idle..)
    - [x] MPos
    - [ ] WPos
 -  [ ] Grbl version parser
    - [ ] Display Info Panel   
 -  [ ] Manual command / Jogging [Grbl document](https://github.com/gnea/grbl/wiki/Grbl-v1.1-Jogging)
    - [ ] Joystick Calibrator 
 -  [ ] Probe command
 -  [ ] Realtime commands  [Grbl document](https://github.com/gnea/grbl/wiki/Grbl-v1.1-Jogging)
 -  [ ] Homing [Grbl document](https://github.com/gnea/grbl/wiki/Set-up-the-Homing-Cycle)