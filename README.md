# Bracelet Firmware

This repository is part of the thesis project:  
**Design of an End-to-End IoT System for Monitoring Vulnerable Users**

It contains the firmware for a battery‑efficient LTE‑M wearable used for fall detection and real‑time localization.  
The complete system consists of three software components:

1. **Cloud Python Server**
2. **Flutter Android Application for Caretakers**  
   Repository: https://github.com/andreiFarcas/caregiver_app
3. **Firmware for the Bracelet Tracker**  
   (Current repository)

---

## Repository Structure

Each folder in this repository is a standalone Zephyr application.  
All applications were built using **Nordic Connect SDK v3.3.0**.

The full **firmware used in the final prototype** is located in: [**final_project/**](./final_project/)

All other folders contain smaller Zephyr projects used to validate individual PCB components:
	- [Barometer](./barometer/)
	- [Bosch bmi270 IMU](./accelerometer/)
	- [LTE](./lte_pcb/)
	- [Wi-Fi chip and Wi-Fi scanning](./wifi/)
	- [Flash Memory](./memory/) 
	- [Emergency Button, LED's, Buzzer](./button_led_buzzer/)
	- [GNSS](./gnss/)
	- [ML Inference testing](./test_model/)
	- [Power Supply Testing](./power/)
	- [Data Collection for ML Training](./new_data_logger/)
	- [Testing and Power Profiling of Final Project](./final_project_tests/)

These applications were used during initial hardware testing and iterative development.

---

### Final Project Tests 
They have to be ran with one of the following extra CMake arguments in order to select the specific test needed. 
The arguments can be manually added or set during the build configuration step in Connect SDK.

**ML vs Threshold fall detection models:**
```sh
detector_power -DEXTRA_CONF_FILE="overlay-detector-power.conf"
detector_accuracy -DEXTRA_CONF_FILE="overlay-detector-accuracy.conf"
```

**LTE on/off vs PSM:**
```sh
power_test_modem_off  -DEXTRA_CONF_FILE=overlay-power-test.conf
power_test_psm -DEXTRA_CONF_FILE=overlay-power-test.conf -DCONFIG_APP_POWER_TEST_PSM=y
```

**GNSS vs AGNSS:**
```sh
assisted_gnss_test -DEXTRA_CONF_FILE=overlay-gnss-test.conf -DCONFIG_APP_GNSS_TEST_ASSISTED=y
gnss_fix_power -DEXTRA_CONF_FILE=overlay-gnss-test.conf 
```