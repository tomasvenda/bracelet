# Bracelet Firmware

This repository is part of the thesis project:  
**Design of an End-to-End IoT System for Monitoring Vulnerable Users**

It contains the firmware for a battery‑efficient LTE‑M wearable used for fall detection and real‑time localization.  
The complete system consists of three software components:

1. **Cloud Python Server**
2. **Flutter Android Application for caretakers**  
   Repository: https://github.com/andreiFarcas/caregiver_app
3. **Firmware for the bracelet tracker**  
   (Current repository)

---

## Repository Structure

Each folder in this repository is a standalone Zephyr application.  
All applications were built using **Nordic Connect SDK v3.3.0**.

The full firmware used in the final prototype is located in: final_project/

All other folders contain smaller Zephyr projects used to validate individual PCB components, such as:
	- Barometer
	- IMU (accelerometer folder)
	- LTE connectivity
	- Wi-Fi chip and Wi-Fi scanning
	- Memory

These applications were used during hardware bring‑up and iterative development.

