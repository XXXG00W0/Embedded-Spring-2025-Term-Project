# Embedded Challenge Spring 2025 Term Project
## "Shake, Rattle, and Roll"

### Project Members
Clarence Luo (cl5672), Hongyu Jin (hj2806), Kailin Zhang (kz2739), Yumeng Qian (yq2480), Ziyi Liang (zl5604)

## Project Description

This project detects Parkinsonian tremor and dyskinesia using a on-board accelerometer and  signal processing techniques to classify movement patterns. The program is capable of bluetooth low energy (BLE) communication with mobile devices (via LightBlue or similar apps). This project is developed and run on STM32L4 Discovery kit for the IoT node (B-L475E-IOT01A).

## Features
- Detect tremor (3–5 Hz) and dyskinesia (5–7 Hz)
- Real-time signal filtering and classification
- Bluetooth Low Energy (BLE) communication

## Folder Structure
- `src/`: Source code of the program.
- `lib/`: External libraries used in the project.

## Dependencies
- Visual Studio Code
- Platform IO Extension
- CMSIS-DSP library

## Getting Started
1. Connect the board to your PC.
2. (Windows only) Download driver from rb.gy/2njvp, unzip it, and Run stlink_winsub_install.bat as administrator. 
3. Click on "PlatformIO: Build" and "PlatformIO: Upload" button on VS Code.

## License
MIT License
