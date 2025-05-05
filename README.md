# Embedded Challenge Spring 2025 Term Project
## "Shake, Rattle, and Roll"

This project detects Parkinsonian tremor and dyskinesia using a on-board accelerometer and uses signal processing techniques to classify movement patterns.
This project is developed and run on STM32L4 Discovery kit for the IoT node (B-L475E-IOT01A).

## Features
- Detect tremor (3–5 Hz) and dyskinesia (5–7 Hz)
- Real-time signal filtering and classification

## Folder Structure
- `src/`: Source code of the program.

## Dependencies
- Visual Studio Code
- Platform IO Extension

## Getting Started
1. Connect the board to your PC.
2. (Windows only) Download driver from rb.gy/2njvp, unzip it, and Run stlink_winsub_install.bat as administrator. 
3. Click on "PlatformIO: Build" and "PlatformIO: Upload" button on VS Code.

## License
MIT License
