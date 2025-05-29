# E-Skin Firmware
==========================

This project implements firmware for the **E-Skin** system, utilizing a PIC32 MZ EF microcontroller to acquire high-speed ADC data and transmit it over USB.

## Features

- **High-Speed ADC Sampling:** Efficiently reads analog sensor data using 2 interleaved channels of PIC32 MZ EF's built-in ADC to sample at 6.25 Msps, using DMA to save the data to a buffer without the CPU.
- **USB Data Transmission:** Streams acquired data to a host computer via USB for further processing or visualization.


## Hardware

- **Microcontroller:** PIC32 MZ EF series. The PIC32 MZ EF curiosity 2.0 board is used for development, and the PIC32MZ2048EFH064 chip will be used in the final product.
- **USB Interface:** Standard USB connection to host PC

## Getting Started

1. **Clone the Repository**
    ```sh
    git clone https://github.com/yourusername/e_skin_firmware_final.git
    ```

2. **Open in MPLAB X IDE**
    - Import the project into MPLAB X IDE.
    - Ensure you have the correct compiler and device family selected.

3. **Build and Flash**
    - Connect your PIC32 MZ EF board.
    - Build the project and flash the firmware.

4. **Connect via USB**
    - The device will enumerate as a USB device.
    - Use the python scrips provieded in the `/pyusb_code` directory to read and visualize the data.

## Data Format

Data is sent as binary packets, containing 766 data samples, each sample being 2 bytes (16 bits). The data is interleaved from the two ADC channels.
The first 2 bytes of each packet are the packet header, which contain the channel number 1-32.

## Directory Structure

```
/src                        # Firmware source code
/e_skin_firmware_final.X    # MPLAB X project files
/pyusb_code                 # Python scripts for USB communication
/README.md                  # Project overview
/DOCS.md                    # Documentation for the firmware
```


## Authors

- Cole Abbott [@Cole-Abbott]

## Acknowledgements

- This project was developed as part of the E-Skin project at the Northwestern University under Shihan Lu and Kevin Lynch.
