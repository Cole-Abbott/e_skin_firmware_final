# E-Skin Firmware
---

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
    git clone https://github.com/Cole-Abbott/e_skin_firmware_final.git
    ```

2. **Open in MPLAB X IDE**
    - Import the project into MPLAB X IDE.
    - Ensure you have the correct compiler and device family selected.
    - The project was developed using MPLAB X IDE v6.20, so it is recommended to use this version or later.

3. **Build and Flash**
    - Connect your PIC32 MZ EF board.
    - Build the project and flash the firmware.

4. **Connect via USB**
    - The device will enumerate as a USB device.
    - Use the python scrips provieded in the `/pyusb_code` directory to read and visualize the data.
    - Note: You may need to install the `pyusb` library for Python, as well as set the path to the libusb in the python scripts.
    - The usb_plot script alse requires `pyqtgraph` for visualization.

5. **Build Circuit**
    - For the PIC32 MZ EF curiosity 2.0 board, you can use the following connections:
        - ADC Channel 2: Pin RB2 (AN2), on the Xplained Pro Extension Standard Header, pin 3. (top row, second from the top when the board is oriented with the USB port on the left)
        - ADC Channel 4: Pin RB4 (AN4), on the Arduino Extension Standard Header, pin A0
        - Trigger Signal: Pin RA1, on the Arduino Extension Standard Header, pin A1
        - Ground: Various ground pins, such as Xplained Pro Extension Standard Header, pin 2, (bottom row top pin)

## Data Format

Data is sent as binary packets, containing 766 data samples, each sample being 2 bytes (16 bits). The data is interleaved from the two ADC channels.
The first 2 bytes of each packet are the packet header, which can contain metadata such as the channel number, but this has not yet been implemented in the firmware. The remaining 1532 bytes contain the ADC samples.

## Directory Structure

```
/src                        # Firmware source code
/e_skin_firmware_final.X    # MPLAB X project files
/pyusb_code                 # Python scripts for USB communication
/README.md                  # Project overview
/DOCS.md                    # Documentation for the firmware
```

## Resources
- [PIC32 MZ EF Family Reference Manual](https://ww1.microchip.com/downloads/aemDocuments/documents/MCU32/ProductDocuments/DataSheets/PIC32MZ-Embedded-Connectivity-with-Floating-Point-Unit-Family-Data-Sheet-DS60001320H.pdf)
- [Curiosity PIC32MZ EF 2.0 Development Board User's Guide](https://ww1.microchip.com/downloads/aemDocuments/documents/MCU32/ProductDocuments/UserGuides/PIC32MZ-EF-2.0-Development-Board-Users-Guide-DS70005400.pdf)
- [High Speed ADC Sampling Application Note](https://ww1.microchip.com/downloads/aemDocuments/documents/OTH/ApplicationNotes/ApplicationNotes/Worlds-Fastest-Embedded-Interleaved-12-bit-ADC-Using-PIC32MZ-and-PIC32MK-Families-DS00002785A.pdf)
- [Older but simple PIC32MZ EF Setup video in Harmony](https://www.youtube.com/watch?v=sW-yS2FHI54)
- [Recent Ish Youtube Tutorial on PIC2 MZ EF Setup in Harmony](https://www.youtube.com/watch?v=Z-8srAI8jow)


## Authors

- Cole Abbott [@Cole-Abbott]

## Acknowledgements

- This project was developed as part of the E-Skin project at the Northwestern University under Shihan Lu and Kevin Lynch.
