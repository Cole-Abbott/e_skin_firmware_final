# E-Skin Firmware Code Documentation
---


A comprehensive guide to the E-Skin firmware code, detailing its structure, functionality, and usage.

## Overview
The E-Skin firmware is designed to run on a PIC32 MZ EF microcontroller, enabling high-speed data acquisition from analog sensors and transmitting this data over USB. The firmware utilizes the built-in ADC capabilities of the microcontroller to sample data at a rate of 6.25 Msps, leveraging DMA for efficient data handling. The firmware uses MPLAB Harmony, the hardware abstraction layer for PIC32 microcontrollers. Harmony provides a graphical configurator to setup the peripherals, and generates the code to set the correct values into the registers, and provides functions to interact with the peripherals. This greatly simplifies development, as you do not have to search through the datasheet to figure out which registers need to be set to configure a peripheral. Harmony also provides a USB library, which greatly simplifys the USB code. 


## ADC (Analog to Digital converter)

### How it works
The purpose of an ADC is to convert an analog voltage to a digital value (basicaly measure the voltage). The circuitry required for this is somewhat complcated, and there are multiple architectures for ADCs. 

The PIC32 MZ EF has a Successive-approximation-register (SAR) ADC. SAR ADCs can be capable of up to 5Msps sampling rate and up to 18 bits of resolution, but the ADCs on the PIC32 have a 12 bit resolution with a 3.125Msps sampling rate in 12 bit mode. However, multiple ADCs can be interleaved by reading from different modules in succession. The SAR ADC works by conduncting a binary search to converge find the closest approximation of the input voltage. First, a sample-and-hold circuit captures the input voltage at one instant. Then a DAC (Digital to Analog converter) creates a reference voltage in the middle of the voltage range, and these 2 voltages are fed into a comparator. If the sample voltage is higher, the first result bit is set, and the reference voltage is increased. If the sample voltage is lower, the first bit is cleared and reference voltage is decreases. This process is repeated for each bit of resolution. This process takes some time, as the DAC and comparator have some settling time, and there is some logic required for each step. The time for each result to be ready determines the sampling rate of the ADC [More info on SAR ADCs Here](https://www.analog.com/en/resources/technical-articles/successive-approximation-registers-sar-and-flash-adcs.html)


 The PIC32MZ EF has 5 dedicated ADC modules (ADC0-4), and 1 shared module (ADC7). The dedicated modules are for high-speed and precise sampling, but can only be configured a primary or alternate pin. (alternate pins are not availalbe on the 64 pin version of the chip we will use in the final device). The shared ADC has a multiplexer and can be configured to many inputs, but is slower. Details of the PIC32MZ EF ADC can found in the ADC section of the [datasheet](https://ww1.microchip.com/downloads/aemDocuments/documents/MCU32/ProductDocuments/DataSheets/PIC32MZ-Embedded-Connectivity-with-Floating-Point-Unit-Family-Data-Sheet-DS60001320H.pdf)

In this project, we will use ADC2 and ADC4, since these are broken out on the dev board. We will run these 2 channels in interleaved mode at the full 12 bit resolution for a combined sampling rate of 6.25Msps. 

In order to collect this incoming data, we will use DMA (Direct Memory Access) to automatically move the data from the ADC result register into a buffer after every sample is collected. DMA allows peripherals to move data in and out of memory without the CPU doing anything, which is faster and frees up CPU time. When the ADC data is ready, it will trigger and interupt, and this interupt will trigger the DMA to transfer the ADC_DATA register into a buffer. In this case, the ADC interupt does not trigger an ISR (Interupt Service Routine), it just causes the DMA transfer. Once the DMA transfers a set amount of data, (ie, enough to fill up the buffer) another interupt will trigger to notify the program that the data is ready. 

Both ADCs require a trigger source, and must be in sync with each other. To do this we will use 2 more peripherals, a timer (TMR) and an output compare (OCMP). The timer will just be configured to have a frequency of 3.125Mhz, and this will trigger ADC2. We want ADC4 to trigger halfway between ADC2 trigger, so we can use a OCPM module to compare the value of the timer to a set value, and trigger ADC4 when they are equal. 



### Configuring the ADC in Harmony  
**Step 1:** Open the MCC (MPLAB Content Configurator) in MPLAB X IDE.

**Step 2:** In the "Device Resouces" tab, under "Harmony" and "Peripherals", select Drag ADCHS, TMR3, and OCMP3 into project graph.

**Step 3:** Configure the Pins: 
- ADC2: Set pin RB2 as AN2
- ADC4: Set pin RB4 as AN4

**Step 4:** Configure the ADCs:
- Set the clock source to "System Clock"
- For ADC2, set the following:
  - Enable: Yes
  - Resolution: 12 bits
  - Trigger Source: TMR3 Match

- For ADC4, set the following:
  - Enable: Yes
  - Resolution: 12 bits
  - Trigger Source: OCMP3
Leave the rest of the settings as default. Do not enable the interrupt, as this will enable and ISR for the ADC, which we do not want. We will manually enable the ADC interrupt in the code without enabling the ISR.

**Step 5:** Configure the TMR3:
- Use the 1:1 prescaler
- Set the clock to internal peripheral clock
- Set the Time to 320ns (3.125MHz)
    - This will set the period register to 30, based on the clock frequency

**Step 6:** Configure the OCMP3:
- Set the output mode to "Initalise OCx pin low, generate continuous pulses on OCx pin"
- Set the Timer source to Timer 3
- Set the compare value to 15, half of the period of TMR3, so that it triggers ADC4 halfway through the TMR3 period.
- Set the secondary compare value to 17, 2 cycles after the first value, so that it resets the output pin to low after the first pulse.

Note that there is no "output pin" for the OCMP3, but the pulse is used to trigger the ADC4.

**Step 7:** Configure the DMA:
- In the DMA configuration window, enable Channel 0, with trigger source set to "ADC2_DATA_2".

**Step 8:** Generate the code by clicking the "Generate" button in the MCC toolbar.

### ADC Code

#### Initialization

```c
ADCGIRQEN1bits.AGIEN2 = 1; //Enb ADC2 AN2 interrupts for DMA

//Setup DMA interupt callback and start a transfer
DMAC_ChannelCallbackRegister(DMAC_CHANNEL_0, ADC_2_ResultReadyCallback, 0);
DMAC_ChannelTransfer(DMAC_CHANNEL_0, ADC_SRC_ADDR_2, ADC_SRC_SIZE, adc_buf, sizeof (adc_buf), sizeof (uint32_t));
```


#### Callback Function
```c
static void ADC_2_ResultReadyCallback(DMAC_TRANSFER_EVENT event, uintptr_t contextHandle) {
    appData.adcDataReady = true;
}
```


