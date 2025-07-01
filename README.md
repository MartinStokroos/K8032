# K8032  
An STM32F103 Blue Pill-based replica of the Velleman K8055 USB interface card.  

## K8055  
The K8055 from Velleman Inc. is a well-known USB interface card for PCs and has been available for over 20 years. A DLL for both 32-bit and 64-bit Windows can be downloaded from the Velleman website, allowing the card to be used with the GUI demo application or API.  

## Features of the K8032  
- The K8032 operates at high device bus speed instead of low. Verify that resistor R10 on the Blue Pill is 1.5kΩ. If R10 is 10kΩ, connect a 1.8kΩ resistor between pin 1 (3.3V) and pin 12 (A12).  
- The PWM frequency is: ..  

## Pinout  

(To do: add device ID)  

## Required Components  
To function as a full replacement for the K8055 board, the STM32F103 Blue Pill requires additional circuitry, including:  

- An 8-line digital output buffer/level converter.
- 5 lines of 5V-tolerant digital input buffers.  
- Two low-pass filters for analog PWM outputs.  
- Two analog input amplifier/buffers.  

**Important Note:** The STM32F103 Blue Pill operates with 3.3V logic I/O and accepts a maximum analog input voltage of 3.3V.  

## Acknowledgments
Thanks to (Richard Hull)[https://github.com/rm-hull/k8055] for doing the reverse engineering of the K8055 data packet protocol.
