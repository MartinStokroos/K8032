# K8032
Mimics the Velleman K8055 with the STM32F103 Blue Pill board.

## K8055
The K8055 from Velleman Inc. is a well known USB-interface card for the PC and it already exists for more than 20 years. A DLL for 32-bits and 64-bits Windows is available from the Velleman website to connect the card with the GUI-demo application or API. 

## Differences between the K8032 and the K8055
The K8032 connects with high device bus speed, instead of low. Check R10 from the Blue Pill if it is 1.5kOhms. If R10 is 10KOhms, place a 1.8kOhms resistor in parallel between pin 1 (3.3) and pin 12 (A12). 

## Pinout

