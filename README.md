# Walkie Talkie Project

A wireless voice communication system built using ESP32 microcontrollers.

## Features
- Wireless audio transmission and reception using NRF24L01 radio modules
- ADPCM compression for efficient voice data transfer
- AES encryption for secure communication
- Custom audio tones and startup sounds stored on flash memory
- Simple TX/RX mode switching with a physical button

## Hardware
- ESP32 (upesy_wroom)
- NRF24L01+ 2.4GHz Transceiver
- PCM5102 I2S DAC
- Analog microphone
- Push button and status LED

## How to use
1. Flash the code onto two ESP32 boards.
2. Power them up to hear the boot sound.
3. Use the switch to select transmit mode.
4. Speak into the microphone to transmit audio to the other device.
5. Switch to receive mode to listen for incoming transmissions.

## Building
This project is built using PlatformIO. Open it in Visual Studio Code with the PlatformIO extension and upload to your ESP32 boards.

## Notes
Developed as an educational project to explore wireless communications, audio processing, and embedded systems.