# Secure Optical Data Diode using VLC for IoT

This project is a secure, one-way communication system that transmits sensor data using Visible Light Communication (VLC). The system is designed for environments where RF communication (like Wi-Fi or Bluetooth) is forbidden, providing a secure method to exfiltrate data from a sealed environment to a cloud IoT platform for remote monitoring.

---

## Features
- **Secure Data Transmission:** Uses a line-of-sight optical link, making it immune to RF snooping.
- **Real-Time Monitoring:** Data is sent to an MQTT broker for instant remote visualization.
- **Robust Protocol:** Implements a self-synchronizing Pulse Position Modulation (PPM) protocol, removing the need for perfectly matched clocks.
- **Hybrid Firmware:** Uses an STM32 for real-time transmission and an ESP32 for high-level networking tasks.

---

## System Architecture

1.  The **STM32 Transmitter** reads temperature and humidity from a DHT sensor.
2.  It serializes this data and modulates a high-power LED using a PPM protocol.
3.  The **ESP32 Receiver**, using a photodiode and a transimpedance amplifier, detects the light pulses.
4.  It demodulates the signal to reconstruct the original data.
5.  The ESP32 connects to Wi-Fi, formats the data as a JSON packet, and **publishes** it to an MQTT broker.
6.  Any MQTT client subscribed to the topic can now view the data in real-time.

---

## Hardware Components

**Transmitter:**
- STM32F411 "Black Pill"
- DHT11 Sensor
- High-Brightness LED
- IRLZ44N MOSFET
- SSD1306 OLED Display

**Receiver:**
- ESP32 DevKit V1
- BPW34 Photodiode
- LM358 Op-Amp
- SSD1306 OLED Display

A detailed schematic for the transmitter & receiver circuits can be found in the `/hardware` folder.

---

## Software & Setup

The firmware is located in the `/firmware` folder.

1.  **Transmitter:** The project was built using **STM32CubeIDE**. Open the project located in `/firmware/stm32_transmitter/` and flash to the STM32 board.
2.  **Receiver:** The code was written for the **Arduino IDE**.
    - Install the ESP32 board manager.
    - Install the following libraries from the Library Manager:
        - `Adafruit_SSD1306`
        - `Adafruit_GFX`
        - `PubSubClient` by Nick O'Leary
    - Open `esp32_receiver.ino`, update your Wi-Fi credentials and a unique `mqtt_client_id`, and upload it to the ESP32.

---

## Authors

- **[Ushan Karunarathna](https://www.linkedin.com/in/ushan-karunarathna)** - ESP32 & Cloud Lead
- **[Sasindu Perera](https://www.linkedin.com/in/sasinduperera)** - Hardware Lead & System Integration
- **[Madhuka Dias](https://www.linkedin.com/in/madhuka-dias-5a5794293)** - STM32 Firmware Lead 
---

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
