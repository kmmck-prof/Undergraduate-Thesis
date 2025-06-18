# Undergraduate-Thesis
Development of a Real-Time, Indoor Air Quality Monitoring System for Public Utility Buses With Wi-Fi-Based Occupancy Counting

Authors: 
Kyle Matthew Cayanana§, Franz Gabriel De Leona§, Rian Jherico Virtucioa§,Neil Christian Astrologoa, John Jairus Eslita, Percival Magpantaya, Julius Lustrob, Joshua Agarc, Marc Rosalesa, John Richard Hizona

Writter Under: 
Electrical and Electronics Engineering Institute, University of the Philippines Diliman, Quezon City, Philippines  
Department of Mechanical Engineering, University of the Philippines Diliman, Quezon City, Philippines  
Institute of Civil Engineering, University of the Philippines Diliman, Quezon City, Philippines

Written On:
April 2024

# Features

Mesh Network of Multiple ESP32 Microcontrollers
1) Zigbee Mesh
2) Thread Mesh
3) Bluetooth Mesh

Nearby Occupant Counter
- Uses WiFi Receiver to detect the MAC Address of all nearby Mobile Devices and Computers. Each unique MAC Address is stored and considered a single passenger. 

# List of References

EDKs/SDKs Used
1. ESP-IDE w/ ESP-IDF (https://dl.espressif.com/dl/esp-idf/)

Libraries Used
1. ESP-Zigbee-SDK

- https://github.com/espressif/esp-zigbee-sdk 

2. Wi-Fi MAC Sniffer 
- https://www.hackster.io/p99will/esp32-wifi-mac-scanner-sniffer-promiscuous-4c12f4 
- https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/network/esp_wifi.html?highlight=wifi_promiscuous_pkt_t#_CPPv422wifi_promiscuous_pkt_t 

3. Openthread C API
- https://github.com/openthread/openthread/tree/main/include/openthreadESP-BLE-MESH API https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/bluetooth/esp-ble-mesh.html#api-reference
