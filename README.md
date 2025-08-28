# Autonomous Boat Navigation System

A comprehensive autonomous surface vehicle (USV) control system featuring GPS-guided navigation with real-time Android app control interface. This project demonstrates practical implementation of smart systems for rescue operations and maritime autonomous navigation.

## 🚢 Overview

This project consists of two main components:
- **Android Application**: Provides map interface for setting waypoints and monitoring boat position
- **ESP8266 Controller**: Handles autonomous navigation using GPS and IMU sensors

The system enables precise waypoint navigation with real-time position tracking, making it suitable for rescue operations, environmental monitoring, and autonomous maritime tasks.

## ✨ Features

- **GPS-based Navigation**: Precise positioning using NEO-6M GPS module
- **Compass Heading Control**: MPU6050 with DMP for drift-free heading
- **PID Control System**: Smooth and stable steering control
- **Real-time Communication**: WiFi-based coordination with Android ground station
- **Interactive Map Interface**: OpenStreetMap integration for waypoint selection
- **Live Position Tracking**: Real-time boat position and heading display
- **Autonomous Operation**: Self-guided navigation to target coordinates
- **Safety Features**: Multiple failsafes and error handling mechanisms

## 🛠️ Hardware Requirements

### ESP8266 Controller
- ESP8266 NodeMCU or Wemos D1 Mini
- NEO-6M GPS Module
- MPU6050 6-Axis IMU (with DMP support)
- Servo Motor (for steering)
- DC Motor (for propulsion)
- Jumper wires and breadboard/PCB

### Boat Platform
- Suitable waterproof enclosure
- Marine-grade servo for rudder control
- Propulsion motor with propeller
- Battery pack (LiPo recommended)
- Rudder and propeller assembly

## 📱 Software Requirements

### Android Application
- Android Studio Arctic Fox or later
- Minimum SDK: API 21 (Android 5.0)
- Target SDK: API 33 (Android 13)

### ESP8266 Firmware
- Arduino IDE 1.8.x or 2.x
- ESP8266 Board Package 3.0.x
- Required Libraries (see Installation section)

## 🔧 Installation

### ESP8266 Setup

1. **Install Arduino Libraries:**
   ```
   - ESP8266WiFi
   - ESP8266HTTPClient
   - TinyGPSPlus
   - MPU6050 (by Electronic Cats)
   - I2Cdev
   - Servo (ESP32/ESP8266)
   ```

2. **Hardware Connections:**
   ```
   ESP8266    |    Component
   ---------- | --------------
   GPIO15     |    Servo Signal
   GPIO12     |    Motor Control
   GPIO13     |    MPU6050 INT
   GPIO0      |    GPS RX
   GPIO14     |    GPS TX
   3V3        |    MPU6050 VCC, GPS VCC
   GND        |    Common Ground
   D1/D2      |    MPU6050 SDA/SCL
   ```

3. **Configuration:**
   - Update WiFi credentials in the ESP8266 code
   - Adjust server IP address to match your Android device
   - Calibrate MPU6050 offsets if needed
   - Tune PID parameters for your boat's characteristics

### Android Application

1. **Build and Install:**
   ```bash
   git clone https://github.com/yourusername/autonomous-boat-navigation.git
   cd autonomous-boat-navigation/android
   ./gradlew assembleDebug
   adb install app/build/outputs/apk/debug/app-debug.apk
   ```

2. **Permissions:**
   - Network access for communication with boat
   - Location services (for map centering)

## 🚀 Usage

### Quick Start

1. **Power up the boat controller**
2. **Launch the Android application**
3. **Wait for "USV Connected" message**
4. **Tap "Set Destination" to open the map**
5. **Tap on the map to set a waypoint**
6. **Watch the boat navigate autonomously**

### Operation Modes

- **Manual Control**: Use Android app to send individual waypoints
- **Autonomous Navigation**: Boat follows GPS coordinates with PID steering control
- **Monitoring**: Real-time position and heading feedback on Android map

### Safety Features

- **Destination Threshold**: Boat stops when within 2 meters of target
- **Connection Monitoring**: Fails safely if communication is lost
- **Hardware Limits**: Servo angles constrained to prevent damage
- **Watchdog Protection**: System reset protection during operation

## 📊 Technical Specifications

- **Navigation Accuracy**: ±2 meters (GPS dependent)
- **Update Rate**: 20Hz control loop, 0.2Hz position updates
- **Communication Range**: WiFi range (typically 50-100m)
- **Operating Voltage**: 5V-12V (depending on motors)
- **Heading Accuracy**: ±2° (MPU6050 DMP)

## ⚙️ Configuration

### PID Tuning
Adjust these parameters in the ESP8266 code for optimal performance:
```cpp
float pidKp = 2.0;  // Proportional gain
float pidKi = 0.1;  // Integral gain  
float pidKd = 0.5;  // Derivative gain
```

### Network Settings
```cpp
const char* ssid = "YourWiFiNetwork";
const char* password = "YourPassword";
const char* server = "http://192.168.1.100:8080";  // Android device IP
```

## 🐛 Troubleshooting

### Common Issues

**GPS not getting fix:**
- Ensure clear sky view
- Check GPS module connections
- Verify baud rate (9600)

**MPU6050 connection failed:**
- Check I2C connections (SDA/SCL)
- Verify 3.3V power supply
- Ensure pull-up resistors on I2C lines

**WiFi connection issues:**
- Verify network credentials
- Check signal strength
- Ensure Android device is on same network

**Boat not responding to commands:**
- Check servo and motor connections
- Verify power supply adequacy
- Monitor serial output for errors

## 🤝 Contributing

Contributions are welcome! Please feel free to submit issues, fork the repository, and create pull requests.

1. Fork the Project
2. Create your Feature Branch (`git checkout -b feature/AmazingFeature`)
3. Commit your Changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the Branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 🙏 Acknowledgments

- TinyGPS++ library for GPS parsing
- MPU6050 DMP library for motion processing
- OpenStreetMap for mapping services
- ESP8266 Arduino Core community

## 📞 Support

For questions and support:
- Create an issue on GitHub
- Check the troubleshooting section
- Review the code comments for implementation details

---

**⚠️ Safety Notice**: This system is designed for educational and research purposes. Ensure proper safety measures when operating autonomous vehicles in water. Always maintain visual contact and have manual override capabilities available.
