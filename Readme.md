# YDLiDar_gs2 Library for GS2 Static LiDAR

## OVERVIEW
The `YDLiDar_gs2` library provides an easy-to-use interface for integrating the YDLiDar GS2 Static LiDAR with Arduino-compatible platforms. It allows users to easily configure and control the GS2 LiDAR, enabling real-time distance measurement, scanning, and data retrieval. The library supports multiple connected LiDAR devices, offering flexible options for baud rate configuration, edge mode settings, and calibration. With built-in error handling, it ensures smooth communication with the LiDAR, providing developers with precise and reliable measurements for applications in robotics, automation, and mapping. This library abstracts complex communication protocols and device-specific settings, making it simple for users to integrate and utilize the GS2 LiDAR in their projects.

## Table of Contents
- [Features](#features)
- [Installation](#installation)
- [Getting Started](#getting-started)
- [Functions](#functions)
- [Error Codes](#error-codes)
- [Notes](#notes)
- [References](#references)
- [License](#license)

## Features
- **Multi-device support**: Control multiple GS2 LiDARs connected to a single microcontroller.
- **Measurement retrieval**: Access individual or aggregated scans with adjustable parameters.
- **Configurable communication**: Set baud rates, calibration coefficients, and edge modes.
- **Error handling**: Functions return specific error codes for improved debugging.
- **Platform compatibility**: Works with ESP32 and compatible Arduino boards (excludes AVR-based boards like Arduino UNO due to dependency on `<map>` library).

## Installation
1. Download or clone this repository.
2. Include the library in your Arduino project:
    ```cpp
    #include "YDLiDar_gs2.h"
    ```

## Getting Started

### Hardware Setup
Connect each GS2 LiDAR’s TX and RX to the designated serial port on your microcontroller.

### Basic Usage

1. **Initialize the YDLiDar_gs2 Object**
    ```cpp
    #include "YDLiDar_gs2.h"

    HardwareSerial* YDSerial = &Serial1; // Choose the serial port
    YDLiDar_gs2 lidar(YDSerial, GS_LIDAR_BAUDRATE_921600);
    ```

2. **Initialize the LiDAR**
    ```cpp
    GS_error status = lidar.initialize();
    if (status == GS_OK) {
        // Initialization successful
    } else {
        // Handle initialization error
    }
    ```

3. **Start Scanning**
    ```cpp
    if (lidar.startScanning() == GS_OK) {
        // Scanning started
    }
    ```

4. **Retrieve:**
- A Single Measurement:
    ```cpp
    iter_Measurement measurement = lidar.iter_measurments();
    if (measurement.valid) {
        // Access angle, distance, quality, etc.
    }
    ```
- Multiple Measurements:
    ```cpp
    iter_Scan scan = lidar.iter_scans();
    if (measurement.valid) {
        // Access angle, distance, quality, etc.
    }
    ```

5. **Stop Scanning**
    ```cpp
    lidar.stopScanning();
    ```

### Additional Configuration

- **Set Baud Rate**:
    ```cpp
    lidar.setBaudrate(GS_LIDAR_BAUDRATE_921600);
    ```

- **Set Edge Mode**:
    ```cpp
    lidar.setedgeMode(EDGE_MODE_STANDARD, 0x01); // Set edge mode for specific LiDAR
    ```

## Functions

### Core Functions

| Function               | Description                                                              |
|------------------------|--------------------------------------------------------------------------|
| `initialize()`         | Sets up communication between the LiDAR and the microcontroller.         |
| `startScanning()`      | Begins scanning on the connected LiDARs.                                 |
| `stopScanning()`       | Stops scanning on the connected LiDARs.                                  |
| `iter_measurments()`   | Retrieves a single measurement (angle, distance, quality).               |
| `iter_scans()`         | Retrieves all measurements from the most recent scan.                    |
| `setBaudrate(uint8_t)` | Sets the baud rate for serial communication.                             |
| `setNumberofLiDars(int)` | Specifies the number of connected LiDAR devices.                       |

### Utility Functions

| Function            | Description                                           |
|---------------------|-------------------------------------------------------|
| `getParametes()`    | Retrieves calibration coefficients for a specific LiDAR. |
| `getVersion()`      | Retrieves version information for the connected LiDARs. |
| `getNumberOfDevices()` | Returns the count of connected LiDAR devices.     |
| `softRestart()`     | Soft-restarts the connected LiDAR(s).                  |

## Error Codes

The library provides specific error codes for debugging:

- **GS_OK**: Operation successful.
- **GS_NOT_OK**: Operation failed.

## Notes
- **HardwareSerial Dependency**: The library requires `HardwareSerial` for communication. Ensure the specified serial port is exclusive to the LiDAR.
- **Arduino UNO Limitation**: The library is currently incompatible with the Arduino UNO due to limitations with `<map>`.

## References
### Datasheets:
- Website: https://www.ydlidar.com/service_support/download.html
- Datasheet: https://www.ydlidar.com/Public/upload/files/2023-10-17/YDLIDAR%20GS2%20Data%20Sheet%20V1.9(231009).pdf
- User Manual: https://www.ydlidar.com/dowfile.html?cid=11&type=2
- Development Manual: https://www.ydlidar.com/dowfile.html?cid=11&type=3

### Information
```
Protocol Format:
   +--------------+--------+---------------------------------------+
   |  Parameters  | length |            Description                |
   +--------------+--------+---------------------------------------+
   |Packet_Header |   4    |Data packet header, fixed as A5A5A5A5  |    
   +--------------+--------+---------------------------------------+
   |Device_Address|   1    |Specifies the address of the device    |  
   +--------------+--------+---------------------------------------+
   |Pack_ID       |   1    |Data packet ID (data type)             | 
   +--------------+--------+---------------------------------------+
   |Data_Len      |   2    |Data length of data segment, 0-82      |
   +--------------+--------+---------------------------------------+
   |Data          |   n    |Data, n = Data_Length                  |
   +--------------+--------+---------------------------------------+
   |Check_Sum     |   1    |Checksum, the checksum of the remaining|
   |              |        |  bytes after the header is removed    |
   +--------------+--------+---------------------------------------+

ATTENTION:
    1)
    During command interaction with GS2, except for the stop scan command, other commands
    cannot be interacted in scan mode, which may easily lead to message parsing errors

    2)
    GS2 will not automatically start ranging when power on. It needs to send a start scan command to
    enter the scan mode. When need to stop ranging, send a stop scan command to stop scanning and
    enter sleep mode.

    3)
    Start GS2 normally, our recommended process is:
    First step:
        send the Get Device Address command to get the address of the current device and the number of
        cascades, and configure the address;
    Second step:
        send the get version command to get the version number;
    Third step:
        send a command to obtain device parameters to obtain the angle parameters of the device for data
        analysis;
    Fourth step:
        send a start scan command to obtain point cloud data.

    4)
    In order to reduce the power consumption of the navigation board, if GS2 needs to be powered on
    and off repeatedly, it is recommended to send a stop scan command (see section 3.5) before
    powering off, and then configure the TX and RX of the navigation board to high impedance. Then
    pull VCC low to turn it off. The next time the power is turned on, first pull up VCC, then configure
    TX and RX as normal output and input states, and then after a delay of 300ms, perform command
    interaction with the line laser.

    5)
    About the maximum waiting time after each GS2 command is sent:
    Get address: delay 800ms
    get version: delay 100ms
    Get parameters: delay 100ms
    start scanning: delay 400ms
    Stop scanning: delay 100m
    set baud rate: delay 800ms
    Set edge mode: delay 800m
    start OTA: delay 800ms

DATA ANALYSIS:
``` cpp
    angle_p_x = 1.22
    angle_p_angle = 22.5
    M_PI = π
    angle_p_y = 5.315

    double pixelU = n, Dist, theta, tempTheta, tempDist, tempX, tempY;
    if (n < 80)// Differentiate left and right camera data
    {
    pixelU = 80 - pixelU;
    if (d_compensateB0 > 1) {
    tempTheta = d_compensateK0 * pixelU - d_compensateB0;
    }
    else
    {
    tempTheta = atan(d_compensateK0 * pixelU - d_compensateB0) * 180 / M_PI;
    }
    tempDist = (dist - angle_p_x) / cos((angle_p_angle + bias - (tempTheta)) * M_PI / 180);
    tempTheta = tempTheta * M_PI / 180;
    tempX = cos((angle_p_angle + bias) * M_PI / 180) * tempDist * cos(tempTheta) +
    sin((angle_p_angle + bias) * M_PI / 180) * (tempDist * sin(tempTheta));
    tempY = -sin((angle_p_angle + bias) * M_PI / 180) * tempDist * cos(tempTheta) +
    cos((angle_p_angle + bias) * M_PI / 180) * (tempDist * sin(tempTheta));
    tempX = tempX + angle_p_x;
    tempY = tempY - angle_p_y;
    Dist = sqrt(tempX * tempX + tempY * tempY);
    theta = atan(tempY / tempX) * 180 / M_PI;
    }
    else
    {
    pixelU = 160 - pixelU;
    if (d_compensateB1 > 1)
    {
    tempTheta = d_compensateK1 * pixelU - d_compensateB1;
    }
    else
    {
    tempTheta = atan(d_compensateK1 * pixelU - d_compensateB1) * 180 / M_PI;
    }
    tempDist = (dist - angle_p_x) / cos((angle_p_angle + bias + (tempTheta)) * M_PI / 180);
    Copyright 2021 EAI All Rights Reserved 11 / 18
    tempTheta = tempTheta * M_PI / 180;
    tempX = cos(-(angle_p_angle + bias) * M_PI / 180) * tempDist * cos(tempTheta) + sin(-
    (angle_p_angle + bias) * M_PI / 180) * (tempDist * sin(tempTheta));
    tempY = -sin(-(angle_p_angle + bias) * M_PI / 180) * tempDist * cos(tempTheta) + cos(-
    (angle_p_angle + bias) * M_PI / 180) * (tempDist * sin(tempTheta));
    tempX = tempX + angle_p_x;
    tempY = tempY + angle_p_y;
    Dist = sqrt(tempX * tempX + tempY * tempY);
    theta = atan(tempY / tempX) * 180 / M_PI;
    }
    if (theta < 0)
    {
    theta += 360;
    }
    *dstTheta = theta;
    *dstDist = Dist;
```
```


## License
MIT License

Copyright (c) 2024 HYPERION ROBOTICS

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

The author of this software shall not be held liable for any damages, liabilities, or legal consequences arising from the use, misuse, or inability to use the software.
