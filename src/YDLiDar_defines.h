#ifndef YDLIDAR_DEFINES_H
#define YDLIDAR_DEFINES_H

//constants for calculations
#ifndef M_PI
#define M_PI 3.1415926
#endif
#define ANGLE_PX 1.22
#define ANGLE_PY 5.315
#define ANGLE_PANGLE 22.5

//gs2 addresses
#define GS_LIDAR_THREADED_LIDARS_LIMIT 3
#define GS_LIDAR_GLOBAL_ADDRESS 0x00
#define GS_LIDAR_ADDRESS_1 0x01
#define GS_LIDAR_ADDRESS_2 0x02
#define GS_LIDAR_ADDRESS_3 0x04

//command bytes
#define GS_LIDAR_CMD_GET_ADDRESS               0x60
#define GS_LIDAR_CMD_GET_PARAMETERS            0x61
#define GS_LIDAR_CMD_GET_VERSION               0x62
#define GS_LIDAR_CMD_SCAN                      0x63
#define GS_LIDAR_ANS_SCAN                      0x63
#define GS_LIDAR_CMD_STOP_SCAN                 0x64
#define GS_LIDAR_CMD_RESET                     0x67
#define GS_LIDAR_CMD_SET_BAUDRATE              0x68
#define GS_LIDAR_CMD_SET_EDGE_MODE             0x69
#define GS_LIDAR_CMD_SET_BIAS                  0xD9
#define GS_LIDAR_CMD_SET_DEBUG_MODE            0xF0

//command bytes of availbable baud rates
#define GS_LIDAR_BAUDRATE_230400               0x00
#define GS_LIDAR_BAUDRATE_512000               0x01
#define GS_LIDAR_BAUDRATE_921600               0x02
#define GS_LIDAR_BAUDRATE_1500000              0x03

//edge modes
#define GS_STANDARD_EDGE_MODE                  0x00
#define GS_FACE_UP_ENDGE_MODE                  0x01
#define GS_FACE_DOWN_MODE                      0x02
#define GS_READ_EDGE_MODE                      0xFF

//protocol bytes
#define GS_LIDAR_HEADER_BYTE                   0xA5
#define GS_LIDAR_SCAN_LENGTH_MSB               0x01
#define GS_LIDAR_SCAN_LENGTH_LSB               0x42

//constants for byte length calculations{
//starting bytes lengths
#define GS_LIDAR_HEADER_LENGTH                 4
#define GS_LIDAR_ADDRESS_LENGTH                1
#define GS_LIDAR_COMMAND_TYPE_LENGTH           1
#define GS_LIDAR_DATA_LENGTH_LENGTH            2
//received byte length(one of them)
#define GS_LIDAR_RECV_ADDRESS_LENGTH           0
#define GS_LIDAR_RECV_VERSION_NUMBER_LENGTH    3       
#define GS_LIDAR_RECV_SERIAL_NUMBER_LENGTH     16    
#define GS_LIDAR_RECV_VERSION_LENGTH           19
#define GS_LIDAR_RECV_PARAMETERS_LENGTH        9
#define GS_LIDAR_RECV_SCAN_LENGTH              322
#define GS_LIDAR_RECV_STOP_SCAN_LENGTH         0
#define GS_LIDAR_RECV_SET_BAUDRATE_LENGTH      1
#define GS_LIDAR_RECV_EDGE_MODE_LENGTH         1
#define GS_LIDAR_RECV_SOFT_RESET_LENGTH        0
//final bytes length
#define GS_LIDAR_CHECK_CODE_LENGTH             1
//standard byte length
#define GS_LIDAR_STANDAR_LENGTH (GS_LIDAR_HEADER_LENGTH + GS_LIDAR_ADDRESS_LENGTH + GS_LIDAR_COMMAND_TYPE_LENGTH + GS_LIDAR_DATA_LENGTH_LENGTH + GS_LIDAR_CHECK_CODE_LENGTH)

//env and Si length
#define GS_ENV_MEASUREMENT_LENGTH 2
#define GS_SI_MEASUREMENT_LENGTH 2
//}

//measurment limits
#define BYTES_PER_SCAN (GS_LIDAR_HEADER_LENGTH + GS_LIDAR_ADDRESS_LENGTH + GS_LIDAR_COMMAND_TYPE_LENGTH + GS_LIDAR_DATA_LENGTH_LENGTH + GS_LIDAR_RECV_SCAN_LENGTH + GS_LIDAR_CHECK_CODE_LENGTH)

//baffer options
#define MINIMUM_BUFFER_STORAGE (BYTES_PER_SCAN)
#define SAFE_HIGH_BUFFER_STORAGE (2*MINIMUM_BUFFER_STORAGE)
#ifdef TEENSYDUINO
#define MAX_BUFFER_STORAGE (3*MINIMUM_BUFFER_STORAGE)
#elif defined(ESP32)
#define MAX_BUFFER_STORAGE (3*MINIMUM_BUFFER_STORAGE)
#elif defined(PICO_RP2040)
#define MAX_BUFFER_STORAGE (2*MINIMUM_BUFFER_STORAGE)
#else
#endif

//number of distances received per scan cycle
#define SCANS_PER_CYCLE 160

//data analysis utils
#define LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT 1
#define LIDAR_RESP_MEASUREMENT_CHECKBIT (0x1<<0)

#endif