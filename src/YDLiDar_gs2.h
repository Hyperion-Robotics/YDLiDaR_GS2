#ifndef YDLIDAR_H 
#define YDLIDAR_H

#ifdef ARDUINO_AVR_UNO
#error this library doesnt support Uno board due to the <map> library. Will be fixed soon
#endif

#include <Arduino.h>
#include <math.h>
#ifndef ARDUINO//just so vscod doesnt pop errors i will remove it soon i just cant stand red lines everywhere i make a uint
#include <cstdint>
#endif

#include <map>
#include <cmath>

#include "YDLiDar_defines.h"
#include "YDLiDar_structs.h"    
#include "YDLiDar_enums.h"    


/**
 * @class   YDLiDar_GS2
 * @brief   Class for reading measurements from YDLiDar_GS2
 * 
 * @attention   Only The YDLiDar_GS2 GS2 is supported 
 */
class YDLiDar_GS2{
private:
    double d_compensateK0_aray[GS_LIDAR_THREADED_LIDARS_LIMIT];      //< coefficients for each connected lidar
    double d_compensateK1_aray[GS_LIDAR_THREADED_LIDARS_LIMIT];      //< coefficients for each connected lidar
    double d_compensateB0_aray[GS_LIDAR_THREADED_LIDARS_LIMIT];      //< coefficients for each connected lidar
    double d_compensateB1_aray[GS_LIDAR_THREADED_LIDARS_LIMIT];      //< coefficients for each connected lidar
    double bias_array[GS_LIDAR_THREADED_LIDARS_LIMIT];               //< coefficients for each connected lidar

    HardwareSerial* YDSerial;           //< Serial port that communicates with the lidar

    std::map<uint8_t, int> baudrates;   //< The keys represents the byte message of the baud rate and the stored value its integer value for the serial communication 

    int S_measurement[GS_LIDAR_THREADED_LIDARS_LIMIT] = {0};         //< Counter that stores wich Si meausurment should the iter_measurments() return
    int number_of_lidars = 1;           //< The number of threaded lidars        
    int baudrate = 921600;              //< The baud rate that the YDSerial runs 

    /**
     * @brief   Send bytes in the protocol format
     * @param   device_address the address of the device that should recieve the message 
     * @param   packet_type the byte of the wanted cmd command
     * @param   datasegment the byte array/pointer of the message 
     * @param   size the size of the array/pointer
     * 
     * @attention   The size is requered or a 0 legth message will be sent that means only the packet type
     */
    inline void sendCommand(uint8_t device_address, uint8_t packet_type, uint8_t* datasegment = nullptr, size_t size = 0);

    /**
     * @brief   Get the calibrated measurments based on the coefficients
     * @param   dist the distance received from the lidar (first 9 bits)
     * @param   n the i of the measurment in the received byte data
     * @param   dstTheta the corected angle of the measurment
     * @param   dstDist the corected distance of the measurement
     * @param   dev_add the address of the targeted lidar
     */
    void getMeasurements(uint16_t dist, int n, double *dstTheta, uint16_t *dstDist, uint8_t dev_add);

    /**
     * @brief   Get the coefficients of every lidar
     */
    void setThecoefficients();

    /**
     * @brief   Set the baud rate of the lidar to 921600(typical)
     */
    void setBaudRateToTypical();

    /**
     * @brief   Convert two uint8_t(bytes) to a uint16_t
     * @param   MSB the most significant bytes(xx--)
     * @param   LSB the least significant bytes(--xx)
     * 
     * @returns     (uint16)(xxxx)
     */
    inline uint16_t MSB_LSBtoUINT16(uint8_t MSB, uint8_t LSB) const;

    /**
     * @brief   Discards bytes from the buffer until a 0xA5 is encounterd
     */
    inline void fixBuffer() const;

    /**
     * @brief   Closes and reopens the YDSerial to discard all saved bytes
     */
    inline void clear_input() const;

    /**
     * @brief   Closes the YDSerial so the buffer to stop receiving bytes and to discard any saved ones
     */
    inline void close_buffer() const;

    /**
     * @brief   opens the YDSerial
     */
    inline void open_buffer() const;

    /**
     * @brief   Sends to every baudrate that the lidar can read the stop scanning command
     */
    void stopScanningFORCE();
public:
    /**
     * @brief   Constructor
     * @param   YDSerial the address of the Serial that will communicate with the lidar
     * @param   baudrate the baudrate of the lidars and YDSerial 
     * 
     * @attention   The Serial that will be given to to the object !MUST NOT! be used anywhere else,
     *              any sharing of the serial can result to infinite loops              
     */
    YDLiDar_GS2(HardwareSerial* YDSerial, uint8_t baudrate = GS_LIDAR_BAUDRATE_921600);

    /**
     * @brief   Destractor
     */
    ~YDLiDar_GS2();

    /**
     * @brief   Setting up the communication between the board and the GS2
     * @param   number_of_lidars the number of connected devices(optional)
     * 
     * @return  
     *          GS_OK: The initialiasation was successful
     *          GS_NOT_OK: The initialiasation was not successful
     */
    GS_error initialize(int number_of_lidars = 0);

    /**
     * @brief   The baudrate of the communication
     * @param   Baudrate the baud rate that the board will communicate with the lidar
     * 
     * @return  
     *          GS_OK: The baudrates was set successfully
     *          GS_NOT_OK: The baudrates was not set successfully
     */
    GS_error setBaudrate(uint8_t Baudrate);

    /**
     * @brief   The edge mode of the lidar
     * @param   mode the mode(Standard, face up edge mode, face down edge mode)
     * 
     * @return  
     *          GS_OK: The edge mode was set successfully
     *          GS_NOT_OK: The edge mode was not set successfully
     */
    GS_error setedgeMode(uint8_t mode, uint8_t address);

    /**
     * @brief   The number of connected lidars
     * @param   num the amount of threaded lidars
     * 
     * @return  
     *          GS_OK: The number of connected lidars was set successfully
     *          GS_NOT_OK: The number of connected lidars was not successful
     */
    GS_error setNumberofLiDars(int num);

    /**
     * @brief   Verify the number of connected lidars
     * @param   number_of_devices the wanted amount of lidars
     * 
     * @return  
     *          true: The number_of_devices is equal to the connected devices
     *          false: The number_of_devices is not equal to the connected devices
     */
    bool verifyNumberOfDevices(int number_of_devices);

    /**
     * @brief   Starts the scanning
     */
    GS_error startScanning();

    /**
     * @brief   Stops the scanning
     */
    GS_error stopScanning();

    /**
     * @brief   Gives a single measurement
     * @param   dev_address the address of the lidar to take the measurments
     * 
     * @return
     *          iter_Measurement(): There was a problem while reading
     *          !iter_Measurement(): The struct of the single measurement (.env, .quality, .angle, .distance)
     */
    iter_Measurement iter_measurments(uint8_t dev_address = 0x01);

    /**
     * @brief   Gives a all received measurements
     * @param   dev_address the address of the lidar to take the measurments
     * 
     * @return
     *          iter_Scan(): There was a problem while reading
     *          !iter_Scan(): The struct of the total measurements (.env, .quality[160], .angle[160], .distance[160])
     */
    iter_Scan iter_scans(uint8_t dev_address = 0x01);

    /**
     * @brief   The coefficients of the lidar
     * @param   device the targeted device
     * @param   d_K0 the K0 coefitient
     * @param   d_B0 the B0 coefitient
     * @param   d_K1 the K1 coefitient
     * @param   d_B1 the B1 coefitient
     * @param   Bias the bias coefitient
     * 
     * @return
     *          GS_OK: The coefficients exist
     *          GS_NOT_OK: The coefficients do not exist
     */    
    GS_error getParametes(uint8_t device, double* d_K0, double* d_B0, double* d_K1, double* d_B1, double*Bias);

    /**
     * @brief   Get the version of the lidar
     * @param   V1 the version number of the first device
     * @param   SN1 the serisal version number of the first device
     * @param   V2 the version number of the second device
     * @param   SN2 the serisal version number of the second device
     * @param   V3 the version number of the third device
     * @param   SN3 the serisal version number of the third device
     * 
     * @return
     *          GS_OK: The version was returned
     *          GS_NOT_OK: The version was not find
     */
    GS_error getVersion(char* V1, char* SN1, char* V2 = nullptr, char* SN2 = nullptr, char* V3 = nullptr, char* SN3 = nullptr);

    /**
     * @brief   Get the number of connected devices
     * 
     * @returns
     *          (int)(the amount of connected devices)
     */
    int getNumberOfDevices();
    
    /**
     * @brief   soft Restrt the lidar 
     */
    void softRestart();
};


#endif
