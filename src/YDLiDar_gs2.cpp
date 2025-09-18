#include "YDLiDar_gs2.h"

/******************************CONSTRUCTOR/DECONSTRUCTOR******************************/
YDLiDar_GS2::YDLiDar_GS2(HardwareSerial* YDSerial, uint8_t baudrate, uint8_t ser_config, uint8_t rx_pin, uint8_t tx_pin){
    this->YDSerial = YDSerial;
    this->baudrate = baudrate;
    this->ser_config = ser_config;
    this->rx_pin = rx_pin;
    this->tx_pin = tx_pin;
}

YDLiDar_GS2::~YDLiDar_GS2(){
    //close everything
    stopScanning();
    close_buffer();
}
/******************************CONSTRUCTOR/DECONSTRUCTOR******************************/

/******************************PRIVATE METHODS******************************/
/* Protocol Format:
+-------------+---------+-------------+-------------------+--------------+--------------------+
|   HEADER    | ADDRESS | PACKET TYPE |   PACKET LENGTH   |    PACKET    |     CHECK CODE     |
+-------------+---------+-------------+-------------------+--------------+--------------------+
| A5 A5 A5 A5 |   0x0*  |     0x**    |(LSB)0x** (MSB)0x**|(Length)*0x** | (packet type=)0x** |
+-------------+---------+-------------+-------------------+--------------+--------------------+
*/
inline void YDLiDar_GS2::sendCommand(uint8_t device_address, uint8_t packet_type, uint8_t* datasegment, size_t size){
    uint8_t check_sum = 0x00;
    uint8_t length_MSB = (((uint16_t)size) >> 8 )& 0xFF;
    uint8_t length_LSB = ((uint16_t)size) & 0xFF;
    
    //HEADER
    YDSerial->write(GS_LIDAR_HEADER_BYTE);
    YDSerial->write(GS_LIDAR_HEADER_BYTE);
    YDSerial->write(GS_LIDAR_HEADER_BYTE);
    YDSerial->write(GS_LIDAR_HEADER_BYTE);

    //ADDRESS
    YDSerial->write(device_address);
    check_sum += device_address;

    //PACKET TYPE
    YDSerial->write(packet_type);
    check_sum += packet_type;

    //PACKET LENGTH
    YDSerial->write(length_LSB);
    YDSerial->write(length_MSB);
    check_sum += length_LSB;
    check_sum += length_MSB;

    //PACKET
    for(size_t i = 0; i<size; i++){
        YDSerial->write(*(datasegment+i));
        check_sum += *(datasegment+i);
    }

    //CHECK CODE
    YDSerial->write(check_sum);
}

void YDLiDar_GS2::getMeasurements(uint16_t dist, int n, double *dstTheta, uint16_t *dstDist, uint8_t dev_add){
    //get the coefficients of the targeted device
    if(dev_add == 0x04){
        dev_add = 0x03;
    }
    double d_compensateK0 = d_compensateK0_aray[(int)dev_add-1];
    double d_compensateK1 = d_compensateK1_aray[(int)dev_add-1];
    double d_compensateB0 = d_compensateB0_aray[(int)dev_add-1];
    double d_compensateB1 = d_compensateB1_aray[(int)dev_add-1];
    double bias = bias_array[(int)dev_add];

    double pixelU = n, Dist, theta, tempTheta, tempDist, tempX, tempY;
    if (n < 80){
        pixelU = 80 - pixelU;
        if (d_compensateB0 > 1) {
            tempTheta = d_compensateK0 * pixelU - d_compensateB0;
        }else{
            tempTheta = atan(d_compensateK0 * pixelU - d_compensateB0) * 180 / M_PI;
        }
        tempDist = (dist - ANGLE_PX) / cos((ANGLE_PANGLE + bias - (tempTheta)) * M_PI / 180);
        tempTheta = tempTheta * M_PI / 180;
        tempX = cos((ANGLE_PANGLE + bias) * M_PI / 180) * tempDist * cos(tempTheta) +
        sin((ANGLE_PANGLE + bias) * M_PI / 180) * (tempDist * sin(tempTheta));
        tempY = -sin((ANGLE_PANGLE + bias) * M_PI / 180) * tempDist * cos(tempTheta) +
        cos((ANGLE_PANGLE + bias) * M_PI / 180) * (tempDist * sin(tempTheta));
        tempX = tempX + ANGLE_PX;
        tempY = tempY - ANGLE_PY;
        Dist = sqrt(tempX * tempX + tempY * tempY);
        theta = atan(tempY / tempX) * 180 / M_PI;
    }else{
        pixelU = 160 - pixelU;
        if (d_compensateB1 > 1){
            tempTheta = d_compensateK1 * pixelU - d_compensateB1;
        }
        else{
            tempTheta = atan(d_compensateK1 * pixelU - d_compensateB1) * 180 / M_PI;
        }
        tempDist = (dist - ANGLE_PX) / cos((ANGLE_PANGLE + bias + (tempTheta)) * M_PI / 180);
        tempTheta = tempTheta * M_PI / 180;
        tempX = cos(-(ANGLE_PANGLE + bias) * M_PI / 180) * tempDist * cos(tempTheta) + sin(-
        (ANGLE_PANGLE + bias) * M_PI / 180) * (tempDist * sin(tempTheta));
        tempY = -sin(-(ANGLE_PANGLE + bias) * M_PI / 180) * tempDist * cos(tempTheta) + cos(-
        (ANGLE_PANGLE + bias) * M_PI / 180) * (tempDist * sin(tempTheta));
        tempX = tempX + ANGLE_PX;
        tempY = tempY + ANGLE_PY;
        Dist = sqrt(tempX * tempX + tempY * tempY);
        theta = atan(tempY / tempX) * 180 / M_PI;
    }
    if (theta < 0){
        theta += 360;
    }
    *dstTheta = theta;
    *dstDist = Dist;
}

GS_error YDLiDar_GS2::setThecoefficients(){
    open_buffer();
    sendCommand(GS_LIDAR_GLOBAL_ADDRESS, GS_LIDAR_CMD_GET_PARAMETERS);
    YDSerial->flush();
    fixBuffer();
    // delay(2000);

    // Serial.print("Available bytes: ");
    // Serial.println(YDSerial->available());
    //wait until the buffer gets all the bytes responce 
    long start = millis();
    while((YDSerial->available() < (GS_LIDAR_STANDAR_LENGTH + GS_LIDAR_RECV_PARAMETERS_LENGTH)*number_of_lidars) && (millis() - start < 3000)){
    }

    if(YDSerial->available() < (GS_LIDAR_STANDAR_LENGTH + GS_LIDAR_RECV_PARAMETERS_LENGTH)*number_of_lidars){
        // Serial.println("TIME OUT");
        return GS_NOT_OK;
    }
    
    uint8_t captured[YDSerial->available()];//store the message
    
    YDSerial->readBytes(captured, (GS_LIDAR_STANDAR_LENGTH + number_of_lidars*GS_LIDAR_RECV_PARAMETERS_LENGTH));

    // for(int i=0; i<sizeof(captured); i++){
    //     Serial.println(captured[i], HEX);
    // }
    


    // Serial.print("Header");
    // Serial.println(captured[0], HEX);
    // Serial.print("Header");
    // Serial.println(captured[1], HEX);
    // Serial.print("Header");
    // Serial.println(captured[2], HEX);
    // Serial.print("Header");
    // Serial.println(captured[3], HEX);
    // Serial.print("dev");
    // Serial.println(captured[4], HEX);
    // Serial.print("type");
    // Serial.println(captured[5], HEX);
    // Serial.print("lenLSB");
    // Serial.println(captured[6], HEX);
    // Serial.print("lenMSB");
    // Serial.println(captured[7], HEX);
    // Serial.print("K0LSB");
    // Serial.println(captured[8], HEX);
    // Serial.print("K0MSB");
    // Serial.println(captured[9], HEX);
    // Serial.print("B0LSB");
    // Serial.println(captured[10], HEX);
    // Serial.print("B0MSB");
    // Serial.println(captured[11], HEX);
    // Serial.print("K1LSB");
    // Serial.println(captured[12], HEX);
    // Serial.print("K1MSB");
    // Serial.println(captured[13], HEX);
    // Serial.print("B1LSB");
    // Serial.println(captured[14], HEX);
    // Serial.print("B1MSB");
    // Serial.println(captured[15], HEX);
    // Serial.print("bias");
    // Serial.println(captured[16], HEX);
    for(int j=0; j<number_of_lidars; j++){
        for(int i = 0; i < 4; i++){
            if(captured[i] != GS_LIDAR_HEADER_BYTE){
                return GS_NOT_OK;
            }
        }

    
        if(captured[5] != 0x61) return GS_NOT_OK;//Command type
        if(captured[6] != 0x09) return GS_NOT_OK;//Data length LSB
        if(captured[7] != 0x00) return GS_NOT_OK;//Data length MSB

        uint8_t k0_LSB = captured[8+(GS_LIDAR_STANDAR_LENGTH + GS_LIDAR_RECV_PARAMETERS_LENGTH)*j];
        uint8_t k0_MSB = captured[9+(GS_LIDAR_STANDAR_LENGTH + GS_LIDAR_RECV_PARAMETERS_LENGTH)*j];
        uint8_t b0_LSB = captured[10+(GS_LIDAR_STANDAR_LENGTH + GS_LIDAR_RECV_PARAMETERS_LENGTH)*j];
        uint8_t b0_MSB = captured[11+(GS_LIDAR_STANDAR_LENGTH + GS_LIDAR_RECV_PARAMETERS_LENGTH)*j];
        uint8_t k1_LSB = captured[12+(GS_LIDAR_STANDAR_LENGTH + GS_LIDAR_RECV_PARAMETERS_LENGTH)*j];
        uint8_t k1_MSB = captured[13+(GS_LIDAR_STANDAR_LENGTH + GS_LIDAR_RECV_PARAMETERS_LENGTH)*j];
        uint8_t b1_LSB = captured[14+(GS_LIDAR_STANDAR_LENGTH + GS_LIDAR_RECV_PARAMETERS_LENGTH)*j];
        uint8_t b1_MSB = captured[15+(GS_LIDAR_STANDAR_LENGTH + GS_LIDAR_RECV_PARAMETERS_LENGTH)*j];
        uint8_t bias_LSB = captured[16+(GS_LIDAR_STANDAR_LENGTH + GS_LIDAR_RECV_PARAMETERS_LENGTH)*j];

        d_compensateK0_aray[j] = ((float)MSB_LSBtoUINT16(k0_MSB, k0_LSB))/10000.0f;
        d_compensateK1_aray[j] = ((float)MSB_LSBtoUINT16(k1_MSB, k1_LSB))/10000.0f;
        d_compensateB0_aray[j] = ((float)MSB_LSBtoUINT16(b0_MSB, b0_LSB))/10000.0f;
        d_compensateB1_aray[j] = ((float)MSB_LSBtoUINT16(b1_MSB, b1_LSB))/10000.0f;
        bias_array[j] = ((float)bias_LSB)/10;
    }

    YDSerial->read();//Checkbyte

    return GS_OK;
}


void YDLiDar_GS2::setBaudRateToTypical(){
    while(clear_input() != GS_OK){//reassures that the buffer is empty and that that the serial has started
        stopScanningFORCE();
        delay(10);
    }

    for(unsigned int i= 0; i < sizeof(this->baudrate_array)/sizeof(this->baudrate_array[0]); i++){
        this->baudrate = this->baudrate_array[i];
        open_buffer();
        sendCommand(GS_LIDAR_GLOBAL_ADDRESS, GS_LIDAR_CMD_SET_BAUDRATE, (uint8_t*)GS_LIDAR_BAUDRATE_921600, GS_LIDAR_RECV_SET_BAUDRATE_LENGTH);
        YDSerial->flush();
        softRestart();
        YDSerial->flush();
        close_buffer();
    }
    this->baudrate = GS_LIDAR_BAUDRATE_921600;
    open_buffer();
    softRestart();
    close_buffer();
}

inline uint16_t YDLiDar_GS2::MSB_LSBtoUINT16(uint8_t MSB, uint8_t LSB) const{
    return (MSB << 8) | LSB;
}

inline void YDLiDar_GS2::fixBuffer() const{
    int i = 0;
    while (i < BYTES_PER_SCAN + 1) {
        if (YDSerial->peek() == 0xA5){
            return;
        }
        YDSerial->read();
        i++;
    }
}

inline GS_error YDLiDar_GS2::clear_input() const{
    //by stopping and restarting the Serial it clears the buffer
    close_buffer();
    open_buffer();

    long time = millis();
    while (YDSerial->available() > 0 && millis() - time < 1000){
        YDSerial->read();
    }
    return (YDSerial->available() == 0) ? GS_OK : GS_NOT_OK;
}

inline void YDLiDar_GS2::close_buffer() const{
    // YDSerial->end();
}

inline void YDLiDar_GS2::open_buffer() const{
    if(ser_config == 0xFF){
        YDSerial->begin(baudrate_array[baudrate]);
    }else if(rx_pin == 0xFF || tx_pin == 0xFF){
        YDSerial->begin(baudrate_array[baudrate], SERIAL_8N1);
    }
    #ifdef ARDUINO_ARCH_ESP32
     else if(rx_pin != 0xFF && tx_pin != 0xFF){
        YDSerial->begin(baudrate_array[baudrate], SERIAL_8N1, rx_pin, tx_pin);
    }
    #endif

}

void YDLiDar_GS2::stopScanningFORCE(){
    //send to all possible baud rates to stop scanning
    close_buffer();
    for(unsigned int i = 0; i < sizeof(this->baudrate_array)/sizeof(this->baudrate_array[0]); i++){
        open_buffer();
        sendCommand(GS_LIDAR_GLOBAL_ADDRESS, GS_LIDAR_CMD_STOP_SCAN);
        YDSerial->flush();
        sendCommand(GS_LIDAR_GLOBAL_ADDRESS, GS_LIDAR_CMD_STOP_SCAN);
        YDSerial->flush();
        close_buffer();
    }
}
/******************************PRIVATE METHODS******************************/

/******************************PUBLIC METHODS******************************/
GS_error YDLiDar_GS2::initialize(int number_of_lidars){
    open_buffer();

    uint8_t addr = ping();
    if(addr == 0x04) addr = 0x03;

    this->number_of_lidars = (int)addr;

    if(this->number_of_lidars == 0){
        return GS_NOT_OK;
    }
 
    unsigned long start = millis();
    while(setThecoefficients() != GS_OK){
        if(millis() - start > 1000){
            return GS_NOT_OK;
        }
    }
    
    return GS_OK;
}

uint8_t YDLiDar_GS2::ping(){
    long start = millis();
    int available = -1;
    uint8_t address = 0x00;// Nothing found
    
    close_buffer();
    open_buffer();
    sendCommand(GS_LIDAR_GLOBAL_ADDRESS, GS_LIDAR_CMD_GET_ADDRESS);
    YDSerial->flush();

    while(millis() - start < 100){
        if(YDSerial->available() >  GS_LIDAR_STANDAR_LENGTH){
            // Serial.println("Spam in the serial");
            return 0x03; // Spam in the serial
        }else if(YDSerial->available() != available){
            available = YDSerial->available();
            start = millis();
        }
    }
    uint8_t buffer[YDSerial->available()];

    YDSerial->readBytes(buffer, sizeof(buffer));

    short counter = 0;
    for(unsigned int i = 0; i < sizeof(buffer); i++) {
        // Serial.println(buffer[i], HEX);
        if(buffer[i] == GS_LIDAR_HEADER_BYTE && counter < 4){
            counter++;
            continue;
        }else if(counter < 4){
            counter = 0;
            continue;
        }
        uint8_t checksum = (buffer[i] + buffer[i+1] + buffer[i+2] + buffer[i+3]) & 0xFF;

        if(counter == 4 && buffer[i+1] == GS_LIDAR_CMD_GET_ADDRESS && buffer[i+2] == 0x00 && buffer[i+3] == 0x00 && checksum == buffer[i+4]){
            if(buffer[i] > address){
                address = buffer[i];
            }
        }else{
            counter = 0;
        }
    }

    return address;
}


GS_error YDLiDar_GS2::setBaudrate(uint8_t Baudrate){
    //make sure the buffer is open
    open_buffer();  

    sendCommand(GS_LIDAR_GLOBAL_ADDRESS, GS_LIDAR_CMD_SET_BAUDRATE, &Baudrate, GS_LIDAR_RECV_SET_BAUDRATE_LENGTH);
    YDSerial->flush();
    softRestart();
    YDSerial->flush();
    close_buffer();

    this->baudrate = Baudrate;
    open_buffer();
    softRestart();
    YDSerial->flush();

    return GS_OK;
}

GS_error YDLiDar_GS2::setedgeMode(uint8_t mode, uint8_t address){
    sendCommand(address, GS_LIDAR_CMD_SET_EDGE_MODE, &mode, GS_LIDAR_RECV_EDGE_MODE_LENGTH);
    YDSerial->flush();
    clear_input();
    return GS_OK;
}

GS_error YDLiDar_GS2::setNumberofLiDars(int num){
    if(num <= 0 || 3 < num){
        return GS_NOT_OK;
    }
    number_of_lidars = num;
    return GS_OK;
}


GS_error YDLiDar_GS2::startScanning(){
    open_buffer();
    sendCommand(GS_LIDAR_GLOBAL_ADDRESS, GS_LIDAR_CMD_SCAN);
    YDSerial->flush();
    close_buffer();
    return GS_OK;
}

GS_error YDLiDar_GS2::stopScanning(){
    open_buffer();
    sendCommand(GS_LIDAR_GLOBAL_ADDRESS, GS_LIDAR_CMD_STOP_SCAN);
    YDSerial->flush();
    clear_input();
    close_buffer();
    return GS_OK;
}

iter_Scan YDLiDar_GS2::iter_scans(uint8_t dev_address){
    int recv_pos = 0;
    bool successful_capture = false;

    uint8_t check_sum;
    uint16_t sample_lens;

    //ensures that the starting bytes are correct
    open_buffer();
    for(int pos = 0; pos < (BYTES_PER_SCAN + 1); pos++){
        if(YDSerial->available()){
            uint8_t currentByte = YDSerial->read();
            switch (recv_pos){
                case 0: case 1: case 2: case 3:
                    if(currentByte != GS_LIDAR_HEADER_BYTE){
                        recv_pos = -1;
                        continue;
                    }
                    break; 
                case 4:
                    if(currentByte != dev_address){
                        recv_pos = -1;
                        pos = -1;
                        continue;
                    }
                    check_sum = currentByte;
                    break;
                case 5:
                    if(currentByte != GS_LIDAR_CMD_SCAN){
                        close_buffer();
                        return iter_Scan();
                    }
                    break;
                    check_sum += currentByte;
                case 6:
                    if(currentByte != 0x42){
                        close_buffer();
                        return iter_Scan();
                    }
                    sample_lens |= 0x00ff & currentByte;
                    check_sum += currentByte;
                    break;
                case 7:
                    if(currentByte != 0x01){
                        close_buffer();
                        return iter_Scan();
                    }
                    successful_capture = true;
                    sample_lens |= (0x00ff & currentByte) << 8;
                    check_sum += currentByte;
                    break;
            }
            if(recv_pos == 7){
                break;
            }
            recv_pos++;
        }else{
            pos--;
        }
    }
    if (!successful_capture){
        close_buffer();
        return iter_Scan();
    }

    recv_pos = 0;
    uint8_t captured[GS_ENV_MEASUREMENT_LENGTH + SCANS_PER_CYCLE*GS_SI_MEASUREMENT_LENGTH];
    while (recv_pos < GS_ENV_MEASUREMENT_LENGTH + SCANS_PER_CYCLE*GS_SI_MEASUREMENT_LENGTH){
        if(YDSerial->available()){
            captured[recv_pos] = YDSerial->read();
            recv_pos++;
        }
    }    
    close_buffer();
    
    iter_Scan scan;

    scan.env = MSB_LSBtoUINT16(captured[0], captured[1]);

    for(int i=0; i < SCANS_PER_CYCLE*2; i += 2){
        //locator
        int n = (i/2);
        
        uint16_t sampleAngle = 0;
        scan.valid[n] = true;
        //distance and angle correction
        getMeasurements((MSB_LSBtoUINT16(captured[(i+1)+2], captured[i+2]) & 0x01ff), n, &scan.angle[n], &scan.distance[n], dev_address);


        scan.quality[n] = (captured[(i+1)+2] >> 1);

        //filter for incorect captures
        if(scan.angle[n] < 0){
            sampleAngle = (((uint16_t)(scan.angle[n] * 64 + 23040)) << LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) + LIDAR_RESP_MEASUREMENT_CHECKBIT;
        }else{
            if ((scan.angle[n] * 64) > 23040) {
                sampleAngle = (((uint16_t)(scan.angle[n]* 64 - 23040)) << LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) + LIDAR_RESP_MEASUREMENT_CHECKBIT;
            } else {
                sampleAngle = (((uint16_t)(scan.angle[n] * 64)) << LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) + LIDAR_RESP_MEASUREMENT_CHECKBIT;
            }
        }
        
        if(n < 80){
            if(sampleAngle <= 23041){
                scan.distance[n] = 0;
                scan.valid[n] = false;
            }
        }else {
            if(sampleAngle > 23041){
                scan.distance[n] = 0;
                scan.valid[n] = false;
            }
        }

        scan.angle[n] = (sampleAngle >> LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f;

        if((scan.distance[n] < 25 || scan.distance[n] > 300 || (55 < scan.angle[n] && scan.angle[n] < 305) ) && scan.valid[n] == true){
            // scan.distance[n] = 0;
            scan.valid[n] = false;
        }
    }
    return scan;
}

iter_multi_Scans YDLiDar_GS2::iter_multi_Scan(){
    int recv_pos = 0;
    bool successful_capture = false;
    bool break_for = false;

    uint8_t check_sum;
    uint16_t sample_lens;

    bool address_1_captured = number_of_lidars < 1;
    bool address_2_captured = number_of_lidars < 2;
    bool address_3_captured = number_of_lidars < 3;

    uint8_t captured1[GS_ENV_MEASUREMENT_LENGTH + SCANS_PER_CYCLE*GS_SI_MEASUREMENT_LENGTH];
    uint8_t captured2[GS_ENV_MEASUREMENT_LENGTH + SCANS_PER_CYCLE*GS_SI_MEASUREMENT_LENGTH];
    uint8_t captured3[GS_ENV_MEASUREMENT_LENGTH + SCANS_PER_CYCLE*GS_SI_MEASUREMENT_LENGTH];
    uint8_t* captured_ptr;

    //ensures that the starting bytes are correct
    open_buffer();

    long start_time = millis();
    while((!address_1_captured || !address_2_captured || !address_3_captured) && (millis() - start_time < 1000)){
        for(int pos = 0; pos < (BYTES_PER_SCAN + 1); pos++){
            if(YDSerial->available()){
                uint8_t currentByte = YDSerial->read();
                switch (recv_pos){
                    case 0: case 1: case 2: case 3:
                        if(currentByte != GS_LIDAR_HEADER_BYTE){
                            recv_pos = -1;
                            continue;
                        }
                        break; 
                    case 4:
                        if(currentByte == 0x01){
                            address_1_captured = true;
                            captured_ptr = captured1;
                        }else if(currentByte == 0x02){
                            address_2_captured = true;
                            captured_ptr = captured2;
                        }else if(currentByte == 0x04){
                            address_3_captured = true;
                            captured_ptr = captured3;
                        }else{
                            recv_pos = -1;
                            pos = -1;
                            continue;
                        }
                        check_sum = currentByte;
                        break;
                    case 5:
                        if(currentByte != GS_LIDAR_CMD_SCAN){
                            break_for = true;
                        }
                        break;
                        check_sum += currentByte;
                    case 6:
                        if(currentByte != 0x42){
                            break_for = true;
                        }
                        sample_lens |= 0x00ff & currentByte;
                        check_sum += currentByte;
                        break;
                    case 7:
                        if(currentByte != 0x01){
                            break_for = true;
                        }
                        successful_capture = true;
                        sample_lens |= (0x00ff & currentByte) << 8;
                        check_sum += currentByte;
                        break;
                }
                
                if(recv_pos == 7 || break_for){;
                    break;
                }
                recv_pos++;
            }else{
                pos--;
            }
        }

        if(break_for){
            break_for = false;
            recv_pos = 0;
            continue;
        }

        recv_pos = 0;
        while (recv_pos < GS_ENV_MEASUREMENT_LENGTH + SCANS_PER_CYCLE*GS_SI_MEASUREMENT_LENGTH){
            if(YDSerial->available()){
                *(captured_ptr + recv_pos) = YDSerial->read();
                recv_pos++;
            }
        }
        
    }

    close_buffer();

    if(!successful_capture){
        return iter_multi_Scans();
    }

    iter_multi_Scans multi_scans;

    multi_scans.env1 = MSB_LSBtoUINT16(captured1[0], captured1[1]);
    multi_scans.env2 = MSB_LSBtoUINT16(captured2[0], captured2[1]);
    multi_scans.env3 = MSB_LSBtoUINT16(captured3[0], captured3[1]);

    for(int i=0; i < SCANS_PER_CYCLE*2; i += 2){
        //locator
        int n = (i/2);
        
        uint16_t sampleAngle1 = 0;
        uint16_t sampleAngle2 = 0;
        uint16_t sampleAngle3 = 0;

        multi_scans.valid1[n] = true;
        multi_scans.valid2[n] = true;
        multi_scans.valid3[n] = true;
        //distance and angle correction
        getMeasurements((MSB_LSBtoUINT16(captured1[(i+1)+2], captured1[i+2]) & 0x01ff), n, &multi_scans.angle1[n], &multi_scans.distance1[n], 0x01);
        getMeasurements((MSB_LSBtoUINT16(captured2[(i+1)+2], captured2[i+2]) & 0x01ff), n, &multi_scans.angle2[n], &multi_scans.distance2[n], 0x02);
        getMeasurements((MSB_LSBtoUINT16(captured3[(i+1)+2], captured3[i+2]) & 0x01ff), n, &multi_scans.angle3[n], &multi_scans.distance3[n], 0x03);


        multi_scans.quality1[n] = (captured1[(i+1)+2] >> 1);
        multi_scans.quality2[n] = (captured2[(i+1)+2] >> 1);
        multi_scans.quality3[n] = (captured3[(i+1)+2] >> 1);

        //filter for incorect captures
        if(multi_scans.angle1[n] < 0){
            sampleAngle1 = (((uint16_t)(multi_scans.angle1[n] * 64 + 23040)) << LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) + LIDAR_RESP_MEASUREMENT_CHECKBIT;
        }else{
            if ((multi_scans.angle1[n] * 64) > 23040) {
                sampleAngle1 = (((uint16_t)(multi_scans.angle1[n]* 64 - 23040)) << LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) + LIDAR_RESP_MEASUREMENT_CHECKBIT;
            } else {
                sampleAngle1 = (((uint16_t)(multi_scans.angle1[n] * 64)) << LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) + LIDAR_RESP_MEASUREMENT_CHECKBIT;
            }
        }
        
        if(n < 80){
            if(sampleAngle1 <= 23041){
                multi_scans.distance1[n] = 0;
                multi_scans.valid1[n] = false;
            }
        }else {
            if(sampleAngle1 > 23041){
                multi_scans.distance1[n] = 0;
                multi_scans.valid1[n] = false;
            }
        }

        multi_scans.angle1[n] = (sampleAngle1 >> LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f;

        if((multi_scans.distance1[n] < 25 || multi_scans.distance1[n] > 300 || (55 < multi_scans.angle1[n] && multi_scans.angle1[n] < 305) ) && multi_scans.valid1[n] == true){
            // multi_scans.distance1[n] = 0;
            multi_scans.valid1[n] = false;
        }


        if(multi_scans.angle2[n] < 0){
            sampleAngle2 = (((uint16_t)(multi_scans.angle2[n] * 64 + 23040)) << LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) + LIDAR_RESP_MEASUREMENT_CHECKBIT;
        }else{
            if ((multi_scans.angle2[n] * 64) > 23040) {
                sampleAngle2 = (((uint16_t)(multi_scans.angle2[n]* 64 - 23040)) << LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) + LIDAR_RESP_MEASUREMENT_CHECKBIT;
            } else {
                sampleAngle2 = (((uint16_t)(multi_scans.angle2[n] * 64)) << LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) + LIDAR_RESP_MEASUREMENT_CHECKBIT;
            }
        }
        
        if(n < 80){
            if(sampleAngle2 <= 23041){
                multi_scans.distance2[n] = 0;
                multi_scans.valid2[n] = false;
            }
        }else {
            if(sampleAngle2 > 23041){
                multi_scans.distance2[n] = 0;
                multi_scans.valid2[n] = false;
            }
        }

        multi_scans.angle2[n] = (sampleAngle2 >> LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f;

        if((multi_scans.distance2[n] < 25 || multi_scans.distance2[n] > 300 || (55 < multi_scans.angle2[n] && multi_scans.angle2[n] < 305) ) && multi_scans.valid2[n] == true){
            // multi_scans.distance2[n] = 0;
            multi_scans.valid2[n] = false;
        }
        
        if(multi_scans.angle3[n] < 0){
            sampleAngle3 = (((uint16_t)(multi_scans.angle3[n] * 64 + 23040)) << LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) + LIDAR_RESP_MEASUREMENT_CHECKBIT;
        }else{
            if ((multi_scans.angle3[n] * 64) > 23040) {
                sampleAngle3 = (((uint16_t)(multi_scans.angle3[n]* 64 - 23040)) << LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) + LIDAR_RESP_MEASUREMENT_CHECKBIT;
            } else {
                sampleAngle3 = (((uint16_t)(multi_scans.angle3[n] * 64)) << LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) + LIDAR_RESP_MEASUREMENT_CHECKBIT;
            }
        }

        if(n < 80){
            if(sampleAngle3 <= 23041){
                multi_scans.distance3[n] = 0;
                multi_scans.valid3[n] = false;
            }
        }else {
            if(sampleAngle3 > 23041){
                multi_scans.distance3[n] = 0;
                multi_scans.valid3[n] = false;
            }
        }
        multi_scans.angle3[n] = (sampleAngle3 >> LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f;
        if((multi_scans.distance3[n] < 25 || multi_scans.distance3[n] > 300 || (55 < multi_scans.angle3[n] && multi_scans.angle3[n] < 305) ) && multi_scans.valid3[n] == true){
            // multi_scans.distance3[n] = 0;
            multi_scans.valid3[n] = false;
        }

    }

    return multi_scans;
}


void YDLiDar_GS2::softRestart(){
    sendCommand(GS_LIDAR_GLOBAL_ADDRESS, GS_LIDAR_RECV_SOFT_RESET_LENGTH);
    YDSerial->flush();
    clear_input();
}

GS_error YDLiDar_GS2::getParametes(uint8_t device, double* d_K0, double* d_B0, double* d_K1, double* d_B1, double* Bias){
    if(device == 0x04){
        device = 0x03;
    }

    if(device < 0x01 && device > 0x03){
        return GS_NOT_OK;
    }

    *d_K0 = d_compensateK0_aray[(int)device-1];
    *d_K1 = d_compensateK1_aray[(int)device-1];
    *d_B0 = d_compensateB0_aray[(int)device-1];
    *d_B1 = d_compensateB1_aray[(int)device-1];
    *Bias =   bias_array[(int)device-1];
    
    return GS_OK;
}

GS_error YDLiDar_GS2::getVersion(char* V1, char* SN1, char* V2, char* SN2, char* V3, char* SN3){
    clear_input();
    sendCommand(GS_LIDAR_GLOBAL_ADDRESS, GS_LIDAR_CMD_GET_VERSION);
    YDSerial->flush();

    long start_time = millis();
    while(YDSerial->available() < (GS_LIDAR_STANDAR_LENGTH + GS_LIDAR_RECV_VERSION_LENGTH)*number_of_lidars){
        if(millis() - start_time > 1000){
            return GS_NOT_OK;
        }
    }

    fixBuffer();

    uint8_t capture[(GS_LIDAR_STANDAR_LENGTH + GS_LIDAR_RECV_VERSION_LENGTH)*number_of_lidars];

    YDSerial->readBytes(capture, sizeof(capture));


    for(int device = 0; device<number_of_lidars; device++){
        int offset = device * (GS_LIDAR_STANDAR_LENGTH + GS_LIDAR_RECV_VERSION_LENGTH);

        for(int i = 0; i < 4; i++){
            if(capture[i] != GS_LIDAR_HEADER_BYTE){
                return GS_NOT_OK;
            }
        }

        if(capture[5+offset] != GS_LIDAR_CMD_GET_VERSION) return GS_NOT_OK;
        if(capture[6+offset] != 0x13) return GS_NOT_OK;
        if(capture[7+offset] != 0x00) return GS_NOT_OK;

        if(device == 0){
            memcpy(V1, &capture[8+offset], 3);
            memcpy(SN1, &capture[11+offset], 16);
        }else if(device == 1){
            memcpy(V2, &capture[8+offset], 3);
            memcpy(SN2, &capture[11+offset], 16);
        }else if(device == 2){
            memcpy(V3, &capture[8+offset], 3);
            memcpy(SN3, &capture[11+offset], 16);
        }else{
            return GS_NOT_OK;
        }
    }

    return GS_OK;
}
/******************************PUBLIC METHODS******************************/
