#include "YDLiDar_gs2.h"

/******************************CONSTRUCTOR/DECONSTRUCTOR******************************/
YDLiDar_GS2::YDLiDar_GS2(HardwareSerial* YDSerial, uint8_t baudrate){
    //set the keys and the values of the maps
    baudrates[GS_LIDAR_BAUDRATE_230400] = 230400;
    baudrates[GS_LIDAR_BAUDRATE_512000] = 512000;
    baudrates[GS_LIDAR_BAUDRATE_921600] = 921600;
    baudrates[GS_LIDAR_BAUDRATE_1500000] = 1500000;

    //set the values of the arrays
    S_measurement[GS_LIDAR_ADDRESS_1-1] = 0;
    S_measurement[GS_LIDAR_ADDRESS_2-1] = 0;
    S_measurement[GS_LIDAR_ADDRESS_3-2] = 0;

    //set the desired baudrate
    this->baudrate = baudrates[baudrate];

    //get the Serial
    this->YDSerial = YDSerial;
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
    //HEADER
    YDSerial->write(GS_LIDAR_HEADER_BYTE);
    YDSerial->write(GS_LIDAR_HEADER_BYTE);
    YDSerial->write(GS_LIDAR_HEADER_BYTE);
    YDSerial->write(GS_LIDAR_HEADER_BYTE);

    //ADDRESS
    YDSerial->write(device_address);

    //PACKET TYPE
    YDSerial->write(packet_type);

    //PACKET LENGTH
    uint8_t length_MSB = (((uint16_t)size) >> 8 )& 0xFF;
    uint8_t length_LSB = ((uint16_t)size) & 0xFF;

    YDSerial->write(length_LSB);
    YDSerial->write(length_MSB);

    //PACKET
    for(size_t i = 0; i<size; i++){
        YDSerial->write(*(datasegment+i));
    }

    //CHECK CODE
    YDSerial->write(packet_type);
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
    sendCommand(GS_LIDAR_GLOBAL_ADDRESS, GS_LIDAR_CMD_GET_PARAMETERS);
    delay(500);
    fixBuffer();
    
    //wait until the buffer gets all the bytes responce 
    long start = millis();
    while((YDSerial->available() < GS_LIDAR_STANDAR_LENGTH + number_of_lidars*GS_LIDAR_RECV_PARAMETERS_LENGTH) && (millis() - start < 3000)){
    }
    
    if(YDSerial->available() < GS_LIDAR_STANDAR_LENGTH + number_of_lidars*GS_LIDAR_RECV_PARAMETERS_LENGTH){
        // Serial.println("TIME OUT");
        return GS_NOT_OK;
    }
    
    uint8_t captured[GS_LIDAR_STANDAR_LENGTH + number_of_lidars*GS_LIDAR_RECV_PARAMETERS_LENGTH];//store the message
    
    YDSerial->readBytes(captured, (GS_LIDAR_STANDAR_LENGTH + number_of_lidars*GS_LIDAR_RECV_PARAMETERS_LENGTH));
    
    for(int i = 0; i < 4; i++){
        if(captured[i] != GS_LIDAR_HEADER_BYTE){
            return GS_NOT_OK;
        }
    }
    
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
    
   
    if(captured[5] != 0x61) return GS_NOT_OK;//Command type
    if(captured[6] != 0x09) return GS_NOT_OK;//Data length LSB
    if(captured[7] != 0x00) return GS_NOT_OK;//Data length MSB

    for(int i = 0; i<number_of_lidars; i++){
        uint8_t k0_LSB = captured[8 + i*GS_LIDAR_RECV_PARAMETERS_LENGTH];
        uint8_t k0_MSB = captured[9 + i*GS_LIDAR_RECV_PARAMETERS_LENGTH];
        uint8_t b0_LSB = captured[10 + i*GS_LIDAR_RECV_PARAMETERS_LENGTH];
        uint8_t b0_MSB = captured[11 + i*GS_LIDAR_RECV_PARAMETERS_LENGTH];
        uint8_t k1_LSB = captured[12 + i*GS_LIDAR_RECV_PARAMETERS_LENGTH];
        uint8_t k1_MSB = captured[13 + i*GS_LIDAR_RECV_PARAMETERS_LENGTH];
        uint8_t b1_LSB = captured[14 + i*GS_LIDAR_RECV_PARAMETERS_LENGTH];
        uint8_t b1_MSB = captured[15 + i*GS_LIDAR_RECV_PARAMETERS_LENGTH];
        uint8_t bias_LSB = captured[16 + i*GS_LIDAR_RECV_PARAMETERS_LENGTH];

        d_compensateK0_aray[i] = (MSB_LSBtoUINT16(k0_MSB, k0_LSB))/10000.0f;
        d_compensateK1_aray[i] = (MSB_LSBtoUINT16(k1_MSB, k1_LSB))/10000.0f;
        d_compensateB0_aray[i] = (MSB_LSBtoUINT16(b0_MSB, b0_LSB))/10000.0f;
        d_compensateB1_aray[i] = (MSB_LSBtoUINT16(b1_MSB, b1_LSB))/10000.0f;
        bias_array[i] = (bias_LSB)/10;
    }

    YDSerial->read();//Checkbyte

    return GS_OK;
}


void YDLiDar_GS2::setBaudRateToTypical(){
    while(clear_input() != GS_OK){//reassures that the buffer is empty and that that the serial has started
        stopScanningFORCE();
        delay(100);
    }

    //send to every possible baudrate the command to set the baudrate to typical and then restart it
    for(const auto& pair : baudrates){
        close_buffer();
        YDSerial->begin(pair.second);
        delay(60);

        uint8_t byte_to_send = GS_LIDAR_BAUDRATE_921600;
        sendCommand(GS_LIDAR_GLOBAL_ADDRESS, GS_LIDAR_CMD_SET_BAUDRATE, &byte_to_send, GS_LIDAR_RECV_SET_BAUDRATE_LENGTH);
        YDSerial->flush();
        softRestart();
        YDSerial->flush();
    }
    close_buffer();
    delay(10);
    baudrate = 921600;
    YDSerial->begin(921600);
    delay(10);
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
    YDSerial->end();
    YDSerial->begin(baudrate);

    long time = millis();
    while (YDSerial->available() > 0 && millis() - time < 1000){
        YDSerial->read();
    }
    return (YDSerial->available() == 0) ? GS_OK : GS_NOT_OK;
}

inline void YDLiDar_GS2::close_buffer() const{
    YDSerial->end();
}

inline void YDLiDar_GS2::open_buffer() const{
    YDSerial->begin(baudrate);
}

void YDLiDar_GS2::stopScanningFORCE(){
    //send to all possible baud rates to stop scanning
    for(const auto& pair : baudrates){
        YDSerial->begin(pair.second);
        delay(100);
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
    YDSerial->begin(921600);
    delay(100);

    int current_bytes = YDSerial->available();

    delay(10);
    
    //reassures that the lidar does not scan
    if(YDSerial->available() - current_bytes > 0){   
        stopScanningFORCE();
    }else{
        close_buffer();
    }
    delay(10);
    int set_baudrate_to = baudrate;//save the wanted baudrate

    //set the baud rate to 921600
    setBaudRateToTypical();
    
    //check if the desired baudrate exists
    int responces  = 0;
    if(set_baudrate_to != baudrates[GS_LIDAR_BAUDRATE_921600]){
        for(const auto& pair : baudrates){
            if(pair.second == set_baudrate_to){
                setBaudrate(pair.first);
                long start = millis();
                while ((YDSerial->available() < (GS_LIDAR_STANDAR_LENGTH + GS_LIDAR_RECV_SET_BAUDRATE_LENGTH)) && (millis() - start < 1200)){
                    //pass
                }
                if(millis() - start > 1200){
                    return GS_NOT_OK;
                }
                
                current_bytes = YDSerial->available();
                if(YDSerial->available()%10 != 0){
                    fixBuffer();
                }
                if(YDSerial->available()%10 != 0){
                    return GS_NOT_OK;
                }
                responces = (int)(YDSerial->available()/10);
                softRestart();
                break;
            }

        }
    }


    //if the number of kidars are not given they are beung founf from the getNumberOfDevices() 
    if(number_of_lidars == 0){
        this->number_of_lidars = responces;
    }else{
        setNumberofLiDars(number_of_lidars);
    }

    if(this->number_of_lidars == 0){
        return GS_NOT_OK;
    }


    while(YDSerial->available() > 0){
        YDSerial->read();
    }
    
    unsigned long start = millis();
    while(setThecoefficients() != GS_OK){
        if(millis() - start > 3000){
            return GS_NOT_OK;
        }
    }
    
    return GS_OK;
}

GS_error YDLiDar_GS2::setBaudrate(uint8_t Baudrate){
    //checks if the desired baudrate exists
    if(baudrates.find(Baudrate) == baudrates.end()) {
        return GS_NOT_OK;
    }

    //make sure the buffer is open
    open_buffer();
    delay(100);
    
    sendCommand(GS_LIDAR_GLOBAL_ADDRESS, GS_LIDAR_CMD_SET_BAUDRATE, &Baudrate, GS_LIDAR_RECV_SET_BAUDRATE_LENGTH);
    YDSerial->flush();

    YDSerial->end();

    this->baudrate = baudrates[Baudrate];
    YDSerial->begin(baudrates[Baudrate]);

    delay(800);//recomended delay

    softRestart();
    return GS_OK;
}

GS_error YDLiDar_GS2::setedgeMode(uint8_t mode, uint8_t address){
    sendCommand(address, GS_LIDAR_CMD_SET_EDGE_MODE, &mode, GS_LIDAR_RECV_EDGE_MODE_LENGTH);
    YDSerial->flush();
    delay(800);//recomended delay
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

bool YDLiDar_GS2::verifyNumberOfDevices(int number_of_devices){
    int single_lidar_recv_bytes = GS_LIDAR_STANDAR_LENGTH + GS_LIDAR_RECV_ADDRESS_LENGTH;

    clear_input();

    sendCommand(GS_LIDAR_GLOBAL_ADDRESS, GS_LIDAR_CMD_GET_ADDRESS);
    YDSerial->flush();
    delay(800);//recomended delay

    fixBuffer();

    int devices = (int)(YDSerial->available()/single_lidar_recv_bytes);

    return devices == number_of_devices;
}


GS_error YDLiDar_GS2::startScanning(){
    stopScanning();
    delay(100);
    sendCommand(GS_LIDAR_GLOBAL_ADDRESS, GS_LIDAR_CMD_SCAN);
    YDSerial->flush();
    delay(800);
    close_buffer();
    return GS_OK;
}

GS_error YDLiDar_GS2::stopScanning(){
    open_buffer();
    delay(100);
    sendCommand(GS_LIDAR_GLOBAL_ADDRESS, GS_LIDAR_CMD_STOP_SCAN);
    YDSerial->flush();
    delay(100);
    clear_input();
    return GS_OK;
}

iter_Measurement YDLiDar_GS2::iter_measurments(uint8_t dev_address){
    int recv_pos = 0;
    bool successful_capture = false;
    //ensures that the starting bytes are correct
    open_buffer();
    for(int pos = 0; pos < (BYTES_PER_SCAN + 1); pos++){
        if(YDSerial->available()){
            uint8_t currentByte = YDSerial->read();
             switch (recv_pos){
                case 0:
                    if(currentByte != GS_LIDAR_HEADER_BYTE){
                        recv_pos = -1;
                        continue;
                    }
                    break; 
                case 1:
                    if(currentByte != GS_LIDAR_HEADER_BYTE){
                        recv_pos = -1;
                        continue;
                    }
                    break; 
                case 2:
                    if(currentByte != GS_LIDAR_HEADER_BYTE){
                        recv_pos = -1;
                        continue;
                    }
                    break; 
                case 3:
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
                    break;
                case 5:
                    if(currentByte != GS_LIDAR_CMD_SCAN){
                        return iter_Measurement();
                    }
                    break;
                case 6:
                    if(currentByte != 0x42){
                        return iter_Measurement();
                    }
                    break;
                case 7:
                    if(currentByte != 0x01){
                        return iter_Measurement();
                    }
                    break;
            }
            if(recv_pos == 7){
                successful_capture = true;
                break;
            }
            recv_pos++;
        }else{
            pos--;
        }
    }
    if (!successful_capture){
        return iter_Measurement();
    }
    
    recv_pos = 0;
    uint8_t captured[GS_ENV_MEASUREMENT_LENGTH + SCANS_PER_CYCLE*GS_SI_MEASUREMENT_LENGTH];
    while (recv_pos < GS_ENV_MEASUREMENT_LENGTH + SCANS_PER_CYCLE*GS_SI_MEASUREMENT_LENGTH){//captures every measurement bytes are captured
        if(YDSerial->available()){
            captured[recv_pos] = YDSerial->read();
            recv_pos++;
        }
    }    
    close_buffer();

    uint8_t device = dev_address;

    if(device == 0x04){
        device = 0x03;
    }
    device--;

    //locators
    int i = 2*(S_measurement[device]) + 2;
    int n = S_measurement[device];

    iter_Measurement measure;

    measure.env = MSB_LSBtoUINT16(captured[0], captured[1]);

    measure.valid = true;

    uint16_t angle_q6_checkbit;

    //distance and angle correction
    getMeasurements(MSB_LSBtoUINT16(captured[i+1], captured[i]) & 0x01ff, n, &measure.angle, &measure.distance, dev_address);

    

    measure.quality = (captured[i+1] >> 1);

    //filter for incorect captures
    if(measure.angle < 0){
        angle_q6_checkbit = (((uint16_t)(measure.angle * 64 + 23040)) << LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) + LIDAR_RESP_MEASUREMENT_CHECKBIT;
    }else{
        if ((measure.angle * 64) > 23040) {
            angle_q6_checkbit = (((uint16_t)(measure.angle * 64 - 23040)) << LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) + LIDAR_RESP_MEASUREMENT_CHECKBIT;
        } else {
            angle_q6_checkbit = (((uint16_t)(measure.angle * 64)) << LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) + LIDAR_RESP_MEASUREMENT_CHECKBIT;
        }
    }
    if(n < 80){
        if(angle_q6_checkbit <= 23041){
            measure.distance = 0;
            measure.valid = false;
        }
    }else {
        if(angle_q6_checkbit > 23041){
            measure.distance = 0;
            measure.valid = false;
        }
    }

    if(measure.distance < 25 || measure.distance > 300){
        return iter_Measurement();
    }

    

    S_measurement[device] = (S_measurement[device] + 1)%SCANS_PER_CYCLE;
    return iter_Measurement();
}

iter_Scan YDLiDar_GS2::iter_scans(uint8_t dev_address){
    int recv_pos = 0;
    bool successful_capture = false;

    uint8_t check_sum = 0x00;
    uint16_t sample_lens = 0x0000;
    
    uint8_t captured[GS_ENV_MEASUREMENT_LENGTH + SCANS_PER_CYCLE*GS_SI_MEASUREMENT_LENGTH + GS_LIDAR_CHECK_CODE_LENGTH];

    float inc_origin_angle = (float)360.0 / 160;

    //ensures that the starting bytes are correct
    open_buffer();
    unsigned long start = millis();
    for(int pos = 0; pos < (BYTES_PER_SCAN + 1); pos++){
        if(YDSerial->available()){
            start = millis();
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
                    // check_sum = (check_sum + currentByte) & 0xFF;
                    break;
                case 5:
                    if(currentByte != GS_LIDAR_CMD_SCAN){
                        return iter_Scan();
                    }
                    break;
                    // check_sum = (check_sum + currentByte) & 0xFF;
                case 6:
                    if(currentByte != 0x42){
                        return iter_Scan();
                    }
                    // sample_lens |= 0x00ff & currentByte;
                    // check_sum = (check_sum + currentByte) & 0xFF;
                    break;
                case 7:
                    if(currentByte != 0x01){
                        return iter_Scan();
                    }
                    successful_capture = true;
                    // sample_lens |= (0x00ff & currentByte) << 8;
                    // check_sum = (check_sum + currentByte) & 0xFF;
                    break;
            }
            if(recv_pos == 7){
                break;
            }
            recv_pos++;
        }else{
            if(millis() - start > 100){
                return iter_Scan();
            }
            pos--;
        }
        
    }
    if (!successful_capture){
        return iter_Scan();
    }

    recv_pos = 0;
    start = millis();
    while ((recv_pos < GS_ENV_MEASUREMENT_LENGTH + SCANS_PER_CYCLE*GS_SI_MEASUREMENT_LENGTH + GS_LIDAR_CHECK_CODE_LENGTH)){
        recv_pos += YDSerial->readBytes(captured + recv_pos, GS_ENV_MEASUREMENT_LENGTH + SCANS_PER_CYCLE*GS_SI_MEASUREMENT_LENGTH + GS_LIDAR_CHECK_CODE_LENGTH - recv_pos);
        if(millis() - start > 100){
            return iter_Scan();
        }
    }    
    // recv_pos--;
    // Serial.printf("Check sum calculated: %x, actual: %x, loss: %x\n", check_sum, captured[recv_pos], (check_sum - captured[recv_pos]));
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
            scan.distance[n] = 0;
            scan.valid[n] = false;
        }
    }

    return scan;
}


void YDLiDar_GS2::softRestart(){
    sendCommand(GS_LIDAR_GLOBAL_ADDRESS, GS_LIDAR_RECV_SOFT_RESET_LENGTH);
    YDSerial->flush();
    clear_input();
}

int YDLiDar_GS2::getNumberOfDevices(){
    int single_lidar_recv_bytes = GS_LIDAR_STANDAR_LENGTH + GS_LIDAR_RECV_ADDRESS_LENGTH;

    sendCommand(GS_LIDAR_GLOBAL_ADDRESS, GS_LIDAR_CMD_GET_ADDRESS);
    YDSerial->flush();
    delay(800);

    int devices = (int)(YDSerial->available()/single_lidar_recv_bytes);

    uint8_t packet_recv[(devices*single_lidar_recv_bytes)];

    YDSerial->readBytes(packet_recv, (devices*single_lidar_recv_bytes));
    for(int i=0; i<devices; i++){//checks if the first 4 bytes of each responce are actually the header bytes cause if not then there aren't as many devices as believed connected should check lidar connections
        if(!(packet_recv[0+(i*single_lidar_recv_bytes)] == GS_LIDAR_HEADER_BYTE && packet_recv[1+(i*single_lidar_recv_bytes)] == GS_LIDAR_HEADER_BYTE && packet_recv[2+(i*single_lidar_recv_bytes)] == GS_LIDAR_HEADER_BYTE && packet_recv[3+(i*single_lidar_recv_bytes)] == GS_LIDAR_HEADER_BYTE)){
            devices--;
        }
    }
    return devices;
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
    delay(100);
    sendCommand(GS_LIDAR_GLOBAL_ADDRESS, GS_LIDAR_CMD_GET_VERSION);
    YDSerial->flush();

    while(YDSerial->available() < GS_LIDAR_STANDAR_LENGTH + number_of_lidars*GS_LIDAR_RECV_VERSION_LENGTH){
    }

    uint8_t capture[GS_LIDAR_STANDAR_LENGTH + number_of_lidars*GS_LIDAR_RECV_VERSION_LENGTH];

    for(int i = 0; i < 4; i++){
        if(capture[i] != GS_LIDAR_HEADER_BYTE){
            return GS_NOT_OK;
        }
    }
    
    char V[number_of_lidars][GS_LIDAR_RECV_VERSION_NUMBER_LENGTH];
    char SN[number_of_lidars][GS_LIDAR_RECV_SERIAL_NUMBER_LENGTH];

    for(int i = 0; i < number_of_lidars; i++){
        V[i][0] = (char)capture[i*GS_LIDAR_RECV_VERSION_LENGTH + 8];
        V[i][1] = (char)capture[i*GS_LIDAR_RECV_VERSION_LENGTH +9];
        V[i][2] = (char)capture[i*GS_LIDAR_RECV_VERSION_LENGTH +10];

        SN[i][0] = (char)capture[i*GS_LIDAR_RECV_VERSION_LENGTH +11];
        SN[i][1] = (char)capture[i*GS_LIDAR_RECV_VERSION_LENGTH +12];
        SN[i][2] = (char)capture[i*GS_LIDAR_RECV_VERSION_LENGTH +13];
        SN[i][3] = (char)capture[i*GS_LIDAR_RECV_VERSION_LENGTH +14];
        SN[i][4] = (char)capture[i*GS_LIDAR_RECV_VERSION_LENGTH +15];
        SN[i][5] = (char)capture[i*GS_LIDAR_RECV_VERSION_LENGTH +16];
        SN[i][6] = (char)capture[i*GS_LIDAR_RECV_VERSION_LENGTH +17];
        SN[i][7] = (char)capture[i*GS_LIDAR_RECV_VERSION_LENGTH +18];
        SN[i][8] = (char)capture[i*GS_LIDAR_RECV_VERSION_LENGTH +19];
        SN[i][9] = (char)capture[i*GS_LIDAR_RECV_VERSION_LENGTH +20];
        SN[i][10] = (char)capture[i*GS_LIDAR_RECV_VERSION_LENGTH +21];
        SN[i][11] = (char)capture[i*GS_LIDAR_RECV_VERSION_LENGTH +22];
        SN[i][12] = (char)capture[i*GS_LIDAR_RECV_VERSION_LENGTH +23];
        SN[i][13] = (char)capture[i*GS_LIDAR_RECV_VERSION_LENGTH +24];
        SN[i][14] = (char)capture[i*GS_LIDAR_RECV_VERSION_LENGTH +25];
        SN[i][15] = (char)capture[i*GS_LIDAR_RECV_VERSION_LENGTH +26];
    }

    for(int i = 0; i < 19*number_of_lidars; i++){
        int flag = floor(i/19);
        if(flag>=number_of_lidars){
            break;
        }

        if(i<=18){
            if(i<2){
                *(V1+i) = V[0][i];
            }else{
                *(SN1+i) = SN[0][i];
            }
        }else if(i <= 2*18){
            if(i%19<2){
                *(V2+i%19) = V[1][i%19];
            }else{
                *(SN2+i%19) = SN[1][i%19];
            }
        }else if(i <= 3*18){
            if(i%19<2){
                *(V3+i%19) = V[2][i%19];
            }else{
                *(SN3+i%19) = SN[2][i%19];
            }
        }
    }
    
    
    
    return GS_OK;
}
/******************************PUBLIC METHODS******************************/
