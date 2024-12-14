#include <YDLiDar_gs2.h>

YDLiDar_GS2 lidar(&Serial1);//give the Serial1

void setup() {
  Serial.begin(115200);

  GS_error status = lidar.initialize(1);//initialise 1 device

  while(status != GS_OK){
    Serial.println("There was an error while initializing the lidar tryiing again");
    status = lidar.initialize(1);
  }
  //avoid K0 , K1, ... some boards have pre-defined macros named this way
  double d_K0, d_B0, d_K1, d_B1, Bias;
  lidar.getParametes(0x01, &d_K0, &d_B0, &d_K1, &d_B1, &Bias);//get the coefficients
  Serial.printf("%f,%f,%f,%f,%f\n",d_K0, d_B0, d_K1, d_B1, Bias);
  lidar.startScanning();//start the scanning
}

void loop() {
  while(true){
    iter_Scan scan = lidar.iter_scans();//get multiple measurments
    Serial.println();
    for(int i=0;i<MAX_SCAN; i++){
      if(!scan.valid[i]){//check
        continue;
      }
      Serial.printf("DIST: %i mm, Angle: %f, quality: %i, env: %i\n", i, scan.distance[i], scan.angle[i], scan.quality[i], scan.env);
    }
    delay(10);//minimal delay
  }

}

