#include <SoftwareSerial.h>
#include <Wire.h>
#include <HMC5883L.h>
#include "ESP8266.h"

/***********************************************
 * de5finition
 ***********************************************/

#define DEBUG

/*
 * GPS用の定数 
 * 
 */
#define PIN_GPS_Rx 10
#define PIN_GPS_Tx 11
#define SERIAL_BAUDRATE 9600
#define GPS_BAUDRATE 9600


/* 
 * Wifiモジュール用の定数
 */
#define SSID        "SSID"
#define PASSWORD    "PASSWORD"
#define HOST_NAME   "x.x.x.x"
#define HOST_PORT   (7005)
/*
 * Wifiモジュール 変数
 */
bool wifiConnected = false;

/* 
 * 振動モータ用の定数
 */
#define VIBRATION_TIME 5000
const int vibrationPins[] = {2,3,4,5,6,7,8,9,10};

/*
 * その他定数
 */
#define BUFFER_SIZE 256
#define TIME_INTERVAL 5000
#define TIME_OUT 1000
/*
 * Gpsモジュール用の変数
 */
SoftwareSerial sGps(PIN_GPS_Rx, PIN_GPS_Tx);
bool isValid;

/*
 * Wifiモジュール用の変数
 */
ESP8266 wifi(Serial1);

/*
 * 地磁気センサ用の変数
 */
// Store our compass as a variable.
HMC5883L compass;
// Record any errors that may occur in the compass.
int error = 0;

 
/***********************************************
 * Life Cycle
 ***********************************************/
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  setupGps();
  setupGeomag();
  wifiConnected = setupWifi();
}

void loop() {
  // put your main code here, to run repeatedly:
  //Serial.println(wifi.getIPStatus());
  //analogWrite(VIBRATION1_PIN, 255*3.3/5);
  if(wifiConnected){
    loopForWifi();
    loopForGps();
    loopForWifi();
    loopForGeomag();
    loopForWifi();
    //delay(TIME_INTERVAL);  
  }
}

/***********************************************
 * setup methods
 ***********************************************/
void setupGps(){
  Serial.println("GPS Logger Start!"); 
  
  sGps.begin(GPS_BAUDRATE);

  isValid = false;
}

bool setupWifi(){
    
    Serial.print("setup begin\r\n");
    
    Serial.print("FW Version:");
    Serial.println(wifi.getVersion().c_str());
      
    if (wifi.setOprToStationSoftAP()) {
        Serial.print("to station + softap ok\r\n");
    } else {
        Serial.print("to station + softap err\r\n");
        return false;
    }
 
    if (wifi.joinAP(SSID, PASSWORD)) {
        Serial.print("Join AP success\r\n");
        Serial.print("IP:");
        Serial.println( wifi.getLocalIP().c_str());       
    } else {
        Serial.print("Join AP failure\r\n");
         return false;
    }
    
    if (wifi.disableMUX()) {
        Serial.print("single ok\r\n");
    } else {
        Serial.print("single err\r\n");
         return false;
    }
    
    Serial.print("setup end\r\n");

    int count = 0;
    while (!wifi.createTCP(HOST_NAME, HOST_PORT)) {
        Serial.print("create tcp err\r\n");

        if(count++ > 10)
           return false;
    }

    return true;
    Serial.print("create tcp ok\r\n");
}

void setupGeomag(){
  Serial.println("Starting the I2C interface.");
  Wire.begin(); // Start the I2C interface.
 
  Serial.println("Constructing new HMC5883L");
  compass = HMC5883L(); // Construct a new HMC5883 compass.
    
  Serial.println("Setting scale to +/- 1.3 Ga");
  error = compass.SetScale(1.3); // Set the scale of the compass.   スケールを設定
  if(error != 0) // If there is an error, print it out.
    Serial.println(compass.GetErrorText(error));
  
  Serial.println("Setting measurement mode to continous.");
  error = compass.SetMeasurementMode(Measurement_Continuous); // Set the measurement mode to Continuous
  if(error != 0) // If there is an error, print it out.
    Serial.println(compass.GetErrorText(error));
}

/***********************************************
 * loop methods
 ***********************************************/

void loopForGps(){
  bool result = false;
  int count = 0;
  
  while (!result){ 
    char buf[BUFFER_SIZE];
    getLine(buf);
    result = analyzeData(buf);

    recieveData(TIME_OUT);
    if(count++ > 10)
      break;
  } 
  
}

void loopForGeomag(){
  // Retrive the raw values from the compass (not scaled).
  MagnetometerRaw raw = compass.ReadRawAxis();
  // Retrived the scaled values from the compass (scaled to the configured scale).
  MagnetometerScaled scaled = compass.ReadScaledAxis();
  
  // Values are accessed like so:
  int MilliGauss_OnThe_XAxis = scaled.XAxis;// (or YAxis, or ZAxis)
 
  // Calculate heading when the magnetometer is level, then correct for signs of axis.
  float heading = atan2(scaled.YAxis, scaled.XAxis);
  
  // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
  // Find yours here: http://www.magnetic-declination.com/
  // Mine is: 2&#65533; 37' W, which is 2.617 Degrees, or (which we need) 0.0456752665 radians, I will use 0.0457
  // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
  //  float declinationAngle = 0.0457;
 
  //  弊店所在地における磁気偏角は 、7°18 '(西偏) =127.4 mrad
  //  磁気偏角については http://magnetic-declination.com/とhttp://vldb.gsi.go.jp/sokuchi/geomag/menu_01/index.htmlを参照してください。
  float declinationAngle = 0.1274; 
  
  //  磁気偏角が東偏の場合 -= declinationAngle、西偏の場合 += declinationAngleを使ってください。
  heading += declinationAngle;
  
  // Correct for when signs are reversed.
  if(heading < 0)
    heading += 2*PI;
    
  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;
   
  // Convert radians to degrees for readability.
  float headingDegrees = heading * 180/M_PI; 
 
  // Output the data via the serial port.
//  Output(raw, scaled, heading, headingDegrees);
 
  // Normally we would delay the application by 66ms to allow the loop
  // to run at 15Hz (default bandwidth for the HMC5883L).
  // However since we have a long serial out (104ms at 9600) we will let
  // it run at its natural speed.
  // delay(66);

  String message = "GEOMAG,";
  
  message.concat(String(scaled.XAxis));
  message.concat(",");
  message.concat(String(scaled.YAxis));
  message.concat(",");
  message.concat(String(scaled.ZAxis));
  message.concat(",");
  message.concat(String(heading));
  message.concat(",");
  message.concat(String(headingDegrees));
  
  sendData(message, 1000);
}

void loopForWifi(){
  for (int i = 0; i < 5; i++){
    recieveData(TIME_OUT);
  }
}

 /***********************************************
 * utillity methods
 ***********************************************/

/*
 * TCPでデータを送信
 */
bool sendData(String data, const int timeout){
  Serial.println(data);

  bool result = false;
  
  char message[BUFFER_SIZE]; 
  
  data.toCharArray(message,BUFFER_SIZE);
  result = wifi.send((const uint8_t*)message, strlen(message));
    
   recieveData(TIME_OUT);
    
   return result;
}

/*
 * TCPでデータを送信
 */
void recieveData(const int timeout){
  Serial.println("recieveData start");
  uint8_t buffer[BUFFER_SIZE] = {0};
  char message[BUFFER_SIZE]; 
  char* vibrationNum;
  
  uint32_t len = wifi.recv(buffer, sizeof(buffer), timeout);
  if (len > 0) {
    Serial.println(message);
    if (strncmp("VIBRATION", message, 9) == 0) {
      strtok(message, ",");
      vibrationNum = strtok(NULL, ","); 
      vibrate(vibrationNum);
    }

    sendData("$RESPONSE,success", TIME_OUT); 
    for(uint32_t i = 0; i < len; i++) {
      message[i] = (char)buffer[i];
    }
  }
 
  Serial.println("recieveData end");
}

/*
 * bufにgpsから取得するデータ1行分(改行コード0x0Aまで)の文字列を格納する。 
 * ただし、最大でBUFFER_SIZEの大きさまで。
 */
void getLine(char *buf){
  if(buf == NULL)
    return;
    
  int count = 0;
  
  do {
    if (sGps.available()) {
      buf[count] = sGps.read();
      count++;
    }
    if (count >= BUFFER_SIZE) break;
  } while(buf[count - 1] != 0x0A);
  buf[count] = '\0';
}

/*
 *gpsデータ1行分(buf)を読み込んで解析する。 
 *
 *gpsデータが有効状態のときに、センテンスID=$GPGAAのデータから位置情報を取得してデータを送信する。
 *データ送信に成功したらtrueを返す。
 */

bool analyzeData(char *buf){
  if(buf == NULL)
    return false;
  
  checkValidity(buf);

  if(!isValid)
    return false;
  
  char *gpsTime, *gpsLat, *gpsLatDir, *gpsLon, *gpsLonDir, *gpsTemp; 
  String message = "GPS,";
  
  if (strncmp("$GPGGA", buf, 6) == 0) {
    strtok(buf, ",");
    gpsTime = strtok(NULL, ","); 
    gpsLat = strtok(NULL, ",");  
    gpsLatDir = strtok(NULL, ",");
    gpsLon = strtok(NULL, ","); 
    gpsLonDir = strtok(NULL, ",");

    message.concat(String(gpsLat));
    message.concat(",");
    message.concat(String(gpsLatDir));
    message.concat(",");
    message.concat(String(gpsLon));
    message.concat(",");
    message.concat(String(gpsLonDir));
    message.concat(",");
    message.concat(String(gpsTime));

    //TODO: 送信失敗時の処理
    #ifdef DEBUG
      sendData(message, 1000);
      return true;
    #else
      return sendData(message, 1000);
    #endif
  }

  return false;
}

/*
 * gps情報が有効かの判断をする。
 * 
 * センテンスID=$GPRMCの第２Fieldの値がAのとき、gps情報が有効であると判定する。
 * http://akizukidenshi.com/download/ds/canmore/GMS7-CR6_v1.0_ak.pdf
 */
void checkValidity(char *buf){
  if(buf == NULL){
    isValid = false;
    return;
  }
  
  char *stat;
  if (strncmp("$GPRMC", buf, 6) == 0) {
    char tmp[BUFFER_SIZE];
    strcpy(tmp, buf);
    
    stat = strtok(tmp, ","); 
    stat = strtok(NULL, ","); 
    stat = strtok(NULL, ","); 
    Serial.println(stat);
    
    isValid = (strncmp("A", stat, 1) == 0);
  }
}

void vibrate(char* num){
  String tmp = String(num);
  int n = tmp.toInt();
  Serial.print("vibrate!\nVibration ID:");
  Serial.print(n);
  Serial.print("\n");
  
  if(n >=0 && n < 8){
    analogWrite(vibrationPins[n], 255*3.3/5);

    delay(VIBRATION_TIME);

    analogWrite(vibrationPins[n], 0);
  }
}

