#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WiFiMulti.h>
#include <ESP8266mDNS.h>
#include <ESP8266WebServer.h>
#include <FS.h>   // Include the SPIFFS library
#include <microDS3231.h>
#include <Wire.h>

///////////////////часть кода опроса датчиков

#define SMAARTWIRE_CALIBRATION_SEQ        0x55

// TMP107 регистры 
#define TMP107_REG_TEMPERATURE            0x00
#define TMP107_REG_CONFIGURATION          0x01
#define TMP107_REG_HIGH_LIMIT_1           0x02
#define TMP107_REG_LOW_LIMIT_1            0x03
#define TMP107_REG_HIGH_LIMIT_2           0x04
#define TMP107_REG_LOW_LIMIT_2            0x05
#define TMP107_REG_EEPROM_1               0x06
#define TMP107_REG_EEPROM_2               0x07
#define TMP107_REG_EEPROM_3               0x08
#define TMP107_REG_EEPROM_4               0x09
#define TMP107_REG_EEPROM_5               0x0A
#define TMP107_REG_EEPROM_6               0x0B
#define TMP107_REG_EEPROM_7               0x0C
#define TMP107_REG_EEPROM_8               0x0D
#define TMP107_REG_DIE_ID                 0x0F

//TMP107_регистры конфигурации
#define TMP107_CONV_RATE_15_MS            0b00000000
#define TMP107_CONV_RATE_50_MS            0b00100000
#define TMP107_CONV_RATE_100_MS           0b01000000
#define TMP107_CONV_RATE_250_MS           0b01100000
#define TMP107_CONV_RATE_500_MS           0b10000000
#define TMP107_CONV_RATE_1_S              0b10100000
#define TMP107_CONV_RATE_4_S              0b11000000
#define TMP107_CONV_RATE_16_S             0b11100000

// TMP107 команды
#define TMP107_CMD_ADDRESS_INIT           0x95
#define TMP107_CMD_ADDRESS_ASSIGN         0x0D
#define TMP107_CMD_LAST_DEVICE_POLL       0x57
#define TMP107_CMD_GLOBAL_SOFT_RST        0x5D
#define TMP107_CMD_GLOBAL_ALERT_CLR_1     0xB5
#define TMP107_CMD_GLOBAL_ALERT_CLR_2     0x75
#define TMP107_CMD_GLOBAL_READ            0b00000011
#define TMP107_CMD_GLOBAL_WRITE           0b00000001
#define TMP107_CMD_INDIVIDUAL_READ        0b00000010
#define TMP107_CMD_INDIVIDUAL_WRITE       0b00000000
#define TMP107_CMD_PTR_PADDING            0b10100000

#define NUMBER_OF_SENSORS                 5

#define  TMP_pin   D5          

unsigned short int bw, br;//, i;
char buffer[50] = " ";

/////////////////////////// СЕКЦИЯ КОДА ДЛЯ ОПРОСА ДАТЧИКОВ

int rc;

uint8_t StringLength = 0;
uint64_t AccStringLength = 0;

class SmaartWire {
  public:
    SmaartWire(int pin);
    void begin(long speed);
    void write(uint8_t b);
    uint8_t read();

  private:
    uint16_t _brDelay;
    void writeBit(uint8_t b);
    uint8_t readBit();
};


SmaartWire::SmaartWire(int pin) {
  pinMode(pin, INPUT);
}

void SmaartWire::begin(long speed) {
  //задаем задержку для записи и чтения битов в датчики
  _brDelay = (1000000 / speed)  - 10; 

}

void SmaartWire::write(uint8_t b) {
  uint8_t bitMask;

  noInterrupts();
  pinMode(TMP_pin, OUTPUT);
  digitalWrite(TMP_pin, LOW);

  delayMicroseconds(_brDelay);
  interrupts();

  for (bitMask = 0x01; bitMask; bitMask <<= 1) {
    writeBit((bitMask & b) ? 1 : 0);
  }

  noInterrupts();
  digitalWrite(TMP_pin, HIGH);
  delayMicroseconds(_brDelay);
  pinMode(TMP_pin, INPUT);
  digitalWrite(TMP_pin, LOW);
  interrupts();
}

uint8_t SmaartWire::read() {
  uint8_t bitMask;
  uint8_t r = 0;

  uint32_t start = millis();
  while ((digitalRead(TMP_pin) == 1) && (millis() - start < _brDelay * 10));
  delayMicroseconds(_brDelay + _brDelay / 2);

  for (bitMask = 0x01; bitMask; bitMask <<= 1) {
    if (readBit()) {
      r |= bitMask;
    }
  }

  return r;
}

void SmaartWire::writeBit(uint8_t b) {
  if (b & 1) {
    noInterrupts();
    pinMode(TMP_pin, OUTPUT);
    digitalWrite(TMP_pin, HIGH);
    interrupts();
    delayMicroseconds(_brDelay - 10);
  } else {
    noInterrupts();
    digitalWrite(TMP_pin, LOW);
    pinMode(TMP_pin, OUTPUT);
    interrupts();
    delayMicroseconds(_brDelay - 10); // это  общая задержка
  }
}

uint8_t SmaartWire::readBit() {
  uint8_t r;
  noInterrupts();
  pinMode(TMP_pin, INPUT);
  if (digitalRead(TMP_pin) == HIGH)
  {
    r = 0b11111111;
  }
  if (digitalRead(TMP_pin) == LOW) {
    r = 0b00000000;
  }
  interrupts();
  delayMicroseconds(_brDelay - 10);

  return r;
}
SmaartWire tmp107(TMP_pin);

// массив адресов датчиков
byte addr[NUMBER_OF_SENSORS];

// функция инициализации датчиков
// Важно! эта функция вызывввается до работы с датчиками
void addrInit(byte* addr) {
  // калибровочная фаза
  tmp107.write(SMAARTWIRE_CALIBRATION_SEQ);

  // фаза команд
  tmp107.write(TMP107_CMD_ADDRESS_INIT);

  // задаем бит адреса
  tmp107.write(TMP107_CMD_ADDRESS_ASSIGN);

  // считывание данных
  for (unsigned short int i = 0; i < NUMBER_OF_SENSORS; i++) {
    addr[i] = (tmp107.read() & 0b11111000) >> 3;
  }

  // задержка ожидани инициализации 
  delay(1250);
}

// функция чтения темпераутры по выбранному адресу
float getTemp(byte addr) {
  // калибровочная фаза
  tmp107.write(SMAARTWIRE_CALIBRATION_SEQ);

  // фаза команды и адреса
  tmp107.write(TMP107_CMD_INDIVIDUAL_READ | (addr << 3));

  // 
  tmp107.write(TMP107_REG_TEMPERATURE | TMP107_CMD_PTR_PADDING);

  // считываем данные
  byte tmpLSB = tmp107.read();
  byte tmpMSB = tmp107.read();

  // конвертируем 14-битные данные в температуру
  int raw;
  float temp;
  if (tmpMSB & 0b10000000) {
    // при отрицательной температуре
    raw = ~((tmpMSB << 6) | (tmpLSB >> 2)) + 1;
    raw &= 0x3FFF;
    temp = (float)raw * -0.015625;
  } else {
    // если положительная, то просто выводим её
    raw = (tmpMSB << 6) | (tmpLSB >> 2);
    temp = (float)raw * 0.015625;
  }

  return (temp);
}

// функция для установки интервала измерения
void setInterval(byte lastAddr, byte convRate) {
  // фаза калибровки
  tmp107.write(SMAARTWIRE_CALIBRATION_SEQ);

  // фаза команд и адресов
  tmp107.write(TMP107_CMD_GLOBAL_WRITE | (lastAddr << 3));

  // фаза указателя регистра
  tmp107.write(TMP107_REG_CONFIGURATION | TMP107_CMD_PTR_PADDING);

  // фаза данных
  tmp107.write(0x00);
  tmp107.write(convRate);
}

MicroDS3231 rtc; //иниаицлизация часов времени

char* printTime() {

  DateTime now = rtc.getTime();
  char buf[30];
  snprintf(buf, sizeof(buf), "%02d-%02d-%02d %02d:%02d:%04d",
           
           now.year, now.month, now.date,
           now.hour, now.minute, now.second);
  return (buf);
}



//////////////////////////////////////////////////////////\\


const char * javascriptCode1 = " <!DOCTYPE html> "

"<html> "

"<body> "

"<p>Click the button to get a message from the ESP8266:</p> "

"<button onclick=\"buttonFunction()\">Message</button> "

"<script>"

"function buttonFunction() { "

" alert(\"Hello from the ESP8266!\"); "

"} "

"</script>"

"</body> "

"</html> ";

//=
//ESP8266WiFiMulti wifiMulti;     //Создайте экземпляр класса ESP8266WiFiMulti с именем 'wifiMulti'
/* Установите здесь свои SSID и пароль */
const char* ssid = "MyLogger";       // SSID
const char* password = "12345678";  // пароль

IPAddress local_ip(192,168,0,10);  //статический IP
IPAddress gateway(192,168,0,1);
IPAddress subnet(255,255,255,0);



ESP8266WebServer server(80);    // Создайте объект веб-сервера, который прослушивает HTTP-запрос на порту 80

File fsUploadFile;              // файловый объект для временного хранения полученного файла

String getContentType(String filename); // преобразование файла в нужный тип
bool handleFileRead(String path);       // отправить нужный файл клиенту (если он существует)
void handleFileUpload();                // загрузить новый файл а  SPIFFS



int nowTime;

///////////////// Функция старта!
void setup() {
   delay(1000);
  Serial.begin(9600);         // инициализируем ком порт
  delay(100);

  //////////////
unsigned short int avarage = 0;
  int Temp_now1, Temp_now2;
  //do {
    delay(100);
    tmp107.begin(9600);

    Serial.println("Initializing rtc");
 //rtc.setTime(BUILD_SEC, BUILD_MIN, BUILD_HOUR, BUILD_DAY, BUILD_MONTH, BUILD_YEAR);
  
  if (rtc.lostPower()) {            // выполнится при сбросе батарейки
    Serial.println("lost power!");
    // тут можно однократно установить время == времени компиляции    
    rtc.setTime(BUILD_SEC, BUILD_MIN, BUILD_HOUR, BUILD_DAY, BUILD_MONTH, BUILD_YEAR);
  }

//блок с модулем часов. читаем файл с сервера
SPIFFS.begin();
String line = "";
if(SPIFFS.exists("/RTC.txt"))
{
  Serial.println("Файл существует");
}

  File file = SPIFFS.open("/RTC.txt", "r+");
  

  if(file){ // иначе 
    // мы мо;ем открыть файл
    while(file.available()) {
      Serial.println("Файл открыт");
      //читаем строку за строкой в файле
      line = file.readStringUntil('\n');
      Serial.println(line);
    }
      file.close();
      SPIFFS.remove("/RTC.txt"); //удаляем файл
      if(line.length() != 0) //если строка не пустая
     {
        String part01 = getValue(line,' ',0); //сек
        String part02 = getValue(line,' ',1); //мин   
        String part03 = getValue(line,' ',2); //часы

        String part04 = getValue(line,' ',3); //день
        String part05 = getValue(line,' ',4); //месяц
        String part06 = getValue(line,' ',5); //год
Serial.println(part01);
Serial.println(part02);
Serial.println(part03);
Serial.println(part04);
Serial.println(part05);
Serial.println(part06);

//rtc.setTime(part01.toInt(), part02.toInt(), part03.toInt(), part04.toInt(), part05.toInt(), part06).toInt();

 // также можно установить время через DateTime

  DateTime now;
  now.second = part01.toInt();
  now.minute = part02.toInt();//.toInt()
  now.hour = part03.toInt();
  now.date = part04.toInt();
  now.month = part05.toInt();
  now.year = part06.toInt();
  
  rtc.setTime(now);  // загружаем в RTC

    }
    
   //  return;
}
    Serial.println("Initializing sensors");
 delay(100);
    addrInit(addr);
    setInterval(addr[NUMBER_OF_SENSORS - 1], TMP107_CONV_RATE_100_MS);
    float Temp_now = 0;

  Serial.println('\n');


Serial.print("Setting soft-AP configuration ... ");
Serial.println(WiFi.softAPConfig(local_ip, gateway, subnet) ? "Ready" : "Failed!");

Serial.print("Setting soft-AP ... ");
Serial.println(WiFi.softAP("MyLoggerPredDiplom") ? "Ready" : "Failed!");


IPAddress IP = WiFi.softAPIP();
Serial.print("AP IP address: ");
Serial.println(IP);
 
  
  Serial.println('\n');
  Serial.print("Connected to ");
  Serial.println(WiFi.SSID());              // Говорит нам, к какой сети мы подключены
  Serial.print("IP address:\t");
  Serial.println(WiFi.localIP());           // отправить IP-адрес в комп

  if (MDNS.begin("esp8266")) {              // Запускаем ответчик mDNS для esp8266.local
    Serial.println("mDNS responder started");
  } else {
    Serial.println("Error setting up MDNS responder!");
  }

  

 // SPIFFS.begin();                           // начало SPI Flash Files System

  server.on("/upload", HTTP_GET, []() {                 //если клиент запрашивает страницу загрузки
    if (!handleFileRead("/upload.html"))                // отправляем его, если он существует
      server.send(404, "text/plain", "404: Not Found"); // в противном случае  404 (не найдено)
  });

   

  server.on("/upload", HTTP_POST,                       // если клиент перешел на страницу загрузки
    [](){},
    handleFileUpload                                    // Получаем и сохраняем файл
  );


///часть кода обработки других кнопок

///обработка загрузки
  server.on("/download", HTTP_GET, []() {                 // if the client requests the upload page
    if (!handleFileRead("/download.html"))                // send it if it exists
      server.send(404, "text/plain", "404: Not Found"); // otherwise, respond with a 404 (Not Found) error
  });
 server.on("/download", HTTP_POST,                       // if the client posts to the upload page
    [](){},
    handleRoot2                                    // Receive and save the file
  );

 server.on("/fileInDir", HTTP_GET, []() {                 // if the client requests the upload page
    if (!handleFileRead("/Graf.html"))                // send it if it exists
      server.send(404, "text/plain", "404: Not Found"); // otherwise, respond with a 404 (Not Found) error
  });
 server.on("/fileInDir", HTTP_POST,                       // if the client posts to the upload page
    [](){},
    openDirInBrows                                   // Receive and save the file
  );

  server.on("/javascript", HTTP_GET, []() {                 // if the client requests the upload page
javascriptCode();

});


  server.onNotFound([]() {                              // If the client requests any URI
    if (!handleFileRead(server.uri()))                  // send it if it exists
      server.send(404, "text/plain", "404: Not Found"); // otherwise, respond with a 404 (Not Found) error
  });
Write_SDcard();
Write_SDcard();
  server.begin();                           // Actually start the server
  Serial.println("HTTP server started");
  nowTime = rtc.getMinutes();//now.minute;
}

///////////////////
void loop() {
  //DateTime now = rtc.getTime();
  
  server.handleClient();
  //now = rtc.getTime();
   
  delay(100);
  //int newNowTime = rtc.getMinutes();//now.minute;
  //if((newNowTime-nowTime)>2||((newNowTime-nowTime)<0))
  if((nowTime < (rtc.getMinutes()-2)) || ((nowTime - rtc.getMinutes())>50)) //если текущее время больше заданного на 2м
  {
    Write_SDcard();
   //delay(10000);
   nowTime =rtc.getMinutes();
    }
}

/////////////////////////////


String getContentType(String filename) { // convert the file extension to the MIME type
  if (filename.endsWith(".html")) return "text/html";
  else if (filename.endsWith(".css")) return "text/css";
  else if (filename.endsWith(".js")) return "application/javascript";
  else if (filename.endsWith(".ico")) return "image/x-icon";
  else if (filename.endsWith(".gz")) return "application/x-gzip";
  return "text/plain";
}

bool handleFileRead(String path) { // send the right file to the client (if it exists)
  Serial.println("handleFileRead: " + path);
  if (path.endsWith("/")) path += "index.html";          // If a folder is requested, send the index file
  String contentType = getContentType(path);             // Get the MIME type
  String pathWithGz = path + ".gz";
  if (SPIFFS.exists(pathWithGz) || SPIFFS.exists(path)) { // If the file exists, either as a compressed archive, or normal
    if (SPIFFS.exists(pathWithGz))                          // If there's a compressed version available
      path = pathWithGz;                                      // Use the compressed verion
    File file = SPIFFS.open(path, "r");                    // Open the file
    size_t sent = server.streamFile(file, contentType);    // Send it to the client
    file.close();                                          // Close the file again
    Serial.println(String("\tSent file: ") + path);
    return true;
  }
  Serial.println(String("\tFile Not Found: ") + path);   // If the file doesn't exist, return false
  return false;
}

void handleFileUpload(){ // upload a new file to the SPIFFS
  HTTPUpload& upload = server.upload();
  if(upload.status == UPLOAD_FILE_START){
    String filename = upload.filename;
    if(!filename.startsWith("/")) filename = "/"+filename;
    Serial.print("handleFileUpload Name: "); Serial.println(filename);
    fsUploadFile = SPIFFS.open(filename, "w");            // Open the file for writing in SPIFFS (create if it doesn't exist)
  } else if(upload.status == UPLOAD_FILE_WRITE){
    if(fsUploadFile)
      fsUploadFile.write(upload.buf, upload.currentSize); // Write the received bytes to the file
  } else if(upload.status == UPLOAD_FILE_END){
    if(fsUploadFile) {                                    // If the file was successfully created
      fsUploadFile.close();                               // Close the file again
      Serial.print("handleFileUpload Size: "); Serial.println(upload.totalSize);
      server.sendHeader("Location","/success.html");      // Redirect the client to the success page
      server.send(303);
    } else {
      Serial.println("File upload failed");
      server.send(500, "text/plain", "500: couldn't create file");
    }
  }
}


void startWeb()
{
  //String s = FPSTR(MAIN_page);
  SPIFFS.begin();
  File f = SPIFFS.open("/myTXT.txt", "r");
  server.sendHeader("Location","/download.html"); 
  SPIFFS.end();
  }

void handleRoot() {
  SPIFFS.begin();  
  File f = SPIFFS.open("/myTXT.txt", "a+");
  server.sendHeader("Location","/success.html");
   server.send(303);
  SPIFFS.end();
}


void handleRoot3() {
  SPIFFS.begin();
  File f = SPIFFS.open("/myTXT.txt", "r");
  size_t sent = server.streamFile(f, "text/txt");
  f.close();
  if(!f){
  Serial.print("выгрузка завершена ");
  server.sendHeader("Location","/success.html");
  }
  SPIFFS.end();
}

void handleRoot2() {
  SPIFFS.begin();
  File file = SPIFFS.open("/download", "r");                    // Open the file
    //size_t sent = server.streamFile(file, "html");    // Send it to the client
     server.sendHeader("Location","/success.html");      // Redirect the client to the success page
      server.send(303);
    file.close();
    //server.send(200);  
  SPIFFS.end();
}

void openDirInBrows() {
  String s = "";
}


void javascriptCode()
{
String Tempnow1 =String(getTemp(addr[0]));
String Tempnow2 =String(getTemp(addr[1]));
String Tempnow3 =String(getTemp(addr[2]));
String Tempnow4 =String(getTemp(addr[3]));
String Tempnow5 =String(getTemp(addr[4]));

  String s ="";
  s+=" <!DOCTYPE html> ";

s+="<html> ";
s+="<html lang=\"ru-RU\">";
s+="<head>";
s+="<meta charset=\"UTF-8\" />";
  s+="<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">";
   s+=" <title>Начальная страница работы с логгером</title>";
   s+=" <link rel=\"stylesheet\" type=\"text/css\" href=\"main.css\">";
s+="</head>";


s+="<body> ";
  s+="<canvas width=\"500\" height=\"500\" id=\"canvas\"></canvas>";
  // Получаем элемент canvas

 
// Назначаем тип рисования
//s+="<button onclick=\"buttonFunction()\">Message</button> ";

s+="<script>";


s+="var data = [";
s+=Tempnow1;
s+=",";
s+=Tempnow2;
s+=",";
s+=Tempnow3;
s+=",";
s+=Tempnow4;
s+=",";
s+=Tempnow5;
s+="];"; //вот здесь должна быть запись показаний с датчиков
  // получаем указатель на холст
s+="var canvas = document.getElementById(\'canvas\');"; 
  
// получаем указатель на контекст рисования
s+="var c = canvas.getContext(\'2d\'); ";
  
// рисуем фон
s+="c.fillStyle = \"white\"; ";
s+="c.fillRect(0,0,500,500); ";
  
// рисуем данные
s+="c.fillStyle = \"green\"; ";
s+="for(var i=0; i<data.length; i++) { ";
s+=" var dp = data[i]; ";
s+="  if(dp<=-0)";
s+="  {c.fillStyle = \"turquoise\";}";
s+="  if(dp<=-20)";
s+="  {c.fillStyle = \"blue\";}";
s+="  if(dp>=0)";
s+="  {c.fillStyle = \"orange\";}";
s+="  if(dp>=20)";
s+="  {c.fillStyle = \"green\";}";
s+="  c.fillRect(40 + i*60, 245-dp*4 , 50, dp*4); ";
s+="}";
// рисуем осевые линии
s+="c.fillStyle = \"black\"; ";
s+="c.lineWidth = 3.0; ";
s+="c.beginPath(); ";
s+="c.moveTo(30,5); ";
s+="c.lineTo(30,490); ";
s+="c.moveTo(30,245);";
s+="c.lineTo(490,245); ";
s+="c.stroke();";


// рисуем текст и вертикальные линии
s+="c.fillStyle = \"black\"; ";
s+="for(var i=0; i<7; i++) { ";
s+="  c.fillText((6-i)*10 + \"C\",5, i*40+5); ";
s+="  c.beginPath(); ";
s+="  c.moveTo(25,i*40+5); ";
s+="  c.lineTo(30,i*40+5); ";
s+="  c.stroke(); ";
s+="}";

s+="for(var i=7; i<13; i++) { ";
s+="  c.fillText(0-(i-6)*10 + \"C\",5, i*40+5); ";
s+="  c.beginPath(); ";
s+="  c.moveTo(25,i*40+5); ";
s+="  c.lineTo(30,i*40+5); ";
s+="  c.stroke(); ";
s+="}";

s+="var labels = [\"0,3-m\",\"0,6-m\",\"0,9-m\",\"1,2-m\",\"1,5-m\"]; ";
// выводим текст
s+="for(var i=0; i<5; i++) { ";
s+="  c.fillText(labels[i], 50+ i*60, 265); ";
s+="}";




s+="</script>";
s+="<p>показания с датчиков:";
s+="</p>";
s+="<p>";
s+="Дата и Время:  ";
s+=rtc.getDateString();
s+="  ";
s+=rtc.getTimeString();
s+="</p>";
s+="<p>";
s+=Tempnow1;
s+="</p>";
s+="<p>";
s+=Tempnow2;
s+="</p>";
s+="<p>";
s+=Tempnow3;
s+="</p>";
s+="<p>";
s+=Tempnow4;
s+="</p>";
s+="<p>";
s+=Tempnow5;
s+="</p>";
s+="</body> ";
s+="</html> ";
//return (s);
server.send(200, "text/html", s);
  }


int count = 0; //счетчик измерений

  void Write_SDcard() //
{
  SPIFFS.begin();
  File f = SPIFFS.open("/LOG.txt", "a"); //открыл файл для дозаписи или его создания
    if (!f) {
      Serial.println("file open failed"); //если файл не открылся
    }
    delay(50);
  float Temp_now = 0;
  for (unsigned short int i = 0; i < NUMBER_OF_SENSORS; i++) {
    delay(100);
    bw = 0;
    Temp_now = getTemp(addr[i]);
    int Temp_now1 = Temp_now;
    int Temp_now2 = ((Temp_now) * 100);
    Temp_now2 = Temp_now2 % 100;
    count++;
  String TimeNow = rtc.getDateString();
  String DateNow = rtc.getTimeString();
  //Serial.println(rtc.getTimeString());
  Serial.println(DateNow);
  //Serial.println(rtc.getDateString());
  Serial.println(TimeNow);
    Serial.print("адрес "); 
    Serial.println(i+1); 
    Serial.print("Temrerature"); 
    Serial.print(Temp_now1);
    Serial.print(".");
    Serial.println(Temp_now2);
    f.printf("%s %s %d.%d \n",DateNow,TimeNow, Temp_now1,Temp_now2 );
    delay(150);
  }
  f.close();  //Close file
}

void ReadFile()
{
  File f = SPIFFS.open("/LOG.txt", "r");
  
  if (!f) {
    Serial.println("file open failed");
  }
  else
  {
      Serial.println("Reading Data from File:");
      //Data from file
      for(int i=0;i<f.size();i++) //Read upto complete file size
      {
        Serial.print((char)f.read());
      }
      f.close();  //Close file
      Serial.println("File Closed");
  }
  delay(5000);
  }



String getValue(String data, char separator, int index)//разбивает строку на подстроки через разделитель
{
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length()-1;

  for(int i=0; i<=maxIndex && found<=index; i++){
    if(data.charAt(i)==separator || i==maxIndex){
        found++;
        strIndex[0] = strIndex[1]+1;
        strIndex[1] = (i == maxIndex) ? i+1 : i;
    }
  }

  return found>index ? data.substring(strIndex[0], strIndex[1]) : "";
}
