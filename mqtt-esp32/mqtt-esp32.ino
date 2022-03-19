#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <EEPROM.h>
#include <ESP32WebServer.h>
#include "SPIFFS.h"
#include "DHT.h"
#include <Servo.h>

/**==============================================================
 * ##### Hardware pin map define #####
 *==============================================================*/
#define LED_PIN         2                     // Blue LED on board

/* DHT11 */
#define DHT_TYPE        DHT11                 // DHT11 sensor 
#define DHT_PIN         4                     // Pin read value connection 

#define DEVICE_PIN     13                     // Servo pin connect 13 

#define SSID_MAX_SIZE   10                    // Max size of ssid 
#define PASS_MAX_SIZE   15                    // Max size of password 
#define EEPROM_MEM      512                   // EEPROM size 

#define AP_SSID         "AP Mode"             // AP wifi's name 
#define AP_PSK          "1234567890"          // AP wifi 's password 

#define STA_SSID        "IOT_LS_N"            // STA wifi's name 
#define STA_PSK         "ftelecomiot1"        // STA wifi's password 

#define MQTT_SERVER     "192.168.1.120"       // IP server broker running 
#define LISTENER        1883                  // Port listener 

#define TOPIC_SUB       "CONTROL"
#define TOPIC_PUB       "SENSOR"

typedef enum {
  SSID_TYPE = 0,
  PASS_TYPE,
} EEPROM_Type_t;

typedef enum {
  TEMPERATURE = 0,
  HUMIDITY,
} DHT11_Return_t;

typedef enum {
  DEVICE_OPEN = 0,
  DEVICE_CLOSE,
  LED_ON,
  LED_OFF,
} Device_status_t;

typedef enum {
  MODE_AUTO = 0,
  MODE_MANUAL,
} Mode_control_t;

/**==============================================================
 * ##### Global variables #####
 *==============================================================*/
WiFiClient espClient;
PubSubClient client(espClient);
ESP32WebServer server(80);

DHT dht(DHT_PIN, DHT_TYPE);
Servo device;
Device_status_t device_status;
Device_status_t led_status;
Mode_control_t mode_control = MODE_MANUAL;

const char * g_sta_ssid = STA_SSID;
const char * g_sta_pass = STA_PSK;

const char * g_ap_ssid = AP_SSID;
const char * g_ap_pass = AP_PSK;

const char * mqtt_server = MQTT_SERVER;
int port = LISTENER;

const char * topic_subcribe = TOPIC_SUB;
const char * topic_publish = TOPIC_PUB;

long lastMsg = 0;
bool flag_sta_mode = false;

/**==============================================================
 * ##### Function prototypes #####
 *==============================================================*/
/* LED */
void led_setup(void);
void led_on(void);
void led_off(void);

/* DHT11 */
void dht11_setup(void);
float dht11_read(DHT11_Return_t ret);

/* Servo */
void device_setup(void);
void device_control(Device_status_t state);

/* Mode control */
void set_mode_control(Mode_control_t mod);


void wifi_ap_setup(void);                 // Set up wifi AP mode 
bool wifi_sta_isvalid(void);              // Check wifi STA mode 
void eeprom_clear_one_shot(void);         // Clear EEPROM first time when upload to ESP32 
String eeprom_return(EEPROM_Type_t typ);  // Get EEPROM value 

/* Reconnect MQTT */
void reconnect(void);                     
/* MQTT callback */
void callback(char* topic, byte* message, unsigned int length); 
void handle_topic_msg(char * topic, String msg);

/* Webserver handler request */
void handleROOT(void);
void handleLOGIN(void);   
void handleSUBMIT(void);


/**
 * =================
 * @function: setup
 */
void setup() {
  /* Hardware setup */
  Serial.begin(115200);     /* Enable UART communicate with baudrate 115200 */
  SPIFFS.begin();           /* Read files */
  EEPROM.begin(512);        /* EEPROM */
  
  led_setup();
  dht11_setup();
  device_setup();  
  
  /* Software setup */
  eeprom_clear_one_shot();
  wifi_ap_setup();

  /* Check wifi fill in from EEPROM */
  String sta_ssid = eeprom_return(SSID_TYPE);
  String sta_pass = eeprom_return(PASS_TYPE);

  Serial.print("[SET_UP] STA WIFI SSID: ");
  Serial.println(sta_ssid);
  Serial.print("[SET_UP] STA WIFI PASS: ");
  Serial.println(sta_pass);
  
  if (sta_ssid.equals(g_sta_ssid) && sta_pass.equals(g_sta_pass))
  {
    Serial.println("[SET_UP] WIFI INFORMATION");
    Serial.print("[SET_UP] STA WIFI SSID: ");
    Serial.println(sta_ssid);
    Serial.print("[SET_UP] STA WIFI PASS: ");
    Serial.println(sta_pass);
  }

  /* STA mode transfer */
  Serial.println("[SET_UP] BEGIN STA MODE TRANSFER");
  WiFi.begin(sta_ssid.c_str(), sta_pass.c_str());

  if (wifi_sta_isvalid() == true)
  {
    Serial.println("[SET_UP] STATION MODE IS VALID");
    Serial.println("[SET_UP] WIFI CONNECTED");
    Serial.println("[SET_UP] IP ADDRESS: ");
    Serial.println(WiFi.localIP());
    flag_sta_mode =  true;

    /* Connect to mosquito MQTT broker */
    client.setServer(mqtt_server, port);
    client.setCallback(callback);
  }
  else
  {
    /* Come back to AP mode */
    wifi_ap_setup();
    server.on("/", handleROOT);
    server.on("/login.html",handleLOGIN);
    server.on("/submit.html", handleSUBMIT);
    server.begin();
    
    Serial.println("HTTP server started");   
  }
}

/**
 * =================
 * @function: loop
 */
void loop() {
  if (flag_sta_mode == true && !client.connected()) {
    reconnect();
  }
  client.loop();

  server.handleClient();

  static bool publish_one_shot = false;
  if (mode_control == MODE_MANUAL)
  {
    /* TO DO */
    if (publish_one_shot == false)
    {
      client.publish(topic_publish, "MODE MANUAL IS READY");
      publish_one_shot = true;
    }
  }
  else /* mode auto */
  {
    publish_one_shot = false;
    
    /* Publish msg */
    long now = millis();
    if (now - lastMsg > 5000) {
      lastMsg = now;
      
      /* update information device & sensor */
      float t = dht11_read(TEMPERATURE);  
      float h = dht11_read(HUMIDITY);
      Serial.print("[LOOP] Temp: ");
      Serial.println(t);
      Serial.print("[LOOP] Humi: ");
      Serial.println(h);

      String mes = "TEMPERATURE ";
      mes += String(t);
      mes += " - HUMIDITY: ";
      mes += String(h);
      client.publish(topic_publish, (char *)mes.c_str());
  
      Serial.print("[LOOP] Device: ");
      if (device_status == DEVICE_OPEN)
      {
        Serial.println("[LOOP] DEVICE_OPEN");
        client.publish(topic_publish, "DEVICE OPEN");
      }
      else
      {
        Serial.println("[LOOP] DEVICE_CLOSE");
        client.publish(topic_publish, "DEVICE CLOSE");
      }
  
      Serial.print("[LOOP] LED: ");
      if (led_status == LED_ON)
      {
        Serial.println("[LOOP] LED_ON");
        client.publish(topic_publish, "LED ON");
      }
      else
      {
        Serial.println("[LOOP] LED_OFF");
        client.publish(topic_publish, "LED OFF");
      }   
    }
  }  
}



/**
 * ====================
 * @function: led_setup
 */
void led_setup()
{
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  led_status = LED_OFF;
}

/**
 * ===================
 * @function: led_on
 */
void led_on()
{
  digitalWrite(LED_PIN, HIGH);
  led_status = LED_ON;
}

/**
 * ===================
 * @function: led_off
 */
void led_off()
{
  digitalWrite(LED_PIN, LOW);
  led_status = LED_OFF;
}

/**
 * ======================
 * @function: dht11_setup
 */
void dht11_setup()
{
  dht.begin();
}

/**
 * ======================
 * @function: dht11_read
 */
float dht11_read(DHT11_Return_t ret)
{
  float val_ret = 0;
  switch(ret)
  {
    case TEMPERATURE: {
      val_ret = dht.readTemperature(); 
    }
    break;

    case HUMIDITY: {
      val_ret = dht.readHumidity(); 
    }
    break;

    default:
    break;
  }
  
  /* check is valid value ? */
  if (isnan(val_ret)) {
    Serial.println(F("[DHT11_READ] Failed to read from DHT sensor!"));
    return 0;
  }
  
  return val_ret;
}

/**
 * =======================
 * @function: device_setup
 */
void device_setup()
{
  device.attach(DEVICE_PIN);
  device_status = DEVICE_CLOSE;
}

/**
 * ==========================
 * @function: device_control
 */
void device_control(Device_status_t state)
{
  static int stat = 0;
  switch(state)
  {
    case DEVICE_OPEN: {
      do
      {
        ++stat;
        device.write(stat);
        delay(20);
      } while (stat < 360);
      
      device_status = DEVICE_OPEN;
    }
    break;

    case DEVICE_CLOSE: {
      do
      {
        --stat;
        device.write(stat);
        delay(20);
      } while (stat > 0);

      device_status = DEVICE_CLOSE;
    }
    break;

    default:
    break;
  }
}

/**
 * ===========================
 * @function: set_mode_control
 */
void set_mode_control(Mode_control_t mod)
{
  if (mod == MODE_AUTO)
  {
    mode_control = MODE_AUTO;
  }
  else 
  {
    mode_control = MODE_MANUAL;
  }
}

/**
 * ========================
 * @function: wifi_ap_setup
 */
void wifi_ap_setup() {
  delay(10);
  WiFi.mode(WIFI_AP);
  Serial.println("[WIFI_AP_SETUP] ACCESS POINT MODE");

  /* Wifi AP mode */
  WiFi.softAP(g_ap_ssid, g_ap_pass);
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("[WIFI_AP_SETUP] AP IP address: ");
  Serial.println(myIP);
}

/**
 * ===========================
 * @function: wifi_sta_isvalid
 */
bool wifi_sta_isvalid() {
  int t = 0;
  Serial.println("[WIFI_STA_ISVALID] Wait for connection (10s) .......");
  
  while (t < 20) {
    if (WiFi.status() == WL_CONNECTED)
    {
      return true;
    }
    delay(500);
    Serial.print(".");
    ++t;
  }
  
  Serial.println("");
  Serial.println("[WIFI_STA_ISVALID] Time out.....");  
  return false;
}

/**
 * ===============================
 * @function: eeprom_clear_one_shot
 */
void eeprom_clear_one_shot()
{
  byte val = EEPROM.read(EEPROM_MEM - 1);
  if (val != 1)
  {
    Serial.println("[EEPROM_CLEAR_ONE_SHOT]");
    EEPROM.write(EEPROM_MEM - 1, 1);
    for (int i = 0; i < EEPROM_MEM - 1; i++)
    {
      EEPROM.write(i,0);
    }
  }
}

/**
 * ======================
 * @function: eeprom_return
 */
String eeprom_return(EEPROM_Type_t typ)
{
  String s_ret = "";
  switch(typ)
  {
  case SSID_TYPE: {
    byte val;
    int k = 0;
    
    do{
      val = EEPROM.read(k);
      if (val != 0){
        s_ret += char(val);
      }
      ++k;
    }while (val != 0 && k < SSID_MAX_SIZE);
  }
  break;

  case PASS_TYPE: {
    byte val;
    int k = 10;
    
    do{
      val = EEPROM.read(k);
      if (val != 0){
        s_ret += char(val);
      }
      ++k;
    }while (val != 0 && k < PASS_MAX_SIZE + 10);
  }
  break;

  default:
  break;
  }

  return s_ret;
}

/**
 * ======================
 * @function: reconnect
 */
void reconnect() {
  while (!client.connected()) {
    Serial.print("[RECONNECTED] Attempting MQTT connection...");
 
    /* Attempt to connect */
    if (client.connect("ESP32Client")) {
      Serial.println("CONNECTED SUCCESSFULLY");
      /* Subcribe TOPIC */
      client.subscribe(topic_subcribe);
    } 
    else 
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" Try again in 5 seconds");
      
      delay(5000); /* Wait 5 seconds before retrying */
    }
  }
}

/**
 * ======================
 * @function: callback
 */
void callback(char* topic, byte* message, unsigned int length) {
  Serial.println("[CALLBACK] MESSAGE COMING:");
  Serial.print("TOPIC: ");
  Serial.println(topic);
  Serial.print("MSG: ");
  
  String msg;
  
  for (int i = 0; i < length; i++) {
    msg += (char)message[i];
  }
  Serial.print(msg);
  Serial.println();

  handle_topic_msg(topic, msg);
}


/**
 * ============================
 * @function: handle_topic_msg
 */
void handle_topic_msg(char * topic, String msg)
{
  if (String(topic) == "CONTROL") {
    if(msg == "CONTROL LED ON"){
      if (mode_control == MODE_MANUAL)
      {
        Serial.print("[HANDLE TOPIC MSG] CONTROL LED ON");
        led_on();
        client.publish(topic_publish, "LED ON");
      }
      else
      {
        client.publish(topic_publish, "PLEASE SET MODE MANUAL");
      }
    }
    else if(msg == "CONTROL LED OFF"){
      if (mode_control == MODE_MANUAL)
      {
        Serial.print("[HANDLE TOPIC MSG] CONTROL LED OFF");
        led_off();
        client.publish(topic_publish, "LED OFF");
      }
      else
      {
        client.publish(topic_publish, "PLEASE SET MODE MANUAL");
      }
    }
    else if (msg == "CONTROL DEVICE OPEN")
    {
      if (mode_control == MODE_MANUAL)
      {
        device_control(DEVICE_OPEN);
        client.publish(topic_publish, "DEVICE OPEN");
      }
      else
      {
        client.publish(topic_publish, "PLEASE SET MODE MANUAL");
      }
    }
    else if (msg == "CONTROL DEVICE CLOSE")
    {
      if (mode_control == MODE_MANUAL)
      {
        device_control(DEVICE_CLOSE);
        client.publish(topic_publish, "DEVICE CLOSE");
      }
      else
      {
        client.publish(topic_publish, "PLEASE SET MODE MANUAL");
      }
    }
    else if (msg == "CONTROL MODE AUTO")
    {
      set_mode_control(MODE_AUTO);
      client.publish(topic_publish, "MODE AUTO");
    }
    else if (msg == "CONTROL MODE MANUAL")
    {
      set_mode_control(MODE_MANUAL);
      client.publish(topic_publish, "MODE MANUAL");
    }
    else if (msg == "GET DHT11 INFO")
    {
      client.publish(topic_publish, "[MSG RESPOND] DHT11");
      
      float t = dht11_read(TEMPERATURE);  
      float h = dht11_read(HUMIDITY);
      String mes = "TEMPERATURE: ";
      mes += String(t);
      mes += " - HUMIDITY: ";
      mes += String(h);
      client.publish(topic_publish, (char *)mes.c_str()); /* TEMPERATURE x - HUMIDITY y */
    }
    else if (msg == "GET DEVICE INFO")
    {
      /* Device */
      if (device_status == DEVICE_OPEN)
      {
        client.publish(topic_publish, "DEVICE OPEN");
      }
      else
      {
        client.publish(topic_publish, "DEVICE CLOSE");
      }
      
      /* LED */
      if (led_status == LED_ON)
      {
        client.publish(topic_publish, "LED ON");
      }
      else
      {
        client.publish(topic_publish, "LED OFF");
      }
    }
    else if (msg == "GET MODE INFO")
    {
      if (mode_control == MODE_AUTO)
      {
        client.publish(topic_publish, "MODE AUTO");
      }
      else 
      {
        client.publish(topic_publish, "MODE MANUAL");
      }
    }
    /* Auto control when value read over milestone */
    else if (msg == "OVER MILESTOME")
    {
      if (led_status == LED_OFF)
      {
        led_on();
        client.publish(topic_publish, "LED ON");
      }
      if (device_status == DEVICE_CLOSE)
      {
        device_control(DEVICE_OPEN);
        client.publish(topic_publish, "DEIVCE OPEN");
      }
    }
  }
}

/**
 * ======================
 * @function: handleROOT
 */
void handleROOT()
{
  server.send(200, "text/html", "<h1>YOU ARE CONNECTED</h1>");
}

/**
 * ======================
 * @function: handleLOGIN
 */
void handleLOGIN()
{
  File f = SPIFFS.open("/login.html","r");
  String debugLogData = "";
  
  while (f.available()){
    debugLogData += char(f.read());
  }
  
  int i = 0;
  byte val;
  String sta_ssid = "";
  String sta_pass = "";
  
  do{
    val = EEPROM.read(i);
    if (val != 0){
      sta_ssid += char(val);
    }
    i++;
  }while (val!=0 && i < SSID_MAX_SIZE);
  i=10;
  do{
    val = EEPROM.read(i);
    if (val != 0){
      sta_pass += char(val);
    }
    i++;
  }while (val != 0 && i < PASS_MAX_SIZE + 10);
  
  debugLogData.replace("abcdefgh",sta_ssid);
  debugLogData.replace("123456789",sta_pass);
  
  server.send(200, "text/html", debugLogData);  
}

/**
 * =======================
 * @function: handleSUBMIT
 */
void handleSUBMIT()
{
  String sta_ssid = server.arg("ssid");
  String sta_pass = server.arg("pass");

  /* Clear EEPROM after each login */
  if (sta_ssid.length() > 0 && sta_pass.length() > 0){
    for (int i = 0; i < EEPROM_MEM - 1; i++) {
      EEPROM.write(i, 0);
    }
  }
  
  int i;
  for (i = 0; i < sta_ssid.length(); i++){
    EEPROM.write(i, sta_ssid[i]);
  }
  EEPROM.write(i,0);

  for (i = 0; i < sta_pass.length(); i++){
    EEPROM.write(PASS_MAX_SIZE + i, sta_pass[i]);
  }
  EEPROM.write(PASS_MAX_SIZE + i,0);
  
  if(EEPROM.commit()){
    server.send(200, "text/html", "<html><head></head><body>WRITE TO EEPROM SUCCESSFULLY</body></html>");
    Serial.println("ESP32 REBOOT AFTER ");
    for (int t = 0; t < 10; t++){
      Serial.println(t);
      delay(500);
    }
    ESP.restart();
  }
  else{
    server.send(200, "text/html", "<html><head></head><body>FAIL TO WRITE TO EEPROM</body></html>");
  }
}
