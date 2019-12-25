#include <Wire.h>
#include <U8g2lib.h>
#include <RTClib.h>
#include <dht11.h>
//#include <HardwareSerial.h>


//#define PUMP_TANK1_TIMER 60000
//#define PUMP_TANK2_TIMER 60000
//#define PUMP_TANK3_TIMER 60000
#define PUMP_TANK1_TIMER 600000
#define PUMP_TANK2_TIMER 600000
#define PUMP_TANK3_TIMER 3900000
#define PERPUMP_TIMER    180000 // 3 minutes

#define PUMP_TANK1  27
#define PUMP_TANK2  14
#define PUMP_TANK3  12
#define PERPUMP     23
#define PUMP_XHAUST 13

#define PUMP_TANK1_SENSOR 19
#define PUMP_TANK2_SENSOR 2
#define PUMP_TANK3_SENSOR 36
#define PERPUMP_SENSOR    4 // GPIO that monitors the state (ON/OFF) of the pump

#define CLOSE  500
#define OPEN   1500
#define FEEDER 17

#define TEMPSETPOINT     25
#define DHT11_PIN        15
#define SEND_DATA_PERIOD 30000

#define PHSETPOINT 7.0
#define PHSENSOR   35
#define OFFSET     0.00

const int DUMMY = 22;

      int PUMP_TANK1_SCHED[]  = {1, 4, 7, 10, 13, 16, 19, 22};
      int PUMP_TANK2_SCHED[]  = {1, 4, 7, 10, 13, 16, 19, 22};
      int PUMP_TANK3_SCHED[]  = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23};

//      int PUMP_TANK1_SCHED[]  = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23};
//      int PUMP_TANK2_SCHED[]  = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23};
//      int PUMP_TANK3_SCHED[]  = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23};

//      int PUMP_TANK1_SCHED[]  = {0, 30};
//      int PUMP_TANK2_SCHED[]  = {0, 20, 40};
//      int PUMP_TANK3_SCHED[]  = {0, 10, 20, 30, 40, 50};
      
      int PERPUMP_SCHED[]     = {1, 5, 9, 13, 17, 21}; // automatically turn on the pump every 4 hours
      int FEED_SCHED[]        = {6, 18}; // feed the fish every 6AM and 6PM per day

const int PUMP_TANK1_SCHED_SIZE  = sizeof(PUMP_TANK1_SCHED) / sizeof(PUMP_TANK1_SCHED[0]); 
const int PUMP_TANK2_SCHED_SIZE  = sizeof(PUMP_TANK2_SCHED) / sizeof(PUMP_TANK2_SCHED[0]); 
const int PUMP_TANK3_SCHED_SIZE  = sizeof(PUMP_TANK3_SCHED) / sizeof(PUMP_TANK3_SCHED[0]); 
const int PERPUMP_SCHED_SIZE     = sizeof(PERPUMP_SCHED) / sizeof(PERPUMP_SCHED[0]); // get the size of the PERPUMP_SCHED array
const int FEED_SCHED_SIZE        = sizeof(FEED_SCHED) / sizeof(FEED_SCHED[0]); // get the size of the PERPUMP_SCHED array

int minutes = 0;

int pump_tank1_curr = 0;
int pump_tank2_curr = 0;
int pump_tank3_curr = 0;
int perpump_curr    = 0; // schedule locator of the PERPUMP_SCHED array
int feed_curr       = 0; // schedule locator of the FEED_SCHED array

int pump_tank1_timer = 0;
int pump_tank2_timer = 0;
int pump_tank3_timer = 0;
int perpump_timer    = 0;
int send_data_timer  = 0;
int global_time      = 0;

char print_time[50] = "";
char print_temperature[50] = "";
char print_relative_humidity[50] = "";
char print_ph_level[50] = "";

char print_pump_tank1_on_time[50] = "";
char print_pump_tank2_on_time[50] = "";
char print_pump_tank3_on_time[50] = "";
char print_feed_time[50]          = "";

unsigned long int avgValue = 0;

boolean tank1_today = true;
boolean tank2_today = true;
boolean tank3_today = true;

boolean fed = false;

U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
RTC_DS3231 rtc;
dht11 DHT;

int get_time_index(char o);
int get_hour();
void feed(int state);
void turn_servo();
float measure_temp_rh(char o);
void temperature();
void humidity();
void send_data();
float measure_ph();

void setup() {
  Serial.begin(9600);

  pinMode(PUMP_TANK1_SENSOR, INPUT);
  pinMode(PUMP_TANK2_SENSOR, INPUT);
  pinMode(PUMP_TANK3_SENSOR, INPUT);
  pinMode(PERPUMP_SENSOR, INPUT);

  pinMode(PUMP_TANK1, OUTPUT);
  pinMode(PUMP_TANK2, OUTPUT);
  pinMode(PUMP_TANK3, OUTPUT);
  pinMode(PERPUMP, OUTPUT);
  pinMode(PUMP_XHAUST, OUTPUT);
  pinMode(FEEDER, OUTPUT);

  Serial.println("after pinmodes");

  u8g2.begin();
  if(!rtc.begin()) {
    while(1);
  }
//   rtc.adjust(DateTime(__DATE__, __TIME__)); // copy the time from pc to the RTC
  pump_tank1_curr = get_time_index('X');
  pump_tank2_curr = get_time_index('Y');
  pump_tank3_curr = get_time_index('Z');
  perpump_curr    = get_time_index('T');
  feed_curr       = get_time_index('F');

  digitalWrite(PUMP_TANK1, LOW);
  digitalWrite(PUMP_TANK2, LOW);
  digitalWrite(PUMP_TANK3, LOW);
  digitalWrite(PERPUMP, LOW);

  feed(CLOSE);
  delay(5000);
  Serial.println("setup done");
}

void loop() {
  /*
    If the current time, in hours, is greater than the schedule: PERPUMP_SCHED[perpump_curr], the locator
    perpump_curr is incremented by 1. ESP32 then signals the arduino to turn on by sending a 'TMP1'.
    Then, the timer perpump_timer is started.
    For example, if the current time is 10:00 AM while the perpump_curr is 2 (9:00 schedule) the pump must
    turn ON. The increment of the perpump_curr and the start of the timer are only triggered during the first 
    start up of the motor in every schedule.

    If the current time is below PERPUMP_SCHED[perpump_curr], status of the motor is checked. If the motor is ON, 
    the time of operation is checked if it has been operating beyond the set duration of PERPUMP_TIMER. If
    ybeyond the PUMP_timer, the ESP32 signals the Arduino to turn off by sending a 'TMP0' message
    For example, if the schedule is 13:00 (that is, perpump_curr = 3) while the current time is 10:00, and the 
    motor has been running since 9:00 AM, the ESP32 must signal the arduino to turn off the motor.
  */
 int current_hour = get_hour();

 if (current_hour < global_time) {
  if (!tank1_today || !tank2_today || !tank3_today ) {
    tank1_today = true;
    tank2_today = true;
    tank3_today = true;
  }
  if (fed) {
    fed = false;
  }
 }

 global_time = current_hour;
 
  Serial.println(pump_tank1_curr);
  Serial.println(pump_tank2_curr);
  Serial.println(pump_tank3_curr);
  Serial.println(current_hour);
  
 if ((current_hour >= PUMP_TANK1_SCHED[pump_tank1_curr]) && tank1_today) {
    digitalWrite(PUMP_TANK1, HIGH);
    
    int h = get_hour();
    print_pump_tank1_on_time[0] = 0;
    sprintf(print_pump_tank1_on_time, "P1: %d:%d", h, minutes); 
    
    Serial.println("pump1 on");
    pump_tank1_timer = millis();
    if(pump_tank1_curr == PUMP_TANK1_SCHED_SIZE - 1) {
      pump_tank1_curr = 0;
      tank1_today = false;
    }else {
      Serial.println("increment curr1");
      pump_tank1_curr += 1;
      tank1_today = true;
    }
  } else {
   if (digitalRead(PUMP_TANK1_SENSOR)) {
    Serial.println("pump1 high pin");
      if ((millis() - pump_tank1_timer) > PUMP_TANK1_TIMER) {
        digitalWrite(PUMP_TANK1, LOW);
        Serial.println("pump1 off");
      }
   }
  }
  Serial.println("check here");
  Serial.println(current_hour);

   if ((current_hour >= PUMP_TANK2_SCHED[pump_tank2_curr]) && tank2_today) {
    digitalWrite(PUMP_TANK2, HIGH);
    
    int h = get_hour();
    print_pump_tank2_on_time[0] = 0;
    sprintf(print_pump_tank2_on_time, "P2: %d:%d", h, minutes); 
    
    Serial.println("pump2 on");
    pump_tank2_timer = millis();
    if(pump_tank2_curr == PUMP_TANK2_SCHED_SIZE - 1) {
      pump_tank2_curr = 0;
      tank2_today = false;
    }else {
      Serial.println("increment curr2");
      pump_tank2_curr += 1;
      tank2_today = true;
    }
  } else {
   if (digitalRead(PUMP_TANK2_SENSOR)) {
    Serial.println("pump2 high pin");
      if ((millis() - pump_tank2_timer) > PUMP_TANK2_TIMER) {
        digitalWrite(PUMP_TANK2, LOW);
        Serial.println("pump2 off");
      }
   }
  }

  if ((current_hour >= PUMP_TANK3_SCHED[pump_tank3_curr]) && tank3_today) {
    digitalWrite(PUMP_TANK3, HIGH);

    int h = get_hour();
    print_pump_tank3_on_time[0] = 0;
    sprintf(print_pump_tank3_on_time, "P3: %d:%d", h, minutes); 
    
    Serial.println("pump3 on");
    pump_tank3_timer = millis();
    if(pump_tank3_curr == PUMP_TANK3_SCHED_SIZE - 1) {
      pump_tank3_curr = 0;
      tank3_today = false;
    }else {
      Serial.println("increment curr3");
      pump_tank3_curr += 1;
      tank3_today = true;
    }
  } else {
   if (digitalRead(PUMP_TANK3_SENSOR)) {
    Serial.println("pump3 high pin");
      if ((millis() - pump_tank3_timer) > PUMP_TANK3_TIMER) {
        digitalWrite(PUMP_TANK3, LOW);
        Serial.println("pump3 off");
      }
   }
  }

  // if (current_hour >= PERPUMP_SCHED[perpump_curr] && today) {
  //   digitalWrite(PERPUMP, HIGH);
  //   perpump_timer = millis();
  //   if(perpump_curr == PERPUMP_SCHED_SIZE - 1) {
  //     perpump_curr = 0;
  //     today = false;
  //   }else {
  //     perpump_curr += 1;
  //     today = true;
  //   }
  // } else {
  //  if (digitalRead(PERPUMP_SENSOR)) {
  //     if ((millis() - perpump_timer) > PERPUMP_TIMER) {
  //       digitalWrite(PERPUMP, LOW);
  //     }
  //  }
  // }

// uncomment after debugging pumps

  if (current_hour >= FEED_SCHED[feed_curr] && !fed) {
   turn_servo();
   /*
    * If the feed_curr is at the last element of the schedule, it's next value will reset to zero.
    * In effect, the current time will be greater than the FEED_SCHED[feed_curr]. For this case, 
    * to avoid turning the feeder, the progrma shall send a signal, fed as TRUE, to indicate that 
    * feeding was already done.
    */
   if(feed_curr == FEED_SCHED_SIZE - 1) {
    feed_curr = 0;
    fed = true;
  }else {
    feed_curr += 1;
    fed = false;
  }
 }

 if (millis() - send_data_timer > SEND_DATA_PERIOD) {
  measure_temp_rh('t'); 
  measure_ph();
 }
  delay(1000);

  if (abs(measure_ph() - PHSETPOINT) >= 0.3) {
    digitalWrite(PERPUMP, HIGH);
  } else {
    digitalWrite(PERPUMP, LOW);
  }

  if (abs(measure_temp_rh('t') - TEMPSETPOINT) >= 3.0) {
    digitalWrite(PUMP_XHAUST, HIGH);
  } else {
    digitalWrite(PUMP_XHAUST, LOW);
  }
  print_oled();
}

/*
  Gets the schedule to turn ON the pump at the start up of the device. If the current h is in between
  i and i+1, the locator 's' is set as i. For example, if the current h is 11,
  the s is set as 2 (corresponds to 9:00 AM schedule)
*/
int get_time_index(char o) {
  int i = 0;
  int n = 0;
  int s = 0;
  int size = 0;
  int *m = {};

  int h = get_hour();

  // select which scheule to look up based on the value of 'o'
  if (o == 'T') {
    size = PERPUMP_SCHED_SIZE;
    m = PERPUMP_SCHED;
  } else if (o == 'X') {
    size = PUMP_TANK1_SCHED_SIZE;
    m = PUMP_TANK1_SCHED;
  } else if (o == 'Y') {
    size = PUMP_TANK2_SCHED_SIZE;
    m = PUMP_TANK2_SCHED;
  } else if (o == 'Z') {
    size = PUMP_TANK3_SCHED_SIZE;
    m = PUMP_TANK3_SCHED;
  } else {
    size = FEED_SCHED_SIZE;
    m = FEED_SCHED;
  }

  // size = FEED_SCHED_SIZE;
  //   m = FEED_SCHED;
  // }
  
  for (i = 0; i < size - 1; i++) {
    if (i == size - 1) {
      n = 0;
    }else {
      n = i + 1;
    }
    if (h >= m[i] && h < m[n]) {
      s = i;
      return s;
    }
  }
}

/*
  Gets the current time in hours
*/
int get_hour() {
  DateTime now = rtc.now();
  int H  = 0;
  H = now.hour();
  minutes = now.minute();

  print_time[0] = 0;
  sprintf(print_time, "%d:%d", H, minutes); 
  Serial.println(print_time);  
  return H;
}

void print_oled() {
  get_hour();
  measure_temp_rh('t');
  measure_temp_rh('r');
  measure_ph();
  u8g2.clearBuffer();          // clear the internal memory
  u8g2.setFont(u8g2_font_logisoso16_tn); // choose a suitable font
  u8g2.drawStr(8,16,print_time);  // write something to the internal memory
  u8g2.setFont(u8g2_font_ncenB10_tr); // choose a suitable font
  u8g2.drawStr(60,12,print_temperature);  // write something to the internal memory
  u8g2.drawStr(60,26,print_relative_humidity);  // write something to the internal memory
  u8g2.drawStr(60,40,print_ph_level);  // write something to the internal memory
  u8g2.setFont(u8g2_font_missingplanet_t_all ); // choose a suitable font
  u8g2.drawStr(8,32,print_pump_tank1_on_time);  // write something to the internal memory
  u8g2.drawStr(8,43,print_pump_tank2_on_time);  // write something to the internal memory
  u8g2.drawStr(8,54,print_pump_tank3_on_time);  // write something to the internal memory
  u8g2.drawStr(60,54,print_feed_time);  // write something to the internal memory
  u8g2.sendBuffer();          // transfer internal memory to the display
}

void turn_servo() {
  feed(OPEN);
  Serial.println("TURNING FINISHED");
  
  delay(2000);
  feed(CLOSE);
  delay(2000);

  int h = get_hour();
  print_feed_time[0] = 0;
  sprintf(print_feed_time, "AF: %d:%d", h, minutes); 
}

void feed(int state) {
  uint32_t timer = millis();
  while (millis() - timer < 2000) {
    digitalWrite(FEEDER, HIGH);
    delayMicroseconds(state);
    digitalWrite(FEEDER, LOW);
    delay(20);
  }
  return;
}

float measure_temp_rh(char o) {
  int chk = DHT.read(DHT11_PIN);
  float t, h;
  switch (chk){
    case DHTLIB_OK:  
                Serial.print("OK,\t"); 
                break;
    case DHTLIB_ERROR_CHECKSUM: 
                Serial.print("Checksum error,\t"); 
                break;
    case DHTLIB_ERROR_TIMEOUT: 
                Serial.print("Time out error,\t"); 
                break;
    default: 
                Serial.print("Unknown error,\t"); 
                break;
  }
  t = DHT.temperature;
  h = DHT.humidity;
  delay(2000);
  if (o == 't') {
//    Serial.println(t,1);
//    delay(2000);
    print_temperature[0] = 0;
    sprintf(print_temperature, "TE: %.1f", t); 
    return t;
  } else {
//    Serial.println(h,1);
//    delay(2000);
    print_relative_humidity[0] = 0;
    sprintf(print_relative_humidity, "RH: %.1f", h); 
    return h;
  }
}

float measure_ph() {
  int buf[10];
  for (int i=0; i<10; i++) {
    buf[i] = analogRead(PHSENSOR);
    delay(10);
  }
  for(int i=0;i<9;i++) {
    for(int j=i+1;j<10;j++) {
      if(buf[i]>buf[j]) {
        int temp=buf[i];
        buf[i]=buf[j];
        buf[j]=temp;
      }
    }
  }
  avgValue = 0;
  for (int i=2; i<8; i++) {
    avgValue+=buf[i];
  }
  float phValue = (float)avgValue*3.3/4095.0/6;
  Serial.println(phValue);
  phValue = 3.5*phValue + OFFSET;
  Serial.print("    pH:");  
  Serial.print(phValue,2);
  Serial.println(" ");
  delay(800);
  print_ph_level[0] = 0;
  sprintf(print_ph_level, "PH: %.1f", phValue); 
  return phValue;
}


