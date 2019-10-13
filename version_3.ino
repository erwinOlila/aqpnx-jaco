#include <Wire.h>
#include <RTClib.h>
#include <dht11.h>
//#include <HardwareSerial.h>

#define PUMP_TANK1_TIMER 13200000
#define PUMP_TANK2_TIMER 13200000
#define PUMP_TANK3_TIMER 216000000
#define PERPUMP_TIMER    180000 // 3 minutes

#define PUMP_TANK1  27
#define PUMP_TANK2  14
#define PUMP_TANK3  12
#define PERPUMP     23
#define PUMP_XHAUST 13

#define PUMP_TANK1_SENSOR 1
#define PUMP_TANK2_SENSOR 2
#define PUMP_TANK3_SENSOR 3
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

      int PUMP_TANK1_SCHED[]  = {0, 6, 12, 18};
      int PUMP_TANK2_SCHED[]  = {0, 6, 12, 18};
      int PUMP_TANK3_SCHED[]  = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23};
      int PERPUMP_SCHED[]     = {1, 5, 9, 13, 17, 21}; // automatically turn on the pump every 4 hours
      int FEED_SCHED[]        = {6, 23}; // feed the fish every 6AM and 6PM per day

const int PUMP_TANK1_SCHED_SIZE  = sizeof(PUMP_TANK1_SCHED) / sizeof(PUMP_TANK1_SCHED[0]); 
const int PUMP_TANK2_SCHED_SIZE  = sizeof(PUMP_TANK2_SCHED) / sizeof(PUMP_TANK2_SCHED[0]); 
const int PUMP_TANK3_SCHED_SIZE  = sizeof(PUMP_TANK3_SCHED) / sizeof(PUMP_TANK3_SCHED[0]); 
const int PERPUMP_SCHED_SIZE  = sizeof(PERPUMP_SCHED) / sizeof(PERPUMP_SCHED[0]); // get the size of the PERPUMP_SCHED array
const int FEED_SCHED_SIZE     = sizeof(FEED_SCHED) / sizeof(FEED_SCHED[0]); // get the size of the PERPUMP_SCHED array

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

unsigned long int avgValue = 0;

boolean today = true;
boolean fed = false;

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

 if (current_hour >= PUMP_TANK1_SCHED[pump_tank1_curr] && today) {
    digitalWrite(PUMP_TANK1, HIGH);
    pump_tank1_timer = millis();
    if(pump_tank1_curr == PUMP_TANK1_SCHED_SIZE - 1) {
      pump_tank1_curr = 0;
      today = false;
    }else {
      pump_tank1_curr += 1;
      today = true;
    }
  } else {
   if (digitalRead(PUMP_TANK1_SENSOR)) {
      if ((millis() - pump_tank1_timer) > PUMP_TANK1_TIMER) {
        digitalWrite(PUMP_TANK1, LOW);
      }
   }
  }

   if (current_hour >= PUMP_TANK2_SCHED[pump_tank2_curr] && today) {
    digitalWrite(PUMP_TANK2, HIGH);
    pump_tank2_timer = millis();
    if(pump_tank2_curr == PUMP_TANK2_SCHED_SIZE - 1) {
      pump_tank2_curr = 0;
      today = false;
    }else {
      pump_tank2_curr += 1;
      today = true;
    }
  } else {
   if (digitalRead(PUMP_TANK2_SENSOR)) {
      if ((millis() - pump_tank2_timer) > PUMP_TANK2_TIMER) {
        digitalWrite(PUMP_TANK2, LOW);
      }
   }
  }

  if (current_hour >= PUMP_TANK3_SCHED[pump_tank3_curr] && today) {
    digitalWrite(PUMP_TANK3, HIGH);
    pump_tank3_timer = millis();
    if(pump_tank3_curr == PUMP_TANK3_SCHED_SIZE - 1) {
      pump_tank3_curr = 0;
      today = false;
    }else {
      pump_tank3_curr += 1;
      today = true;
    }
  } else {
   if (digitalRead(PUMP_TANK3_SENSOR)) {
      if ((millis() - pump_tank3_timer) > PUMP_TANK3_TIMER) {
        digitalWrite(PUMP_TANK3, LOW);
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
  } else {
    size = FEED_SCHED_SIZE;
    m = FEED_SCHED;
  }
  
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

  char the_time[50] = "";
  sprintf(the_time, "Time: %d:%d:%d", now.hour(), now.minute(), now.second()); 
  Serial.println(the_time);  
  return H;
}

void turn_servo() {
  feed(OPEN);
  Serial.println("TURNING FINISHED");
  delay(2000);
  feed(CLOSE);
  delay(2000);
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

  if (o == 't') {
    Serial.println(t,1);
    delay(2000);
    return t;
  } else {
    Serial.println(h,1);
    delay(2000);
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
  return phValue;
}


