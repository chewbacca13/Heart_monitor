#include <SPI.h>
#include "MAX30100.h"
#include <Wire.h>



//===== SPI =====
const bool is_master = true;
volatile byte led_value = LOW;
volatile bool master_send_ok = true;
volatile bool data_available = false;

const unsigned LEN = 1;
char buf1[LEN], buf2[LEN];

volatile unsigned index = 0;
volatile unsigned to_be_displayed = 1u;
volatile bool ready_to_process = false;


void setup_master()
{
  cli();
  SPI.begin();

  pinMode(MOSI, OUTPUT);
  pinMode(MISO, INPUT);
  pinMode(SCK, OUTPUT);
  pinMode(SS, OUTPUT);

  SPCR |= 1 << SPE;   // SPI enable
  SPCR |= 1 << SPIE;  // enable interrupt
  SPCR |= 1 << MSTR;  // this is master => MSTR = 1

  // setez frecventa SPI-ului = 16MHz / 4 = 4MHz
  SPCR &= ~(1 << SPR1);   // SPR1 = 0
  SPCR &= ~(1 << SPR0);   // SPR0 = 0
  SPSR &= ~(1 << SPI2X);  // SPI2X = 0

  SPCR &= ~(1 << DORD);  // DORD = 0 => MSB first

  SPCR &= ~(1 << CPOL);
  SPCR &= ~(1 << CPHA);

  sei();
}

void setup_slave()
{
  cli();
  SPI.begin();

  pinMode(MOSI, INPUT);
  pinMode(MISO, OUTPUT);
  pinMode(SCK, INPUT);
  pinMode(SS, INPUT);

  SPCR |= 1 << SPE;  // enable SPI
  SPCR |= 1 << SPIE; // enable interrupt

  SPCR &= ~(1 << MSTR);  // this is slave => MSTR = 0
  SPCR &= ~(1 << DORD);  // DORD = 0 => MSB first

  SPCR &= ~(1 << CPOL);
  SPCR &= ~(1 << CPHA);

  sei();
}


void spi_send(unsigned char value)
{
  digitalWrite(SS, LOW);  // slave select
  master_send_ok = false;
  SPDR = value;
  digitalWrite(SS, HIGH);  
}


ISR(SPI_STC_vect)
{
  if (!is_master) {
    char *buf = (to_be_displayed == 1) ? buf2 : buf1;
    buf[index++] = SPDR;  // data register
    led_value = (led_value == HIGH) ? LOW : HIGH;  // toggle led
  
    if (index == LEN) {
      to_be_displayed = (to_be_displayed == 1) ? 2 : 1;
      index = 0;
      ready_to_process = true;
    }
  } else {
    master_send_ok = true;
  }
}
// ===== Task manager =====


const unsigned BAUD_RATE = 9600;
const unsigned FREQ = 10; // HERTZ

// =====================================
const unsigned NTASKS = 10;

typedef enum TASK_STATE { WAIT, READY, RUN } task_state; 
typedef struct task
{
  unsigned id{0};
  unsigned vtime{};
  task_state state{WAIT};
  void (*f)(void *){};
  void *param{};
  unsigned prio{};
  unsigned T{};  // period
  unsigned D{};  // delay
  unsigned timeout{};
  bool periodic{true};
} task;

task tasks[NTASKS];

// =====================================


unsigned add_task(void (*f)(void*), void *param, unsigned T, unsigned D, unsigned timeout, bool periodic, unsigned prio)
{
  if (!f || T == 0) return NTASKS;
  for (unsigned i = 0; i < NTASKS; i++) {
    if (tasks[i].f == NULL) {
      tasks[i].id = i;
      tasks[i].f = f;
      tasks[i].param = param;
      tasks[i].T = T;
      tasks[i].D = D;
      tasks[i].timeout = timeout;
      tasks[i].periodic = periodic;
      tasks[i].prio = prio;
      tasks[i].vtime = 0;
      tasks[i].state = WAIT;

      return i;
    }
  }
  return NTASKS;  // lista e plina
}

unsigned delete_task(unsigned id)
{
  if (id < NTASKS) {
    tasks[id].f = NULL;
    tasks[id].state = WAIT;
    tasks[id].T = 0;  
  }
}


void init_scheduler()
{
  setup_timer1();
}


void setup_timer1()
{
  cli();
  TCCR1A=0;
  TCCR1B=0;
  TCNT1=0; //SET INIT VALUE

  //set prescaler to 64
  TCCR1B |= 1 << CS02 | 1 << CS00;

  OCR1A = 16 * pow(10,6) / (1024 * FREQ) - 1;
  
  TIMSK1 |= 1<< OCIE1A;  // ENABLE INTERRUPT FOR OCR1A
  TCCR1B |= 1 << WGM12;  // CTC (clear timer on compare) mode

  sei();  // begin to count
}


ISR(TIMER1_COMPA_vect)  // "scheduler tick"
{
  for (unsigned i = 0; i < NTASKS; i++) {
    if (tasks[i].f != NULL) {
      if (tasks[i].state == RUN) {
        if (++tasks[i].vtime > tasks[i].timeout) {
          delete_task(i);  // TODO
        }
      } else {
        if (tasks[i].D == 0) {
          tasks[i].state = READY;
          tasks[i].vtime = 0;
          if (tasks[i].periodic)
            tasks[i].D = tasks[i].T;
        } else {
          tasks[i].D--;
        }
      }
    }
  }
}

//===== Functions for the tasks =====
MAX30100 sensor;
double heart_rate_avg;
int n_heart_rate;
int abnormal_time;
#define BUZZ_PIN 13
const char ALARM[] = "ALARM";

double get_pulse()
{
  sensor.readSensor();
  //return (double) sensor.IR;
  return 39;    //for testing
}

void buzzer_on(void *unused)  { digitalWrite(BUZZ_PIN, HIGH); }
void buzzer_off(void *unused) { digitalWrite(BUZZ_PIN, LOW); }


void alarm(void *unused)
{
    Serial.println(ALARM);
    abnormal_time = 0;
    add_task(buzzer_on, NULL, 0, 0, FREQ, false, 1);
    add_task(buzzer_off, NULL, 0, FREQ, FREQ, false, 1);
    
    add_task(buzzer_on, NULL, 2*FREQ, 0, FREQ, false, 1);
    add_task(buzzer_off, NULL, 2*FREQ, FREQ, FREQ, false, 1);
    
    add_task(buzzer_on, NULL, 4*FREQ, 0, FREQ, false, 1);
    add_task(buzzer_off, NULL, 4*FREQ, FREQ, FREQ, false, 1);

    for (int i=0; i<strlen(ALARM); i++)
      spi_send(ALARM[i]);    
}

void read_pulse(void *unused)
{
  double pulse = get_pulse();
  heart_rate_avg += get_pulse();
  n_heart_rate++;

  Serial.print("Pulse: ");Serial.println(pulse);
  
  if (pulse < 50.0 || pulse > 140.0)  {
    abnormal_time++;
  }
  else
    abnormal_time = 0;

  if (abnormal_time >= 30){
    add_task(alarm, NULL, 1, 1, FREQ, false, 1);
  }
}


void send_avg_pulse(void *unused)
{
  char buf[15] = "AVG=";
  char buf2[10];
  double avg = heart_rate_avg / n_heart_rate;
  dtostrf(avg, 9, 2, &buf[4]);
  heart_rate_avg = 0;
  n_heart_rate = 0;
  
  Serial.println(buf);
  for (int i=0; i<strlen(buf); i++)
    spi_send(buf[i]);
}


void setup_tasks()
{
    Wire.begin();
    sensor.begin(pw1600, i50, sr100);
    index = 0;
    abnormal_time = 0;
    n_heart_rate = 0;
    heart_rate_avg = 0;
    pinMode(BUZZ_PIN, OUTPUT);
    digitalWrite(BUZZ_PIN, LOW);

    

    add_task(read_pulse, NULL, 2*FREQ, 0, 1*FREQ, true, 2);
    add_task(send_avg_pulse, NULL, 60*FREQ, 60*FREQ, 1*FREQ, true, 2);
    
    
  //Serial.println("DONE");
}


//===== MAIN =====
void setup()
{
  cli();
  Serial.begin(500000);

  if (is_master) {
    setup_master();
    setup_tasks();
    init_scheduler();
  } else {
    setup_slave();
  }
  sei();
}


void loop()
{
  if (is_master) {
    for (unsigned i = 0; i < NTASKS; i++) {
      if (tasks[i].f != NULL && tasks[i].state == READY) {
        tasks[i].state = RUN;
        tasks[i].f(tasks[i].param);  // execute task (might take longer than expected)
        tasks[i].state = WAIT;
        if (!tasks[i].periodic)
          delete_task(i);
      }
    }
  } 
  else {
    if (ready_to_process) {
      char *buf = (to_be_displayed == 1) ? buf1 : buf2;
      Serial.println((int)buf[0]);
      ready_to_process = false;     
    }
  }
}
