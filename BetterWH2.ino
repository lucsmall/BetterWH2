/*
  Updated code for receiving data from WH2 weather station
  This code implements timeouts to make decoding more robust
  Decodes received packets and writes a summary of each packet to the Arduino's
  serial port
  Created by Luc Small on 19 July 2013.
  Released into the public domain.
*/

// Read data from 433MHz receiver on digital pin 2
#define RF_IN 2
// For better efficiency, the port is read directly
// the following two lines should be changed appropriately
// if the line above is changed.
#define RF_IN_RAW PIND2
#define RF_IN_PIN PIND

// Port that is hooked to LED to indicate a packet has been received
#define LED_PACKET A2

#define COUNTER_RATE 3200-1 // 16,000,000Hz / 3200 = 5000 interrupts per second, ie. 200us between interrupts
// 1 is indicated by 500uS pulse
// wh2_accept from 2 = 400us to 3 = 600us
#define IS_HI_PULSE(interval)   (interval >= 2 && interval <= 3)
// 0 is indicated by ~1500us pulse
// wh2_accept from 7 = 1400us to 8 = 1600us
#define IS_LOW_PULSE(interval)  (interval >= 7 && interval <= 8)
// worst case packet length
// 6 bytes x 8 bits x (1.5 + 1) = 120ms; 120ms = 200us x 600
#define HAS_TIMED_OUT(interval) (interval > 600)
// we expect 1ms of idle time between pulses
// so if our pulse hasn't arrived by 1.2ms, reset the wh2_packet_state machine
// 6 x 200us = 1.2ms
#define IDLE_HAS_TIMED_OUT(interval) (interval > 6)
// our expected pulse should arrive after 1ms
// we'll wh2_accept it if it arrives after
// 4 x 200us = 800us
#define IDLE_PERIOD_DONE(interval) (interval >= 4)
// Shorthand for tests
//#define RF_HI (digitalRead(RF_IN) == HIGH)
//#define RF_LOW (digitalRead(RF_IN) == LOW)
#define RF_HI (bit_is_set(RF_IN_PIN, RF_IN_RAW))
#define RF_LOW (bit_is_clear(RF_IN_PIN, RF_IN_RAW))

// wh2_flags 
#define GOT_PULSE 0x01
#define LOGIC_HI  0x02
volatile byte wh2_flags = 0;
volatile byte wh2_packet_state = 0;
volatile int wh2_timeout = 0;
byte wh2_packet[5];
byte wh2_calculated_crc;

ISR(TIMER1_COMPA_vect)
{
  static byte sampling_state = 0;
  static byte count;   
  static boolean was_low = false; 
    
  switch(sampling_state) {
    case 0: // waiting
      wh2_packet_state = 0;
      if (RF_HI) {
        if (was_low) {
          count = 0;
          sampling_state = 1;
          was_low = false;
        }
      } else {
        was_low = true;  
      }
      break;
    case 1: // acquiring first pulse
      count++;
      // end of first pulse
      if (RF_LOW) {
        if (IS_HI_PULSE(count)) {
          wh2_flags = GOT_PULSE | LOGIC_HI;
          sampling_state = 2;
          count = 0;        
        } else if (IS_LOW_PULSE(count)) {
          wh2_flags = GOT_PULSE; // logic low
          sampling_state = 2;
          count = 0;      
        } else {
          sampling_state = 0;
        }    
      }   
      break;
    case 2: // observe 1ms of idle time
      count++;
      if (RF_HI) {
         if (IDLE_HAS_TIMED_OUT(count)) {
           sampling_state = 0;
         } else if (IDLE_PERIOD_DONE(count)) {
           sampling_state = 1;
           count = 0;
         }
      }     
      break;     
  }
  
  if (wh2_timeout > 0) {
    wh2_timeout++; 
    if (HAS_TIMED_OUT(wh2_timeout)) {
      wh2_packet_state = 0;
      wh2_timeout = 0;
    }
  }
}

void setup() {
  Serial.begin(9600);
  Serial.println("BetterWH2");
  
  pinMode(LED_PACKET, OUTPUT);
  pinMode(RF_IN, INPUT);
  
  TCCR1A = 0x00;
  TCCR1B = 0x09;
  TCCR1C = 0x00;
  OCR1A = COUNTER_RATE; 
  TIMSK1 = 0x02;
  
  // enable interrupts
  sei();
}

void loop() {
  static unsigned long old = 0, packet_count = 0, bad_count = 0, average_interval; 
  unsigned long spacing, now;
  byte i;
  
  if (wh2_flags) {
    if (wh2_accept()) {
      // calculate the CRC
      wh2_calculate_crc();
      
      now = millis();
      spacing = now - old;
      old = now;
      packet_count++;
      average_interval = now / packet_count;     
      if (!wh2_valid()) {
        bad_count++;  
      }

      // flash green led to say got packet
      digitalWrite(LED_PACKET, HIGH);
      delay(100);
      digitalWrite(LED_PACKET, LOW);
      
      Serial.print(packet_count, DEC);   
      Serial.print(" | ");
      Serial.print(bad_count, DEC);   
      Serial.print(" | ");
      Serial.print(spacing, DEC);
      Serial.print(" | ");
      Serial.print(average_interval, DEC);
      Serial.print(" | ");
       
      for(i=0;i<5;i++) {
        Serial.print("0x");
        Serial.print(wh2_packet[i], HEX);
        Serial.print("/");
        Serial.print(wh2_packet[i], DEC);
        Serial.print(" ");
      }  
      Serial.print("| Sensor ID: 0x");
      Serial.print(wh2_sensor_id(), HEX);
      Serial.print(" | ");
      Serial.print(wh2_humidity(), DEC);
      Serial.print("% | ");
      Serial.print(wh2_temperature(), DEC);
      Serial.print(" | ");
      Serial.println((wh2_valid() ? "OK" : "BAD"));      
   }
   wh2_flags = 0x00; 
  }
}


// processes new pulse
boolean wh2_accept()
{
  static byte packet_no, bit_no, history;

  // reset if in initial wh2_packet_state
  if(wh2_packet_state == 0) {
     // should history be 0, does it matter?
    history = 0xFF;
    wh2_packet_state = 1;
    // enable wh2_timeout
    wh2_timeout = 1;
  } // fall thru to wh2_packet_state one
 
  // acquire preamble
  if (wh2_packet_state == 1) {
     // shift history right and store new value
     history <<= 1;
     // store a 1 if required (right shift along will store a 0)
     if (wh2_flags & LOGIC_HI) {
       history |= 0x01;
     }
     // check if we have a valid start of frame
     // xxxxx110
     if ((history & B00000111) == B00000110) {
       // need to clear packet, and counters
       packet_no = 0;
       // start at 1 becuase only need to acquire 7 bits for first packet byte.
       bit_no = 1;
       wh2_packet[0] = wh2_packet[1] = wh2_packet[2] = wh2_packet[3] = wh2_packet[4] = 0;
       // we've acquired the preamble
       wh2_packet_state = 2;
    }
    return false;
  }
  // acquire packet
  if (wh2_packet_state == 2) {

    wh2_packet[packet_no] <<= 1;
    if (wh2_flags & LOGIC_HI) {
      wh2_packet[packet_no] |= 0x01;
    }

    bit_no ++;
    if(bit_no > 7) {
      bit_no = 0;
      packet_no ++;
    }

    if (packet_no > 4) {
      // start the sampling process from scratch
      wh2_packet_state = 0;
      // clear wh2_timeout
      wh2_timeout = 0;
      return true;
    }
  }
  return false;
}


void wh2_calculate_crc()
{
  wh2_calculated_crc = crc8(wh2_packet, 4);
}

bool wh2_valid()
{
  return (wh2_calculated_crc == wh2_packet[4]);
}

int wh2_sensor_id()
{
  return (wh2_packet[0] << 4) + (wh2_packet[1] >> 4);
}

byte wh2_humidity()
{
  return wh2_packet[3];
}

/* Temperature in deci-degrees. e.g. 251 = 25.1 */
int wh2_temperature()
{
  int temperature;
  temperature = ((wh2_packet[1] & B00000111) << 8) + wh2_packet[2];
  // make negative
  if (wh2_packet[1] & B00001000) {
    temperature = -temperature;
  }
  return temperature;
}

uint8_t crc8( uint8_t *addr, uint8_t len)
{
  uint8_t crc = 0;

  // Indicated changes are from reference CRC-8 function in OneWire library
  while (len--) {
    uint8_t inbyte = *addr++;
    for (uint8_t i = 8; i; i--) {
      uint8_t mix = (crc ^ inbyte) & 0x80; // changed from & 0x01
      crc <<= 1; // changed from right shift
      if (mix) crc ^= 0x31;// changed from 0x8C;
      inbyte <<= 1; // changed from right shift
    }
  }
  return crc;
}








