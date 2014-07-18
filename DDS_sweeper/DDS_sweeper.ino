/***************************************************************************\
*  Name    : DDS_Sweeper.BAS                                                *
*  Author  : Beric Dunn (K6BEZ)                                             *
*  Notice  : Copyright (c) 2013  CC-BY-SA                                   *
*          : Creative Commons Attribution-ShareAlike 3.0 Unported License   *
*  Date    : 18 Jul 2014                                                    *
*  Version : 1.1/M0CUV                                                      *
*  Notes   : Written using for the Arduino Micro                            *
*          :   Pins:                                                        *
*          :    A0 - Reverse Detector Analog in                             *
*          :    A1 - Forward Detector Analog in                             *
\***************************************************************************/

// Define Pins used to control AD9850 DDS
const int FQ_UD=10;
const int SDAT=11;
const int SCLK=9;
const int RESET=12;

// Pins for Fwd/Rev detectors
const int REV_ANALOG_PIN = A0;
const int FWD_ANALOG_PIN = A1;

// "ON/Scan in progress" LED on M0CUV board
const int LED=8;
// Inbuilt LED on Arduino Micro, fades waiting for input
const int INTLED=13;

int brightness = 0;    // how bright the LED is
int fadeAmount = 1;    // how many points to fade the LED by
int countdown = 1024;   // only change brightness when this wraps round zero, so no delay needed

double Fstart_MHz = 1;  // Start Frequency for sweep
double Fstop_MHz = 10;  // Stop Frequency for sweep
double current_freq_MHz; // Temp variable used during sweep
long serial_input_number; // Used to build number from serial stream
int num_steps = 100; // Number of steps to use in the sweep
char incoming_char; // Character read from serial stream
int settle_delay = 10; // How long to wait in ms after setting the frequency before reading the voltages.
int oscilloscope_pin = FWD_ANALOG_PIN; // Read the forward detector, unless overridden.

// the setup routine runs once when you press reset:
void setup() {
  // Configiure DDS control pins for digital output
  pinMode(FQ_UD,OUTPUT);
  pinMode(SCLK,OUTPUT);
  pinMode(SDAT,OUTPUT);
  pinMode(RESET,OUTPUT);
  
  // Configure LED pin for digital output
  pinMode(LED,OUTPUT);
  pinMode(INTLED,OUTPUT);
  
  // The unit is now "ON"
  LED_on();

  // Set up analog inputs, internal reference voltage
  pinMode(REV_ANALOG_PIN,INPUT);
  pinMode(FWD_ANALOG_PIN,INPUT);
  analogReference(INTERNAL);
  
  // initialize serial communication at 57600 baud
  Serial.begin(57600);

  // Reset the DDS
  ResetDDS();
  
  // Initialise the incoming serial number to zero
  serial_input_number=0;
}

// the loop routine runs over and over again forever:
void loop() {
  //Check for character
  if(Serial.available()>0){
    incoming_char = Serial.read();
    switch(incoming_char){
    case '0':
    case '1':
    case '2':
    case '3':
    case '4':
    case '5':
    case '6':
    case '7':
    case '8':
    case '9':
      serial_input_number=serial_input_number*10+(incoming_char-'0');
      break;
    case 'A':
      //Turn frequency into FStart
      Fstart_MHz = ((double)serial_input_number)/1000000;
      serial_input_number=0;
      break;
    case 'B':
      //Turn frequency into FStop
      Fstop_MHz = ((double)serial_input_number)/1000000;
      serial_input_number=0;
      break;
    case 'C':
      //Turn frequency into FStart and set DDS output to single frequency
      Fstart_MHz = ((double)serial_input_number)/1000000;
      SetDDSFreq(Fstart_MHz*1000000);
      serial_input_number=0;    
      break;
    case 'D':
      // Set the settle delay in the sweep
      settle_delay = serial_input_number;
      serial_input_number=0;
      break;
    case 'E':
      // Oscilloscope reads rEverse detector
      oscilloscope_pin = REV_ANALOG_PIN;
      break;
    case 'F':
      // Oscilloscope reads Forward detector
      oscilloscope_pin = FWD_ANALOG_PIN;
      break;
    case 'N':
      // Set number of steps in the sweep
      num_steps = serial_input_number;
      serial_input_number=0;
      break;
    case 'O':
    case 'o':
      Perform_oscilloscope();
      break;
    case 'Q':
    case 'q':
      Serial.println("K6BEZ Antenna Analyser, modifications by M0CUV");
      Serial.println("Commands: ABCDEFNOQSRV?");
      break;
    case 'S':    
    case 's':    
      Perform_sweep();
      break;
    case 'R':    
    case 'r':    
      ResetDDS();
      Serial.println("Reset");
      break;
    case 'V':
    case 'v':
      Read_voltages();
      break;
    case '?':
      // Report current configuration to PC    
      Serial.print("Start Freq:");
      Serial.println(Fstart_MHz*1000000);
      Serial.print("Stop Freq:");
      Serial.println(Fstop_MHz*1000000);
      Serial.print("Num Steps:");
      Serial.println(num_steps);
      break;
    }
    Serial.flush();     
  } 
  
  fade();
}

void fade() {
  countdown --;

  if (countdown == 0) {
    countdown = 1024;

    analogWrite(INTLED, brightness);    
    // change the brightness for next time through the loop:
    brightness = brightness + fadeAmount;

    // reverse the direction of the fading at the ends of the fade: 
    if (brightness == 0 || brightness == 255) {
      fadeAmount = -fadeAmount ; 
    }     
  }  
}

void LED_on() {
  digitalWrite(LED, HIGH);   // turn the LED on (HIGH is the voltage level)
}

void LED_off() {
  digitalWrite(LED, LOW);    // turn the LED off by making the voltage LOW
}

// An oscilloscope showed some spikes in the detector voltages at regular
// intervals, so I thought I could filter these out by taking the mode of
// several measurements. However, the oscilloscope plot of this firmware
// and the C driver showed that the mode is typically 0, and the smoothed
// values are sinusoidal. So, just average it. 80 measurements is around
// two cycles.
double analog_read_value(int pin) {
double total = 0.0;
double reading;
int i;
  for (i=0; i< 80; i++) {
    reading = analogRead(pin);
    total += reading * reading;
  }
  return total / 80.0;
}

void Perform_sweep(){
  double FWD=0;
  double REV=0;
  double VSWR;
  double Fstep_MHz = (Fstop_MHz-Fstart_MHz)/num_steps;
  int LED_voltage = HIGH;
  int last_flip = 0;

  // Set the start frequency - sometimes, this doesn't seem to work.
  // Need a way of obtaining feedback that it has worked...
  SetDDSFreq(Fstart_MHz*1000000);
  SetDDSFreq(Fstart_MHz*1000000);
  SetDDSFreq(Fstart_MHz*1000000);
  SetDDSFreq(Fstart_MHz*1000000);
  
  // Start loop 
  for(int i=0;i<=num_steps;i++){
    //digitalWrite(LED, LED_voltage);
    
    if(Serial.available()>0) {
      Serial.println("Stop");
      break;
    }
    // Calculate current frequency
    current_freq_MHz = Fstart_MHz + i*Fstep_MHz;
    
    // Set DDS to current frequency
    SetDDSFreq(current_freq_MHz*1000000);

    last_flip += settle_delay;
    if (last_flip > 100) {
      LED_voltage = LED_voltage == HIGH ? LOW : HIGH;
      last_flip = 0;
    }
    
    // Read the forawrd and reverse voltages
    REV = analog_read_value(REV_ANALOG_PIN);
    FWD = analog_read_value(FWD_ANALOG_PIN);

    if(REV >= FWD){
      // To avoid a divide by zero or negative VSWR then set to max 999
      VSWR = 999;
    } else {
      // Calculate VSWR
      VSWR = (FWD+REV)/(FWD-REV);
    }
    
    // Send current line back to PC over serial bus
    Serial.print(current_freq_MHz*1000000);
    Serial.print(",0,");
    Serial.print(VSWR*1000.0);
    Serial.print(",");
    Serial.print(FWD);
    Serial.print(",");
    Serial.println(REV);
  }
  // Send "End" to PC to indicate end of sweep
  Serial.println("End");
  Serial.flush();    

  LED_on();
  ResetDDS();
}

void Perform_oscilloscope() {
const int buffer_size = 256;
int fwd_values[buffer_size];
int i;

  if (Fstart_MHz != 0) {
    // Set DDS to A frequency
    SetDDSFreq(Fstart_MHz*1000000);
  }

  for (i = 0; i < buffer_size; i++) {
    fwd_values[i] = (int)analogRead(oscilloscope_pin);
  }
  for (i = 0; i < buffer_size; i++) {
    Serial.print(i);
    Serial.print(" ");
    Serial.println(fwd_values[i]);
  }
  // Send "End" to PC to indicate end of scope
  Serial.println("End");
  Serial.flush();    

  LED_on();
  ResetDDS();
}

void Read_voltages() {
  double FWD=0;
  double REV=0;
  double VSWR;
  // Read the forawrd and reverse voltages
  REV = analog_read_value(REV_ANALOG_PIN);
  FWD = analog_read_value(FWD_ANALOG_PIN);
  if (REV >= FWD) {
    // To avoid a divide by zero or negative VSWR then set to max 999
    VSWR = 999;
  } else {
    // Calculate VSWR
    VSWR = (FWD+REV)/(FWD-REV);
  }
  Serial.print("VSWR: ");
  Serial.print(int(VSWR*1000));
  Serial.print(", Forward: ");
  Serial.print(FWD);
  Serial.print(", Reverse: ");
  Serial.println(REV);
}

void ResetDDS() {
  digitalWrite(RESET,HIGH);
  // tRS = 5 clock cycle reset width:
  // (5/125,000,000) secs in milliseconds = 4 ms
  delay(4);
  digitalWrite(RESET,LOW);
}

void SetDDSFreq(double Freq_Hz){
  // Calculate the DDS word - from AD9850 Datasheet
  int32_t f = Freq_Hz * 4294967295/125000000;
  // Send one byte at a time
  for (int b=0;b<4;b++,f>>=8){
    send_byte(f & 0xFF);
  }
  // 5th byte needs to be zeros
  send_byte(0);
  // Strobe the Update pin to tell DDS to use values
  digitalWrite(FQ_UD,HIGH);
  // 7.0ns
  delay(1);
  digitalWrite(FQ_UD,LOW);

  // Wait a little for settling
  delay(settle_delay);
}

void send_byte(byte data_to_send){
  // Bit bang the byte over the SPI bus
  for (int i=0; i<8; i++,data_to_send>>=1){
    // Set Data bit on output pin
    digitalWrite(SDAT,data_to_send & 0x01);
    // Strobe the clock pin
    digitalWrite(SCLK,HIGH);
    digitalWrite(SCLK,LOW);
  }
}
