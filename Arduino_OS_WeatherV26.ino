/*Status:  Yet another Oregon Scientific Weather Station 433Mhz Arduino Signal Intercepter
 *MIT Licence
 *Author: Rob Ward April 2014
 *Program Execution Summary-
 *Finds the header bits and synch's to the waveform's 1->0 going edges, which are '1's
 *Sweet spot for timing in this program is 240uS short pulse, 480uS long pulse, or thereabouts???
 *Soaks up rest of header bits and waits for first 0
 *Accumulates incoming data stream into bytes, arranged from MSB to LSB by rotating a mask in a Left Rotate
 *Checks for Sensor ID's, the ID's used here are relative to the previous way the data is collected, so not the same as other authors
 *Using the rotate bit on the fly the two ID bytes are as follows, this program just uses first byte to ID the sensor
 *        Oregon-THGN800   0xAF  	Outside Temp-Hygro    AF 82 ?? 4 ?? 1 (Sensor nybble 1,2or3)  10byte packet (9bytes used)
 *        Oregon-PCR800    0xA2 	Rain Gauge            A2 91 ?? 4 ?? 0                         11byte packet (11bytes used)
 *        Oregon-WGR800    0xA1 	Anemometer            A1 98 ?? 4 ?? 0                         10byte packet (10bytes used)
 *Rolling code 7bits is present to differentiate between alternative sensors, high bit set to 1 for 64 cycles means sensor is new.
 NB Rolling code ignored to ID the sensor as there are no other Oregon Scientifics around me and this avoids problems on sensor resets
 *Calculate the 8bit check sum of nybbles for the sensor and if data passes the test...continue  (2nybbles)
 *Decode the nybbles (if bites are bytes, nybbles are nybbles :-) and calculate the parameters for each sensor
 *On start up wait for a valid reading from each sensor and dump the calculations to the serial, round off to decimal places, every minute.
 
 Why not use interrrupts and count durations?  Manchester encoding came from the time of slow chips, noisey environments and saggy waveforms.
 The beauty of Manchester encoding is that it can be sampled so the 0V logic transitions are the most important and at least the logic 
 state in the middle of the timing periods is most likely to be valid.  It also self synchronises throughout the packet and 
 automatically detects timeouts.  This is an old, classic Manchester decoding decoding strategy, but quite robust never the less.
 Plus the Uno is only doing this task so no problem with it being in the foreground and chewing up most of the CPU cycles there.
 
 To do:
 *find the battery indicators if they exist. (7) suggests it is (my) upper 4 bits of byte[3].  How to fake old batteries??
 I put a 100ohm resistor across the terminals of the anemometer (in the springs) and chose some ordinary but OK batteries.
 I clipped it up and ran it for about 3 days.  After the first 24hr anemometer started to get get shaky and dropping readings
 However the byte[3]==4 all the time and did not change one bit.  I could not spot what bits if any were changing.
 I did the experiment at my son's house away from my system or it would have corrupted the "official anemometer" readings.
 A volt meter showed the voltage was dropping as expected but no clue discovered....
 
 Reference Material:
 Thanks to these authors, they all helped in some way, especially the last one Brian!!!!
 http://wmrx00.sourceforge.net/Arduino/OregonScientific-RF-Protocols.pdf (1)
 http://jeelabs.net/projects/cafe/wiki/Decoding_the_Oregon_Scientific_V2_protocol (2)
 https://github.com/phardy/WeatherStation (3)
 http://lucsmall.com/2012/04/27/weather-station-hacking-part-1/ (4)
 http://lucsmall.com/2012/04/29/weather-station-hacking-part-2/ (5)
 http://lucsmall.com/2012/04/29/weather-station-hacking-part-2/ (6)
 http://www.mattlary.com/2012/06/23/weather-station-project/(7)
 https://github.com/lucsmall/WH2-Weather-Sensor-Library-for-Arduino (8)
 http://www.lostbyte.com/Arduino-OSV3/ (9) brian@lostbyte.com
 
 Most of all thanks to Oregon Scientific, who have produced an affordable, high quality product.  I can now have my LCD Home Base in the
 kitchen to enjoy, with the Arduino in the garage also capturing data for WWW Weather pages.  Lovely!!!!  http://www.oregonscientific.com
 Very Highly recommended equipment. Rob Ward
 */
#include <Wire.h>
#include "DHT.h"
#include "Adafruit_BMP085.h"

#define DHTTYPE DHT22   // DHT 22  (AM2302)
#define DHTPIN 2     // what pin we're connected to

Adafruit_BMP085 bmp;
DHT dht(DHTPIN, DHTTYPE);

// Read data from 433MHz receiver on digital pin 8
#define RxPin 8            //Just an input
#define ledPin 13          //Human feedback
#define sDelay 230         //One Quarter Manchester Bit duration
#define lDelay 460         //One Half Manchester Bit duration
byte    headerHits=0;      //How many ones detected as a header bit
boolean header    = false; //State of header detection
boolean logic     = false; //State of the Manchester decoding
byte    signal    = 0;     //state of RF
boolean test230   = false;
boolean test460   = false;
int     maxBytes  = 11;    //sets the limits of how many data bytes will be required
int     nosBytes  = 0;     //counter for the data bytes required
boolean firstZero = false; //flags when the first '0' is found.
byte    dataByte  = 0;     //accumulates the bits of the signal
byte    dataMask  = 16;    //rotates left one bit for each Manchester bit received, so allows nybbles to be reversed on the fly
byte    nosBits   = 0;     //counts the shifted bits in the dataByte
byte    manchester[12];    //storage array for the data accumulated via manchester format
byte    battery   = 0;     // Indicator byte, merge battery details into this number
byte    quadrant  = 0;     //used to look up 16 positions around the compass rose
double  avWindspeed = 0.0;
double  gustWindspeed = 0.0; //now used for general anemometer readings rather than avWinspeed
float  rainTotal = 0.0;
float  rainRate  = 0.0;
double  temperature = 0.0;
int     humidity  = 0;
double  intTemp;
double  intHumi;
double  intPres;
int     scan=0; //On start up lower three bits used to show data from all sensors has been received before pooling and sending out data
byte    activity=0;//6 Bits, Flags whether good or bad checksum for a given sensor received in last minute 
int     seconds;
const char windDir[16][4] = {  
  "N  ", "NNE", "NE ", "ENE",  "E  ", "ESE", "SE ", "SSE",  "S  ", "SSW", "SW ", "WSW",  "W  ", "WNW", "NW ", "NNW"};

void setup(){
  Serial.begin(115200);
  //Serial.println("Robs V3.0 WMR86 Oregon Decoder");
  //Serial.print("Available Memory ");
  //Serial.println(availableMemory());
  pinMode(ledPin, OUTPUT);
  pinMode(RxPin, INPUT);
  digitalWrite(ledPin, HIGH);
  delay(100);//heart beat
  digitalWrite(ledPin, LOW);
  headerHits=0;

  //Tutorial on using the BMP05 Press/Temp transducer https://www.sparkfun.com/tutorials/253
  bmp.begin();    //start the barometer and temp packages

  // Initialize Timer1 for a 1 second interrupt
  // Thanks to http://www.engblaze.com/ for this section, see their Interrupt Tutorial
  cli();          // disable global interrupts
  TCCR1A = 0;     // set entire TCCR1A register to 0
  TCCR1B = 0;     // same for TCCR1B
  // set compare match register to desired timer count:
  OCR1A = 15624;
  // turn on CTC mode:
  TCCR1B |= (1 << WGM12);
  // Set CS10 and CS12 bits for 1024 prescaler:
  TCCR1B |= (1 << CS10);
  TCCR1B |= (1 << CS12);
  // enable timer compare interrupt:
  TIMSK1 |= (1 << OCIE1A);
  // enable global interrupts:
  sei();
}

//Routine Driven by Interrupt, trap 1 second interrupts, and output CSV every minute
ISR(TIMER1_COMPA_vect){
  seconds++;
  if (seconds == 60){//make 60 for each output
    seconds = 0;
    usbData();//comment this out to temporarily disable data every minute for debug
    activity=0;//just a number to check if all three Tx stations have been seen.
  }
} //end of interrupt routine

void loop(){
  //wait here for a header!!!
  // So far this appears as 'inverted' Manchester 1>0==1, 0>1==0 ??? (G. E. Thomas in 1949) http://en.wikipedia.org/wiki/Manchester_code
  while (header == false){
    while (digitalRead(RxPin)==1) { //Stay in loop while logic =1
      //loop while the RxPin==1, first half of bit pattern, just before data transition, 1 to 0
    }//exits when signal == 0, (1->0 falling edge found, transition to value 0) middle of bit pattern, ie the data edge
    delayMicroseconds(sDelay); //Short wait for a 1/4 of the "1" pattern
    if (digitalRead(RxPin) == 0){ //Check signal is still steady at 0 ( !0 = error detection)
      delayMicroseconds(lDelay); // long wait for next 1/2 of bit pattern, 
      // ie now at 3/4 way through, looks like an ok bit "1" now keep track of how many in row we have detected
      if (digitalRead(RxPin) == 1){ // Check Rx polarity has now swapped, good!
        headerHits ++; // Highly likely a "1" waveform, so count it in
        //Serial.print(headerHits);
        //Serial.print(" ");
        if (headerHits == 20){ //if we can find 20 in a row we will assume it is a header
          header = true; //so we can exit this part, collect rest of header and begin data processing below
          headerHits=0; //reset, so ready to search for another header when next required, or should an error occur in this packet
          //Serial.println("");
          //Serial.print("H"); //uncomment to debug header detection
        }
      }
      else {
        headerHits=0;  // Has not followed the "1" wave pattern, probably badly formed, noisy waveform, so start again
        header=false; // make sure we look for another header
      }
    }
    else {
      headerHits=0;  // Has not followed wave pattern, probably just noise, so start again
      header=false; // make sure we look for another header
    }
  }

  //The program is now synch'ed to the '1' waveform and detecting the 1->0  "data" transitions in the bit waveform
  //The data byte boundaries indicate the Synch '0' is considered a part of the data, so byte boundary begins at that '0'
  logic=1; // look for rest of header 1's, these must be soaked up intil first 0 arrives to denote start of data
  signal=0; //RF Signal is at 0 after 1's 1->0 transition, inverted Manchester (see Wiki, it is a matter of opinion)
  firstZero=false; //The first zero is not immediately found, but is flagged when found

  while (header == true){
    //now get last of the header, and then store the data after trigger bit 0 arrives, and data train timing remains valid 
    while (digitalRead(RxPin)!=signal){ //halt here while signal matches inverse of logic, if prev=1 wait for sig=0
    }//exits when signal==logic
    delayMicroseconds(sDelay); //wait for first 1/4 of a bit pulse
    test230 = digitalRead(RxPin);//snapshot of the input
    if ((test230 == signal)&&(nosBytes < maxBytes)){  //after a wait the signal level is the same, so all good, continue!
      delayMicroseconds(lDelay); //wait for second 1/2 of a bit pulse
      test460=digitalRead(RxPin);//snapshot of the input
      if (test230==test460){  // finds a long pulse, so the logic to look for will change, so flip the logic value 
        //Assuming the manchester encoding, a long pulse means data flips, otherwise data stays the same
        logic = logic^1;
        signal = signal^1;
        //Serial.print(logic,BIN);  //debug data stream in binary
        if (!firstZero){ //if this is the first 0->1 data transition this is the sync 0
          digitalWrite(ledPin,1); //data processing begins, first though chew up remaining header
          firstZero = true; //flag that legit data has begun
          dataByte = B00000000; // set the byte as 1's (just reflects the header bit that have preceded the trigger bit=0)
          dataMask = B00010000; // set the byte as 1's (just reflects the header bit that have preceded the trigger bit=0)
          nosBits = 0;  // preset bit counter so we have 7 bits counted already
          //Serial.print("!");  //debug detection of first zero
        }
      }
      //data stream has been detected begin packing bits into bytes
      if (firstZero){
        if (logic){
          dataByte = dataByte | dataMask; //OR the data bit into the dataByte
        }
        dataMask = dataMask << 1;//rotate the data bit
        if (dataMask==0){
          dataMask=1;//make it roll around, is there a cleaner way than this? eg dataMask *=2?
        }
        nosBits++;
        if (nosBits == 8){ //one byte created, so move onto the next one
          manchester[nosBytes] = dataByte; //store this byte
          nosBits = 0;     //found 8, rezero and get another 8
          dataByte = 0;    //hold the bits in this one
          dataMask = 16;   //mask to do reversed nybbles on the fly
          nosBytes++;      //keep a track of how many bytes we have made
        }
      }
    }  
    else {
      //non valid data found, or maxBytes equalled by nosBytes, reset all pointers and exit the while loop
      headerHits = 0;    // make sure header search begins again
      header = false;    // make sure we look for another header
      firstZero = false; // make sure we look for another 0->1 transition before processing incoming stream
      nosBytes = 0;      // either way, start again at beginning of the bank
    }

    //Temp has 10 bytes in packet, anemometer and rainfall have 11 bytes in packet
    if (manchester[0]==0xa2){
      maxBytes=11; //if looking for Temp then only collect 10 bytes
    }
    else{
      maxBytes=10; //if Anemometer or Rainfall then we must look for 11 bytes to include 2 byte Checksum at the end
    }
    //look for the required number of bytes and when reached process and check the data packet
    if (nosBytes == maxBytes){  
      if (manchester[0]==0xaf){  //detected the Thermometer and Hygrometer
        //binBank();
        if(ValidCS(16)){
          //hexBank();
          scan = scan | 1;
          activity = activity | 1;    // indicate good CS detected
          thermom();
          //dumpThermom();
        }
        else{
          activity = activity | 16;  // indicate bad CS detected
        }
      }
      if (manchester[0]==0xa1){      //detected the Anemometer and Wind Direction
        if(ValidCS(18)){
          //binBank();
          scan = scan | 2;
          activity = activity | 2;   // indicate good CS detected
          anemom();
          //dumpAnemom();
        }
        else{
          activity = activity | 32;  // indicate bad CS detected
        }
      }
      if (manchester[0]==0xa2){      //detected the Rain Gauge
        if(ValidCS(19)){
          //hexBank();
          scan = scan | 4;
          activity = activity | 4;   // indicate good CS detected
          rain();
          //dumpRain();
        }
        else{
          activity = activity | 64;  // indicate bad CS detected
        }
      }
      headerHits = 0;                            //wind header bit hits back to 0
      header = false;                            //Look for a new header
      nosBytes =0;                               //reset byte pointer into bank
      intHumi=(double)dht.readHumidity();        //DHT22 readings %Humidity
      intTemp=(double)bmp.readTemperature();     //internal temperature
      intPres=(double)bmp.readPressure()/100.0;  //Pa reduced to mBar
    }
  }
  digitalWrite(ledPin,0); //data processing ends, look for another header
} //  End of main loop

// Formating routine for interface to host computer

void usbData(){
  // Stn Id, Packet Type, Wind Quadrant, Wind Speed, Rain Tips, Ext temp, Int Temp, Int Pressure, Int Humidity
  if (scan == 7){  //all readings now valid
    Serial.print(activity); // 03210321 Hi-Nybble flags failed Sensor Checksums, Lo-Nybble flags good failed Sensor Checksums
    Serial.print(",");
    Serial.print(battery);  // 00332211 Indicator battery level for sensors 1,2,3 
    //Not sure what these bits mean, pretty sure so called battery bits are wrong, however an Indicator byte idea is an OK Idea
    //When and if the bits that indicate battery levels are found thye can go in here :-)
    //I have a suspicion they may use bad checksums or some such to detect when the Tx is getting low.  However that idea has a
    //weakness as any other Tx on 433 can scramble a packet, and each WMR86 has three Tx's all transmitting at different intervals
    //and will inevitably cause corrupted/bad packets when they overlap. Hence I have been monitoring checksums as well.
    Serial.print(",");
    Serial.print(quadrant);          //0-15 in 22.5 degrees steps clockwise
    Serial.print(",");
    Serial.print(gustWindspeed,1);   //Gust windspeed km/hr, not average windspeed
    Serial.print(",");
    Serial.print(rainTotal,1);       //yet to be checked for calibration mm
    Serial.print(",");
    Serial.print(temperature,2);     // OS Temperature Centigrade
    Serial.print(",");
    Serial.print(intTemp,2);         //BMP085 temperature (used for compensation reading) Centigrade
    Serial.print(",");
    Serial.print(intPres,2);         //BMP085 pressure reading milli-bars
    Serial.print(",");
    Serial.print(intHumi);           //Digital DHT22 seems better than the OS in Temp/Hum sensor % relative
    Serial.println();
  }
}

//Support Routines for Nybbles and CheckSum

// http://www.lostbyte.com/Arduino-OSV3/ (9) brian@lostbyte.com
// Directly lifted, then modified from Brian's work, due to nybbles bits now in correct order MSNybble->LSNybble
// CS = the sum of nybbles, 1 to (CSpos-1), compared to CSpos byte (LSNybble) and CSpos+1 byte (MSNybble);
// This sums the nybbles in the packet and creates a 1 byte number, and then compared to the two nybbles beginning at CSpos
// Note that Temp and anemometer uses only 10 bytes but rainfall use 11 bytes per packet. (Rainfall CS spans a byte boundary)
bool ValidCS(int CSPos){
  bool ok = false;
  byte cs = 0;
  for (int x=1; x<CSPos; x++){
    byte test=nyb(x);
    cs +=test;
  }
  byte check1 = nyb(CSPos);
  byte check2 = nyb(CSPos+1);
  byte check = (check2<<4)+check1;
  /*
  Serial.print(check1,HEX);  //dump out the LSNybble Checksum
   Serial.print("(LSB), ");
   Serial.print(check2,HEX);  //dump out the MSNybble Checksum
   Serial.print("(MSB), ");
   Serial.print(check,HEX);   //dump out the Rx'ed predicted byte Checksum
   Serial.print("(combined),  calculated = ");
   Serial.print(cs,HEX);      //dump out the calculated byte Checksum
   Serial.print("   ");       //Space it out for the next printout 
   */
  if (cs == check){
    ok = true;
  }
  return ok;
}
// Get a nybble from manchester bytes, short name so equations elsewhere are neater :-)
byte nyb(int nybble){
  int bite = nybble / 2;       //DIV 2, find the byte
  int nybb  = nybble % 2;      //MOD 2  0=MSB 1=LSB
  byte b = manchester[bite];
  if (nybb == 0){
    b = (byte)((byte)(b) >> 4);
  }
  else{
    b = (byte)((byte)(b) & (byte)(0xf));
  }       
  return b;
}

//Calculation Routines

/*   PCR800 Rain Gauge  Sample Data:
 //  0        1        2        3        4        5        6        7        8        9        A
 //  A2       91       40       50       93       39       33       31       10       08       02
 //  0   1    2   3    4   5    6   7    8   9    A   B    C   D    E   F    0   1    2   3    4   5  
 //  10100010 10010001 01000000 01010000 10010011 00111001 00110011 00110001 00010000 00001000 00000010
 //  -------- -------  bbbb---  RRRRRRRR 88889999 AAAABBBB CCCCDDDD EEEEFFFF 00001111 2222CCCC cccc
 
 // byte(0)_byte(1) = Sensor ID?????
 // bbbb = Battery indicator??? (7)  My investigations on the anemometer would disagree here.  
 // After exhaustive low battery tests these bbbb bits did not change
 // RRRRRRRR = Rolling Code Byte
 // 222211110000.FFFFEEEEDDDD = Total Rain Fall (inches)
 // CCCCBBBB.AAAA99998888 = Current Rain Rate (inches per hour)
 // ccccCCCC = 1 byte Checksum cf. sum of nybbles
 // Message length is 20 nybbles so working in inches
 Three tips caused the following
 1 tip=0.04 inches or 1.1mm (observed off the LCD)
 My experiment
 Personally I don't like this value I think mult by 25 (it was 42, then 31) and divide by 1000 should be closer to return mm directly.
 0.127mm per tip??? It looks close the above. Again can't vouch 100% for this, any rigorous assistance would be appreciated.
 */
void rain(){
  rainTotal = float(((nyb(18)*100000)+(nyb(17)*10000)+(nyb(16)*1000)+(nyb(15)*100)+(nyb(14)*10)+nyb(13))*25/1000.0);
  //Serial.println((nyb(18)*100000)+(nyb(17)*10000)+(nyb(16)*1000)+(nyb(15)*100)+(nyb(14)*10)+nyb(13),DEC);
  rainRate = float(((nyb(8)*10000)+(nyb(9)*1000)+(nyb(10)*100)+(nyb(11)*10)+nyb(12))*31.0/1000.0);
  //Serial.println((nyb(8)*10000)+(nyb(9)*1000)+(nyb(10)*100)+(nyb(11)*10)+nyb(12),DEC);
  battery  = (battery & B00111100)| (nyb(4)>>1);//lowest 2 bits in Indicator byte is for Rain bits (unproven conc?!?!)
}
void dumpRain(){
  Serial.print("Total Rain ");
  Serial.print(rainTotal);
  Serial.print(" mm, ");
  Serial.print("Rain Rate ");
  Serial.print(rainRate);
  Serial.println(" mm/hr ");
}

// WGR800 Wind speed sensor
// Sample Data:
// 0        1        2        3        4        5        6        7        8        9
// A1       98       40       8E       00       0C       70       04       00       34
// 0   1    2   3    4   5    6   7    8   9    A   B    C   D    E   F    0   1    2   3
// 10100001 10011000 01000000 10001110 00000000 00001100 01110000 00000100 00000000 00110100
// -------- -------- bbbb---- NRRRRRRR xxxx9999 xxxxxxxx CCCCDDDD xxxxFFFF 0000---- CCCCcccc
// Av Speed 0.4000000000m/s Gusts 0.7000000000m/s  Direction: N  

// byte(0)_byte(1) = Sensor ID?????
// bbbb = Battery indicator??? (7)  My investigations would disagree here.  After exhaustive low battery tests these bits did not change
// NRRRRRRR = Rolling Code Byte, the N bit is set to 1 for 64 cycles to indicate it is reset or new to the Rx box
// 9999 = Direction
// DDDD.CCCC = Gust Speed (m per sec)
// 0000.FFFF = Avg Speed(m per sec)
// multiply by 3600/1000 for km/hr
// ccccCCCC = 1 byte checksum cf. sum of nybbles
// packet length is 20 nybbles

void anemom(){
  //D A1 98 40 8E 08 0C 60 04 00 A4
  avWindspeed = ((nyb(16)*10)+ nyb(15))*3.6/10;
  gustWindspeed =((nyb(13)*10)+nyb(12))*3.6/10;
  quadrant = nyb(9) & 0xF;
  battery  = (battery & B00110011)|(nyb(4)<<1);//Indicator middle 2 bits (bits 2&3) in byte is Anemometer (unproven data?!?!)
}
void dumpAnemom(){
  Serial.print("Av Speed ");
  Serial.print(avWindspeed);
  Serial.print(" km/hr, Gusts ");
  Serial.print(gustWindspeed);
  Serial.print(" km/hr, Direction: ");
  Serial.print(quadrant);
  Serial.print(" -> ");
  Serial.println(windDir[quadrant]);
}

// THGN800 Temperature and Humidity Sensor
// 0        1        2        3        4        5        6        7        8        9          Bytes
// 0   1    2   3    4   5    6   7    8   9    A   B    C   D    E   F    0   1    2   3      nybbles
// 01011111 00010100 01000001 01000000 10001100 10000000 00001100 10100000 10110100 01111001   Bits
// -------- -------- bbbbcccc RRRRRRRR 88889999 AAAABBBB SSSSDDDD EEEE---- CCCCcccc --------   Explanation
// byte(0)_byte(1) = Sensor ID?????
// bbbb = Battery indicator??? (7), My investigations on the anemometer would disagree here.  After exhaustive low battery tests these bits did not change
// RRRRRRRR = Rolling code byte
// nybble(5) is channel selector c (Switch on the sensor to allocate it a number)
// BBBBAAAA.99998888 Temperature in BCD
// SSSS sign for negative (- is !=0)
// EEEEDDDD Humidity in BCD
// ccccCCCC 1 byte checksum cf. sum of nybbles
// Packet length is 18 nybbles and indeterminate after that
// H 00 01 02 03 04 05 06 07 08 09    Byte Sequence
// D AF 82 41 CB 89 42 00 48 85 55    Real example
// Temperature 24.9799995422 degC Humidity 40.0000000000 % rel
void thermom(){
  temperature = (double)((nyb(11)*100)+(nyb(10)*10)+nyb(9))/10; //accuracy to 0.01 degree seems unlikely
  if(nyb(12)==1){//  Trigger a negative temperature
    temperature = -1.0*temperature;
  }
  humidity = (nyb(14)*10)+nyb(13);
  battery  = (battery & B00001111)|(nyb(4)<<3);//Indicator 2 bits (bits 4&5) in byte is Temperature  (unproven data?!?!)
}
void dumpThermom(){
  Serial.print("Temperature ");
  Serial.print(temperature);
  Serial.print(" degC, Humidity ");
  Serial.print(humidity);
  Serial.println("% Rel");
}

// Handy Debugging Routines

void binBank(){
  //Print the fully aligned binary data in manchester[] array
  Serial.println("D 00 00001111 01 22223333 02 44445555 03 66667777 04 88889999 05 AAAABBBB 06 CCCCDDDD 07 EEEEFFFF 08 00001111 90 22223333"); 
  Serial.print("D ");
  for( int i=0; i < maxBytes; i++){ 
    byte mask = B10000000;
    if (manchester[i]<16){
      Serial.print("0"); //pad single digit hex
    }
    Serial.print(manchester[i],HEX);
    Serial.print(" ");
    for (int k=0; k<8; k++){
      if (manchester[i] & mask){
        Serial.print("1");
      }
      else{
        Serial.print("0");
      }
      mask = mask >> 1;
    }
    Serial.print(" ");
  }
  Serial.println();
}

void hexBank(){
  //Print the fully aligned binary data, enable the headers if desired
  //Serial.println("H 00 01 02 03 04 05 06 07 08 09");
  //Serial.println("  00 00 00 00 00 00 00 00 11 11");
  //Serial.println("B 10 32 54 76 98 BA DC FE 10 32");
  Serial.print("D ");
  for( int i=0; i < maxBytes; i++){ 
    if (manchester[i]<16){
      Serial.print("0"); //pad single digit hex
    }
    Serial.print(manchester[i],HEX);
    Serial.print(" ");
  }
  Serial.println();
}

int availableMemory() {
  int size = 2048; //Arduino Uno, just keep an eye on this
  byte *buf;
  while ((buf = (byte *) malloc(--size)) == NULL);
  free(buf);
  return size;
}






