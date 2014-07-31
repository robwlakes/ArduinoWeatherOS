/* 
 Manchester Decoding, reading by delay rather than interrupt
 Rob Ward Agust 2014
 This example code is in the public domain.
 Use at your own risk, I will take no responsibility for any loss whatsoever its deployment.
 Visit https://github.com/robwlakes/ArduinoWeatherOS for the latest version and
 documentation. Filename: DebugManchester.ino ReadMe: DebugMan.md
 
 Please leave comment or suggestions in the "Issues" feature on the GIT-Hub site
 
 Remember to check both Polarity settings for sensible data.
 
 DebugVersion_09, 28tyh July 2014
 
 */

//Interface Definitions
int RxPin           = 8;   //The number of signal from the Rx
int ledPin          = 13;  //The number of the onboard LED pin

// Variables for Manchester Receiver Logic:
word    sDelay     = 200;  //Small Delay about 1/4 of bit duration try like 220 to 480
word    lDelay          ;  //Long Delay about 1/2 of bit duration  try like 440 to 880, 1/4 + 1/2 = 3/4
byte    polarity   = 0;    //0 for lo->hi==1 or 1 for hi->lo==1 for Polarity, sets tempBit at start
byte    tempBit       ;    //Reflects the required transition polarity
byte    bitState      ;    //State of the RxPin 3/4 way through Bit Waveform
//Variables for Error detection
boolean noErrors   = true; //flags if signal does not follow Manchester conventions
byte    timeout       ;    //Maximum loops allowed to look for the next bit transition 
byte    loopCount     ;    //Incremented inside the loop looking for next bit transition
//variables for Header detection
byte    headerBits = 15;   //The number of ones expected to make a valid header
byte    headerHits = 0;    //Counts the number of "1"s to determine a header
boolean firstZero  = false;//has it processed the first zero yet?  This a "sync" bit.
//Variables for Byte storage
byte    discards   = 0;    //how many leading "bits" need to be dumped, usually just a zero if anything eg discards=1
byte    dataByte   = 0;    //Accumulates the bit information
byte    nosBits    = 0;    //Counts to 8 bits within a dataByte
byte    maxBytes   = 5;    //Set the bytes collected after each header. NB if set too high, any end noise will cause an error
byte    nosBytes   = 0;    //Counter stays within 0 -> maxBytes
//Variables for multiple packets
byte    bank       = 0;    //Points to the array of 0 to 3 banks of results from up to 4 last data downloads 
byte    nosRepeats = 0;    //Number of times the header/data is fetched at least once or up to 4 times
//Banks for multiple packets if required (at least one will be needed)
byte  manchester[4][20];   //Stores 4 banks of manchester pattern decoded on the fly

word  seconds =0;//used in the interrupt

/* Sample Printout, Binary for every packet, but only combined readings after three of the different packets have been received
 This is an example where the Sync '0' is inside the byte alignment (ie always a zero at the start of the packet)
 
 Using a delay of 1/4 bitWaveform 245 uSecs 1/2 bitWaveform 490 uSecs 
 Positive Polarity
 10 bits required for a valid header
 Sync Zero inside Packet
 D 00 00001111 01 22223333 02 44445555 03 66667777 04 88889999 05 AAAABBBB 06 CCCCDDDD 07 EEEEFFFF 08 00001111 90 22223333
 D 5F 01011111 14 00010100 28 00101000 C5 11000101 01 00000001 //Oregon Scientific Temperature
 D 54 01010100 98 10011000 20 00100000 A0 10100000 00 00000000 //Oregon Scientific Rainfall
 D 58 01011000 91 10010001 20 00100000 52 01010010 13 00010011 //Oregon Scientific Anemometer/Wind direction
 These are just raw test dumps. The data has to be processed and require 10-11 bytes for all the packet to be seen
 Oregon Scientific works on nibbles and these need to reversed ie ABCD become DCBA in each nibble
 Also the OS protocol has a simple checksum to assist with validation as the packets are only sent once a cycle
 */

void setup() {
  Serial.begin(115200);//make it fast so it dumps quick!
  pinMode(RxPin, INPUT);
  pinMode(ledPin, OUTPUT);

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


  Serial.println("Debug Manchester Version GIT-Hub V01");
  lDelay=2*sDelay;//just to make sure the 1:2 ratio is established. They can have some other ratio if required
  Serial.print("Using a delay of 1/4 bitWaveform ");// +-15% and they still seem to work ok, pretty tolerant!
  Serial.print(sDelay,DEC);
  Serial.print(" uSecs 1/2 bitWaveform ");//these may not be exactly 1:2 ratio as processing also contributes to these delays.
  Serial.print(lDelay,DEC);
  Serial.println(" uSecs ");
  if (polarity){
    Serial.println("Negative Polarity hi->lo=1"); 
  }
  else{
    Serial.println("Positive Polarity lo->hi=1"); 
  }
  Serial.print(headerBits,DEC); 
  Serial.println(" bits expected for a valid header"); 
  if (discards){
    Serial.print(discards,DEC);
    Serial.println(" leading bits discarded from Packet"); 
  }
  else{
    Serial.println("All bits inside the Packet");
  }
  timeout=lDelay/5;//Rough number to indicate max loops allowed, if exceeded usually wrong polarity chosen, heals itself if wrong
  Serial.print(timeout,DEC);
  Serial.println(" timeout for wrong Polarity");
  Serial.println("P 00 00001111 01 22223333 02 44445555 03 66667777 04 88889999 05 AAAABBBB 06 CCCCDDDD 07 EEEEFFFF 08 00001111 90 22223333"); 
  Serial.print("sDelay =");
  Serial.println(sDelay,DEC);
  //if packet is repeated then best to have non matching numbers in the array slots to begin with
  //clear the array to different nos cause if all zeroes it might think that is a valid 3 packets ie all equal
  eraseManchester();  //clear the array to different nos cause if all zeroes it might think that is a valid 3 packets ie all equal
}//end of setup

//Routine Driven by Interrupt, trap 1 second interrupts, and output every minute
ISR(TIMER1_COMPA_vect){
  seconds++;
  if (seconds == 300){//make 300 for each output ie 5 minutes
    sDelay = sDelay+10;
    seconds = 0;
    Serial.print("sDelay =");
    Serial.println(sDelay,DEC);
  }
} //end of interrupt routine

// Main routines, find header, then sync in with it, get a packet, and decode data in it, plus report any errors.
void loop(){
  tempBit=polarity^1; //these begin as opposites for each packet
  noErrors=true;
  firstZero=false;
  headerHits=0;
  nosBits=0;
  nosBytes=0;
  while (noErrors && (nosBytes<maxBytes)){
    loopCount=0;
    while(digitalRead(RxPin)!=tempBit){
      //pause here until a transition is found
      loopCount++;//track how long the wait is, too long=error!
    }//at Data transition, half way through bit pattern, this should be where RxPin==tempBit
    delayMicroseconds(sDelay);//skip ahead to 3/4 of the bit pattern
    // 3/4 the way through, if RxPin has changed it is definitely an error
    digitalWrite(ledPin,0); //Flag LED off!
    if (digitalRead(RxPin)!=tempBit){
      noErrors=false;//something has gone wrong, polarity has changed too early, ie always an error
    }//exit and retry
    else{
      //now grab the bitState before setting up for next bit & correct for polarity
      bitState = tempBit ^ polarity;//Polarity=1, invert OR Polarity=0, ignore
      //Now work on what the next bit waveform is going to be 
      delayMicroseconds(lDelay);
      //now 1 quarter into the next bit pattern,
      if(digitalRead(RxPin)==tempBit){ //if RxPin has NOT swapped, then next data bit value IS swapping
        //If the header is done, then it means data change is occuring ie 1->0, or 0->1
        //data transition detection must swap, so it loops for the opposite transition in the next bit waveform
        tempBit = tempBit^1;
      }//end of detecting no transition at end of bit waveform
      if(loopCount>timeout){
        noErrors=false;
        if ((!firstZero)&&(headerHits>headerBits)){
          //Serial.println(loopCount,DEC);
          polarity = polarity^1;
          //Serial.print("Polarity now =");
          //Serial.println(polarity,DEC);
        }
      }

      //Now process the tempBit state and interept data as definite 0 or 1's, (Polarity corrected by now) 
      if(bitState==1){ //1 data could be header or packet
        if(!firstZero){
          headerHits++;
          if (headerHits==headerBits){
            digitalWrite(ledPin,1); //valid header accepted, minimum required found
            //Serial.print("H");
          }
        }
        else{
          add(bitState);//already seen first zero so add bit in
        }
      }//end of dealing with ones
      else{  //bitState==0 could first error, first zero or packet
        // if it is header there must be no "zeroes" or errors
        if(headerHits<headerBits){
          //Still in header checking phase, more header hits required
          noErrors=false;//landing here means header is corrupted, so it is probably an error
        }//end of detecting a "zero" inside a header
        else{
          //we have our header, chewed up any excess and here is a zero
          if ((!firstZero)&&(headerHits>=headerBits)){ //if first zero, it has not been found previously
            firstZero=true;
            //Serial.print("!");
          }//end of finding first zero
          add(bitState);
        }//end of dealing with a zero
      }//end of dealing with zero's (in header, first or later zeroes)
    }//end of first error check
  }//end of while noErrors=true and getting packet of bytes
  digitalWrite(ledPin,0); //data processing exited, look for another header
}//end of mainloop

//Read the binary data from the bank and apply conversions where necessary to scale and format data
void analyseData(){ 
}

void add(byte bitData){
  if (discards>0){ //if first one, it has not been found previously
    discards--;
  }
  else{
    dataByte=(dataByte<<1)|bitData;
    nosBits++;
    if (nosBits==8){
      nosBits=0;
      manchester[bank][nosBytes]=dataByte;
      nosBytes++;
      //Serial.print("B");
    }
    if(nosBytes==maxBytes){
      hexBinDump();//for debug purposes dump out in hex and bainary
      //analyseData();//later on develop your own analysis routines
    }
  }
}

void hexBinDump(){
  //Print the fully aligned binary data in manchester[bank] array
  Serial.print(polarity,DEC);
  Serial.print(" ");
  for( int i=0; i < maxBytes; i++){ 
    byte mask = B10000000;
    if (manchester[bank][i]<16){
      Serial.print("0"); //Pad single digit hex
    }
    Serial.print(manchester[bank][i],HEX);
    Serial.print(" ");
    for (int k=0; k<8; k++){
      if (manchester[bank][i] & mask){
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

void eraseManchester(){
  //Clear the memory to non matching numbers across the banks
  //If there is only one packet, with no repeats this is not necessary.
  for( int j=0; j < 4; j++){ 
    for( int i=0; i < 20; i++){ 
      manchester[j][i]=j+i;
    }
  }
}





