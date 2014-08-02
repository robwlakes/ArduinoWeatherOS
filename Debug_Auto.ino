/* 
 Manchester Decoding, reading by delay rather than interrupt
 Rob Ward Agust 2014
 This example code is in the public domain.
 Use at your own risk, I will take no responsibility for any loss whatsoever its deployment.
 Visit https://github.com/robwlakes/ArduinoWeatherOS for the latest version and
 documentation. Filename: DebugManchester.ino ReadMe: DebugMan.md
 
 Please leave comment or suggestions in the "Issues" feature on the GIT-Hub site
  
 DebugAuto, 28th July 2014
 
 This version steps automatically through a series of delays 10uS at a time each 5mins
 and attempts to decode Manchester encoded signals.  The Polarity of the code can be either
 positive or negative.  This program detects longer intervals bewtween transitions
 as 'faulty' zeroes and automatically reverses the Polarity setting for subsequent
 packets.  Because it starts off at a fairly unlikely setting for a Weather Station
 (low durations between samples) few packets will be initially decoded, if any at all.
 You will have to be patient and wait until the timings get closer and data finally
 begins to appear.  This may involve looking at a static screen for quite a while,
 as it steps through the 10uS increments. When the data does begin to appear then
 note down the Polarity and what delay is best.  You should also allow it to go through
 to longer and longer delays to there the packets begin to reduce in numbers so you know
 you have the best mid point for the sDelay setting. 

 The aim of this program it to provide experimenters and hackers a simple to use tool that
 will reveal the basic settings to receive a particular systems codes.  The program can
 be easily modified to become the basis of permanent installations when an experimenter
 adds in their own calculations and procedures. I have overdone the comments but hopefully
 experimenters, especially beginners will appreciate the overflow! Experts delete them!
 For actual installations the program can be stripped back to its bare essentials by
 deleting all the debug and incrementing stuff etc and putting chosen final values for all
 the parameters.

 Considerable lattitude is allowed in choosing a sDelay value to set up the Manchester
 decoding and experimenters may find there are a range of values of sDelay to choose from.
 Generally aim for the mididle ground to give you a bit of room to handle errors on either
 side. Variations up to +-10-15% are tolerated due to the program syncing each bit
 to the middle transition event that always occurs in proper Manchester encoding.  So
 small errors are contirnually corrected and do not accumulate.
 
 So expect to see a fairly broad band of sDelays that will work OK.  Once you 
 are satisfied you have the sDelay and Polarity worked out (only two settings!)
 you can begin pulling the program apart for your own purposes and building your
 own version, eg what bits get discarded, how many bytes, packet repeat etc
 
 The knowledge for this program came from wanting to intercept the signals from
 a Bios (or Thermor) weather station to feed into my 24/7 weather www page.
 I had to get to get to grips with the Manchester encoding first and after many
 false starts I succeeded.  After that came the decryption of the packets and
 calculating of meaningful values.  My old Bios broke down and I swapped to an Oregon
 Scientific WS, and had to do it all over again.  Both quite different animals, so to
 speak, but both used Manchester Encoding.  Second time through was a lot quicker
 but after having tried to help people do the same on the Arduino Forums I figured
 I needed to publish some good stuff for people to read up on.
 
 After a while I wanted to streamline my Manchester decoding logic and the result
 of that is here in this program.  With better logic and a much tighter procedure
 (instead of one routine for the header and another for the packet data, this routine
 can do it all in one!) design, I have been able to make it almost automatic to 
 at least get some Manchester data from something like a weather station. If the 
 bit waveform has a duration very much above or below 1mS the automatic part will
 will fall down.  If the duration gets very short where the delay caused by the 
 processing becomes similar in size to the sDelay then program will probably work
 but will need fairly inspired alterations.  The simple relationship that
    lDelay=2*sDelay will probably break down. Then the sDelay and lDelay may need to
 to be determined independently.  For longer delays the 1:2 relation will work fine
 but the auto detection will need the loopCount and timeout turned into a word
 variable as it is only a byte varaible at the moment to maximise the speed of the
 loopCount++ counting.  As the delays get bigger counting in words will not matter.
     timeout=lDelay/5; may need to change as well to keep the timeout (the number
 of loop++'s) to be about 1.5 times the duration of the sDelay duration.
 
 Obviously once the validation of the packets is working ie checked by repeats or
 a numerical checksum, then if this process is repeated the best timings will become
 even more evident.  This checking will eliminate packets that don't have any
 obvious Manchester decoding errors, but do in fact have internal errors.
 
 Please read the other documents on my GIT-Hub site that accompany this program for
 a fuller understanding of the Manchester concepts so you can "do you own thing!!"
 
 I hope you enjoy your own personal journey as I have enjoyed mine and this program
 heklps you enjoy even more :-)  Best wishes for your progress!
 
 Rob Ward
 
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
byte    headerBits = 10;   //The number of ones expected to make a valid header
byte    headerHits = 0;    //Counts the number of "1"s to determine a header
boolean firstZero  = false;//has it processed the first zero yet?  This a "sync" bit.
word    nosHits    = 0;    //number hex dumps achieved (not necessarily error free, just hits!)
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


  Serial.println("Debug Manchester Version GIT-Hub V03");
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
  Serial.println(" initial timeout to detect wrong Polarity");
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
    Serial.print("Nos Hits =");//Number of successful Hex dumps (not necessarily fre of errors)
    Serial.println(nosHits,DEC);
    nosHits=0;
    Serial.print("sDelay =");//So the progress of the increments on sDelay can be monitored
    Serial.println(sDelay,DEC);
    lDelay = sDelay*2;//Assume the sampling delays are ratio 1:2
    timeout=lDelay/5; //Rough number to indicate max loops allowed, if exceeded usually wrong polarity chosen, heals itself if wrong
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
      loopCount++;//track how long the wait is, if too long=error!
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
      //when looping to detect the transition in middle of bit waveform it must never take too long
      if(loopCount>timeout){
        noErrors=false;//not allowed to sync on an incorrect transition
        //if in the header phase and looking for the sync 0 
        if ((!firstZero)&&(headerHits>headerBits)){
          //ending here means the zero was found but polarity was wrong
          //Serial.println(loopCount,DEC);//enable for debugging
          polarity = polarity^1;//switch polarity and try for next packet
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
      nosHits++;//keep track
      hexBinDump();//for debug purposes dump out in hex and bainary
      //analyseData();//later on develop your own analysis routines
    }
  }
}

//Read the binary data from the bank and apply conversions where necessary to scale and format data
void analyseData(){ 
}

void hexBinDump(){
  //Print the fully aligned binary data in manchester[bank] array
  Serial.print(polarity,DEC);//show what polarity was used to reveal the packet
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





