General Manchester Debugger Ver8
================================

Hi, here are some rough notes for the moment to accompany the Manchester Debug Program Version 8.

Only a few things need to be changed and tested early, others much later. Pretty much an easy recipe...

>  // Variables for Manchester Receiver Logic:

>  word    sDelay     = 245;  //Small Delay about 1/4 of bit duration  begin with 250 (alter early)

>  word    lDelay     = 390;  //Long Delay about 1/2 of bit duration  begin with 500, 1/4 + 1/2 = 3/4 (alter early)

The above delays are the first things to work on.  They do not need to be very exact as the timing can be out by about +-15 and the encoding will tolerate it ok.  However a CRO, or something like Audacity used as  simple storage CRO, can get you into the right ball park much quicker.  This can also help you decide whether the polarity is positive or negative Manchester.  Either are equally likely and it is just a matter of knowing what your system uses.  Again some sort of CRO is a help here.  As there are only two possibilities, there is no problem with trial and error here. So beginning by just playing with the sDelay, lDelay and the Polarity settings can be enough to get you some decoded signal.  You can fine tune other parts as time goes on.

>  byte    polarity   = 1;    //0 for lo->hi==1 or 1 for hi->lo==1 for Polarity, sets tempBit at start (alter early)

>  byte    tempBit    = 1;    //Reflects the required transition polarity (no need to alter)

The variable tempBit simply follows the raw data coming in, but is exclusive-OR'ed with Polarity to invert it (if Neg Polarity is selected) before being packed into the byte array.

>  boolean firstZero  = false;//flags when the first '0' is found. (no need to alter)

>  boolean noErrors   =true; (no need to alter)

These two variables above need not be changed.

>  //variables for Header detection

>  byte    headerBits = 10;   //The number of ones expected to make a valid header (alter later)

>  byte    headerHits = 0;    //Counts the number of "1"s to determine a header (no need to alter)

Your CRO will also give you an idea of how many header bits are part of the protocol.  If you think there are about 20 header bits (1's) then set the headerBits variable to say half, in this example 10 headerBits are counted before it is accepted as a valid header. 

>  //Variables for Byte storage

>  byte    discards    =0;      //Expecting all bits to be inside byte boundaries, set to number of leading bits to be dumped (alter later)

You may need to set the variable discards (eg =0 means no discards, sync zero is included in the packet, or =1, discard the sync zero from the packet, or =2 discards the first two bits) depending on how the byte boundaries line up.  You will need to be aware when trying to process the data that it maybe not alligned for the easiest processing if the sync zero, for example, is included in the first byte or excluded.

>  byte    dataByte   = 0;    //Accumulates the bit information (no need to alter)

>  byte    nosBits    = 0;    //Counts to 8 bits within a dataByte (no need to alter)

>  byte    maxBytes   = 5;    //Set the bytes collected after each header. NB if set too high, any end noise will cause an error (alter later)

Set maxBytes to quite low to begin with until you can get very stable packet reception (eg many the same or "explainably" different), then experiment with how high you can set the number and still get "valid" packets (this maybe hard to test until you have sorted repetition or checksum out as well). NB If you set the number of bytes too high you will not see packets at all, as any scrambled data received after the valid bits/bytes will cause the program to exit, and not show you anything.  Some protocols eg Oregon Scientific have different packet lengths for various sensors.  This has to be detected on the fly and maxBytes changed before the packet reception has finished. So a fair bit of modification would be require to accomodate this, but not difficult.

>  byte    nosBytes   = 0;    //Counter stays within 0 -> maxBytes (no need to alter)

>  //Variables for multiple packets

>  byte    bank       = 0;    //Points to the array of 0 to 3 banks of results from up to 4 separate, but sequential, data downloads (alter later, if required)

Some Manchester protocols do not have a checksum, they merely quickly repeat the packet say three of four times and each packet repeated has to be stored into seperate banks and later compared.  If all four banks are the same then the packet is declared valid.  In this program only one bank is used, but there are four defined if you find you need to go down that path.

>  byte    nosRepeats = 0;    //Number of times the header/data is fetched at least once or up to 4 times (alter later)

>  //Banks for multiple packets if required (at least one will be needed), four provided here...

>  byte  manchester[4][20];   //Stores 4 banks of manchester pattern decoded on the fly (alter later)

So in a nutshell

>1. Get an idea of what sDelay and lDelay ought to be (lDelay is usually 2*sDelay)

>2. Get an idea of how many header bits you want to look for (10 is OK to begin with)

>3. Experiment with polarity setting

You should aim for steady, repeatable reception at this stage... then

>1. See if packets are repeated for validation (then use banks for validation code)

>2. Experiment with how many bytes are used (build up gradually)

>3. Work out whether all bits are included in, or some are excluded from, the packet bytes

After all this, you need to begin processing the data bytes in the first Manchester array to see if you can replicate the readings from your console.  You will need to keep the discard bits problem in mind here (how many leading bits included or excluded in the packet?).  Data may not make sense easily if this is not sorted out (ie binary overlapping into adjacent bytes can be messy to decode!)

To improve reliability of the reception you will also need to be able reject bad packets.  This will probably be by either repetition of packets (ie get 4 in a row the same, and its valid!!) or a single packet with a simple checksum (or a more complicated cyclic redundancy polynomial algorithm).  Manchester encoding does not have inherent data error checking, just waveform checking.  It is quite tolerant of timing varations, but any error detection in the data must be added at the next level after the raw bytes have been received.  Good luck!

Rob Ward

PS A very interesting observation for many people who have problems with Weather Stations connected by RF, eg 433MHz, is that if there are sensors close by that are a relatively strong transmitter they can upset the AGC of the simpler 433MHz Rx'ers to the point where a more remote sensor will not be received. The AGC does not recover quick enough to correctly respond to the weaker signal and probably causes the header to timeout (not enough hits) or just prone to errors.  So it may be advantageous in some situations to move stronger signals further away.  The Rain Fall sensor Tx with the Oregon Scientifics is much stronger than the Temperature Tx for example.  Buying a good modern receiver brand like Dorji can make all the difference here. eg RF-DRA886RX-S, the Dorji 433MHZ Transceiver 13dBm SMA CONN

