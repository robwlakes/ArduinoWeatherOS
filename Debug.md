Hi here are some rough notes for the moment ot accompany the Manchester Debug Program.

Some things need to be changed and tested early, others much later.

  // Variables for Manchester Receiver Logic:
  word    sDelay     = 245;  //Small Delay about 1/4 of bit duration  begin with 250
  word    lDelay     = 390;  //Long Delay about 1/2 of bit duration  begin with 500, 1/4 + 1/2 = 3/4

The above delays are cirtical to get right early on.  They do not need to be exact as the timing can be out by about +-15 and the encoding will tolerate it ok.  However a CRO, or something like Audacity used as  simple storage CRO, can get you into the right ball park much quicker.  This can also help you decide whether the polarity is positive or negative manchester.  Either are equally acceptable and it is just a matter of knowing what your system uses.  Again some sort of CRO is a help here.  Though there are only two possibilities so no problem trial and error here.

  byte    polarity   = 1;    //0 for lo->hi==1 or 1 for hi->lo==1 for Polarity, sets tempBit at start
  byte    tempBit    = 1;    //Reflects the required transition polarity

The variable tempBit follows the raw data coming in, but is exclusive-OR'ed with Polarity to invert it, if Neg Polarity is selected.

  boolean firstZero  = false;//flags when the first '0' is found.
  boolean noErrors   =true;

These two variables need not be changed.

  //variables for Header detection
  byte    headerBits = 10;   //The number of ones expected to make a valid header
  byte    headerHits = 0;    //Counts the number of "1"s to determine a header

Your CRO will also give you an idea of how many header bits are part of the protocol.  If you think there are about 20 header bits (1's) then set the headerBits variable to say half, in this example 10 headerBits are counted before it is accepted as a valid header. 

  //Variables for Byte storage
  boolean sync0In=true;      //Expecting sync0 to be inside byte boundaries, set to false for sync0 outside bytes

You may need to set the variable sync0In as either true or false depending on how the byte boundaries line up.  You will need to be aware when trying to process the data that it maybe not alligned for the easiest processing if the sync zero is included in the first byte or excluded.

  byte    dataByte   = 0;    //Accumulates the bit information
  byte    nosBits    = 0;    //Counts to 8 bits within a dataByte

  byte    maxBytes   = 5;    //Set the bytes collected after each header. NB if set too high, any end noise will cause an error

Set maxBytes to quite low to begin until you can very stable packet reception, then experiment with how high you can set the number and still get valid packets. NB If you set the number of bytes too high you will not see packets at all as the scrambled data received after the valid bits/bytes will cause the program to exit and not show you anything.  Some protocols eg Oregon Scientific have different packet lengths for various sensors.  This has to be detected on the fly and maxBytes changed before the packet reception has finished.

  byte    nosBytes   = 0;    //Counter stays within 0 -> maxBytes
  //Variables for multiple packets
  byte    bank       = 0;    //Points to the array of 0 to 3 banks of results from up to 4 last data downloads

Some Manchester protocols do not have a checksum, they merely quickly repeat the packet say three of four times and each packet repeated has to be stored into seperate banks and later compared.  If all four banks are the same then the packet is declared valid.  In this program only one bank is used, but there are four defined if you find you need to go down that path.

  byte    nosRepeats = 0;    //Number of times the header/data is fetched at least once or up to 4 times
  //Banks for multiple packets if required (at least one will be needed)
  byte  manchester[4][20];   //Stores 4 banks of manchester pattern decoded on the fly

So in a nutshell
1. Experiment with number of byte at the end
2. Get an idea of what sDelay and lDelay ought to be
3. Get an idea of how many header bits you want to look for (10 is OK to begin with)
3. Experiment with polarity setting
You should aim for steady reception at this stage... then
4. See if packets are repeated for validation
5. Experiment with how many bytes are used
6. Work out whether the sync 0 is included in or excluded from the packet bytes

After all this you need to begin processing the data in the Manchester array to see if you can replicate the readings from your console.  You will need to keep the sync 0 problem in mind here.

To improve reliability of the reception you will also need to be able reject bad packets.  It will probably be either reptition of packets or a single packet with a checksum (or a more complicated polynomial algorithm).

