ArduinoWeatherOS
================

Arduino Uno, 433MhzRx and OS WMR86 Weather Station

This project allows the 433MHz signals from an Oregon Scientific WRM86 weather station to be intercepted and decoded using and Arduino Uno into simple decimal values. The values for Wind Direction, Wind Speed, Temperature, Humidity and Rainfall are then sent via the USB/Serial port on the Arduino to the host computer.  In my case I use a Python program to interpret this CSV formatted string of characters and plot the above parameters for display on my website http://www.laketyersbeach.net.au/weather.html

The Arduino listens continually for WMR86 sensor broadcasts and merges them every minute for an output of the CSV string.  There are three transmitters in the WMR86 package, Wind Direction+Wind Speed (average and gusts), Temperature+Humidity and Rainfall (cumulative total and rate).  They each have different periods for transmission eg Wind every 14Seconds, Temp/Hum amd Rainfall at longer periods.  This does cause overlap of sensor transmissions and inevitable corruption of the packet, however the protocol does have a simple arithmetic checksum based on nybbles that helps eliminate most bad data.  However the chance of substitution errors causing bad data to go un-detected is much higher than if a higher bit CRC based on a polynomial process.  Range validation would be necessary (in the Python or Arduino) to check if the resulting readings were sensible.  The program above reports every minute for all three sensors whether a good packet has been received or not so the Python program can sum the good and bad packets eqach minute and report the relative numbers each week in an email.

The Ardunio algorithm to decode the Macnchester protocol uses timed delays to sample the waveform from the 433MHz Rx and not interrupts and direct measurment of waveform transitions.  This has a number of implications.  The Arduino is continually sampling and analysing the incoming waveform.  As it is a dedicated processor in this application and has not other function this is not a problem. However it does simplify the reception and analysis of the waveforms.

The Manchester protocol is a bias neutral protocol where each bit is composed of a high and low signal of equal length (or close to it).  This means that a transmitter can be totally keyed off and on (ie the 433Mhz signal is mot modulated, but is either fully transmitting or 'silent')and no matter what the sequences of 1's and 0's are transmitted, on average the Tx will be on for the same time as it is off.  A bit in a Manchester waveform always has a high and low signal component for both 1's and 0's.  So a Bit Waveform for a 1 is a high signal followed by a low signal (hi/lo), and a Bit Waveform for a 0 is a low signal followed by a high signal (lo/hi). Each high and low signal duration are both about 430uS.  So a full Bit Waveform will last about 860uS. (as will be seen later the tolerance in these timings is quite wide).

The cheap 433Mhz Rx's available have a simple Automatic Gain Control and Bias Detection built in. The AGC allows the sensitivity ot be maximised and adjusted back when a 433MHz signal is recieved.  The Bias Detection allows the average of the output signal voltage to be detected and applied to one half of a comparator, the other comparator input has the received signal.  With Manchester protocol the on/off ratio is equal, so the Bias Detection will be half way, consequently the transitions of the signal from on to off and back again, can produce very clean logic signals with simple circuitry. This decoding program relies on the timing of transitions, rather than the overall shape of the waveform (ie it does not have to be a very clean square wave to work).

In other words, the decoding program needs to be able to determine whether it has seen a 430uS 'no signal' followed by a 430uS 'on signal' so has received a 0, or has received a 430uS 'no signal" followed by a 430uS 'on signal' and so received a 1. 'on signal' makes the Rx output go 'hi' and 'no signal' makes the Rx output go 'lo'.

How is this decoded?

The first very important information to have is what polarity of the Manchester encoding the system uses.  Arguments are put forward for both versions as being correct, but either polarity works as good as the other and is simple to work out.  The polarity used by OS is that a Data 1 is hi/lo and Data 0 is lo/hi.

However any hi/lo may either indicate the middle of a Data 1 Bit Waveform, or possibly a transition between two Bit Waveforms and not indicate anything. Similarly any lo/hi may indicate the middle of a Data 0 Bit or just a meaningless transition between two Bit Waveforms.

Curiously a long string of Data 1's will have the same looking waveform as a long string of Data 0's.  Whether they are 1's or 0's depends on where we begin to analyse, we would need some sort of marker, or unambiguous beginning point.

Critically the only garanteed meaningful transition of states always occurs in the middle of the Bit Waveform.  Should a Data 1 be followed by a Data 0, then the signal will be hi/lo-lo/hi, and for a Data 0 followed by a Data 1, the signal will be lo/hi-hi/lo (where the - (dash) separates the Bit Waveforms and the / (slash) indicates the middle of the Bit Waveforms).  If we have a Data 1 followed by a Data 1, it will be hi/lo-hi/lo and a Data 0 followed by a Data 0 it will be lo/hi-lo/hi.

The last two examples above, Data 11 and  Data 00, have transitions at both the - and the /. (Hence curious effect above) So if there is a change from a Data 1 to a Data 0, or Data 0 to a Data 1, then there will no transition at the - point, however if there is a 1 followed by a 1, or a 0 followed by a 0, then there will always be a transition at the - point (but opposite polarity). Any transition between the Bit Waveforms (ie the - above) can be thought of as 'optional" and really only needed to make sure the transition at the middle of the next Bit Waveform (ie the / above) is correct for the data bit being represented.  Consequently is essential to concentrate the algorithm around the center of the Bit Waveform.

Here we need to digress to RF practicalities... 

Part of the practical application of the Manchester protocol is to have a header sequence of bits, usually 30 or so 1's.  This stablizes the AGC and establishes the Bias Detection point so the simple 433Mhz Rx has a good chance of settling down and producing a clean logic waveform after say 10 on/off transmissions.  The decoding program can then sample the Bit Waveform by synchronising (ie looping and waiting) the algorithm to the on/off transition, which is the midway point of a Bit Waveform for a 1.  This how it works for the OS protocol, as this is its polarity hi/lo=1, lo/hi=0 (see above).

The algorithm is expecting a stream of Data 1's and to begin with, and looping for any hi/lo transition on the Rx as the middle of a Data 1 bit Waveform.  After detecting a hi/lo then it begins to check the waveform. To filter out noise, the algorithm resamples the Rx output about a 1/4 of a Bit Waveform later, to see if it is still lo.  If is lo then it was possibly a genuine middle of a 1 Bit Waveform, however if it is not a lo then it was not a genuine hi/lo middle of a 1 Bit Waveform, and the algorithm begins the search for a hi/lo transition all over again.  However if this initial test is true, it has possibly sampled a midpoint of a 1, then it waits for another half a Bit Waveform.  This timing is actually 1/4 of the way into the next Bit Waveform, and because we know we are looking for another 1, then the signal should have gone hi by then. If is not hi, then the original sample, that was possibly the mid point of a 1 Bit Waveform is rejected as it has not followed the 'rules', and the search (looping) for then next hi/lo transition begins at the start, all over again.

This simple filtering allows the program to detect and count the number of successfully detected Data 1's received, and once a minimum has been counted in sequence, then the program can assume it has a valid header coming in. This sampling, by looking for transitions and waiting periods of time to sample, is also later applied to any 0's received and can eliminate badly formed packets if the waveform pattern of the Manchester encoding is not followed.  Hence it forms a simple but effective digital filter that greatly reduces spurious results from random or unwanted 433MHz signals. 

Okay so we can detect a header... Let's say the Tx sends a header of 30 Data 1's, and it takes 5 Data ones to stabilise the Rx, then we have 25 left to detect.  If the program locks in and begins receiving the Data 1's and counts to the minimum require for a header, such as 15, then where will be 10 Data 1's more to go.  However we cannot be sure how many Data 1's it will take to stabilise the Rx, it maybe 5 some days, maybe 7 or 3 other days, so we need a marker to indicate the header has finished and the data or payload is beginning.

This is where the next bit of inside information is required, but it is fairly standard.  As any excess header Data 1 bits are being soaked up, the program is now looking for the header Data 1's to stop and a Data 0 to be detected. This 0 indicates that the data payload has begun.  The rules for interpretting the hi and lo's change slightly here.  As we know that we are dealing with a valid header, and we have a series of Data 1's detected, then the program is definitely synched to the mid-point of the Bit Waveform and can decode both Data 1 and 0's by continuing to sample at 3/4 time, and 1/4 time in the next Bit Waveform, to check the integrity of the waveform against the Manchester protocol, but also more importantly can now predict what the next transition will be, and when satisfied with the filtering/sampling results (which continues for every bit received) accumulate the data payload bits into into bytes.


Using our previous nomenclature -
     Tran  3/4  Mid   1/4
hi   /     lo   -     hi    will mean a hi/lo transition expected (ie a 1 followed by a 1), loop (wait) for a hi/lo

lo   /     hi   -     lo    will mean a lo/hi transition expected (ie a 0 followed by a 0), loop (wait) for a lo/hi

hi   /     lo   -     lo    will mean a lo/lo transition expected (ie a 1 followed by a 0), loop (wait) for a lo/hi

lo   /     hi   -     hi    will mean a hi/hi transition expected (ie a 0 followed by a 1), loop (wait) for a hi/lo



The synchronising bit and may or maynot be included in the byte boundaries.  Some implementations send a single 0 then all the bytes of information, whereas the OS example we are dealing with here just makes sure the first bit in the first byte sent, is always a 0.

To be continued...........
