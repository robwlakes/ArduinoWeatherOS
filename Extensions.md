Weather Station Extension
=========================  
### Extension of the Project
Several features became desirable after the original system had been running for several years.
The main concern was a lack of Battery Level indicators and also no direct feedback from the Arduino module
of the signals being received. The solution was tackled in two ways, but involving similar parts of the program.

### Battery Levels

The Battery Levels were a harder one to crack.  I checked the WMR86 manual for mention of a battery level indicator and also checked the LCD screen supplied with it for any indicators that referred to battery levels.
I could not find any.  So I devised this strategy.  Should any sensor not be logged within a minute (ie between sending strings of data to the WWW server) then a number is incremented for that Sensor.  As soon as that
sensor is detected the number is reset to Zero.  However if the sensor is not detected for 20 minutes then a flag bit is set for that Sensor.  This "detection" number is sent to the WWW server along with all the other
data every minute.  If it remains 0 then the sensors are OK, if it is non-zero, then it means one of the three sensors have experienced a "continuous down time" of at least 20 minutes.  The WWW server can check this and send out an Email
at midnight of that day warning of which sensor(s) have been missed.  However the warning could arise for a number of reasons, the first obvious one is the batteries are going flat and the second is that something maybe
effecting either the transmitter or receiver.  Eg the Antenna may have changed position on the Rx and suddenly 2 outof the three are being received.  Or maybe some object has been placed bewteen the Tx and Rx and
that has knocked out the signal. However, just because the signal drops out it may not be because of the Batteries. The system should be sensitive enough to warn of early stages of battery failure.  Even if it begins by 
small periods of time overnight in the cool night air.  

### Status Indicator
  The RGB LED should give a quick indication if an antenna position change for example is still allowing the sensors to be received.
  An RGB LED was added to the board to indicate which sensor transmitter was detected.  By combining Red, Blue and/or Green I could easily produce 8 recognisable states from the LED.
  Simply observing the board would indicate that the different sensors were being logged.  This gave a quick indication, for example after a restart that the Arduino was functioning properly,
  and so were the 4 Oregon sensors
#### Colour coding was:
#### Red = Temperature and Humidity
#### Green = Wind Direction and Speed
#### Blue = Rain Gauge
#### Yellow = UV Light Sensor
#### Purple = Experimental

![alt text](images/RGB_Status.jpg?raw=true "RGB Status LED")


### Software Version

The version to use these features is MainWeather_09.ino.  Please note the order of the sensors in the output string has also been changed.  So if you have a system reading this string and processing it from the previous version
you will have to alter it as well to use this program as it is.



