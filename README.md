GPS data is communicated in a data structure known as NMEA. This contains a multitude of information most of which is surplus to requirement. A key requirement of the GPS system is the translation of a NMEA string such as below into a friendlier format.

\quad \textit{GPGGA,110617.00,4123.56789,N,00831.54761,W,1,05,2.68,129.0,M,50.1,M,,*42}  

As the bearing from point A to B is required, the coordinates of both must be known. Once retrieved the following equations may be used to calculate the bearing from magnetic north between A and B:

\begin{equation}
x = \cos(\theta_{B})\sin({\Delta L})
\end{equation}
\begin{equation}
y = \cos(\theta_{A})\sin(\theta_{B})-\cos(\theta_{B})\sin(\theta_{A})\cos({\Delta L})
\end{equation}
\begin{equation}
\beta = \arctan2(x,y)
\end{equation}

Where \(\theta_{A}\) is the latitude of point A, \(\theta_{B}\) is the latitude of point B and \(\Delta{L}\) is the difference in longitude of point A and B. \(\beta{}\) is the angle in degrees from magnetic north to point B. 

Now that a desired bearing is known, the system must be able to find its current position with regards to it. A magnetometer can be used to supply an angle from magnetic north, \(\alpha{}\), which may be used to find the platform's azimuth error from the desired position, \(\epsilon_{azimuth} \). This is used by the motor controller to drive the mechanical setup horizontally:

\begin{equation}
\epsilon_{azimuth} = \beta_{azimuth} - \alpha_{azimuth}
\end{equation}

\begin{figure}[H]
    \centering
    \includegraphics[scale=0.9]{Images/altitude_gps.PNG}
    \label{fig:gpsaltdiagram}
    \caption{GPS Altitude}
\end{figure}

To find the altitude bearing, data from the GPS modules may be used once again. The following equation is used to find \(\gamma\), the bearing in degrees from the horizon to the drone in the vertical axis.

\begin{equation}
\gamma = \arctan(\frac{h_{drone} - h_{base}} {d})*\frac{180}{\pi}
\end{equation}

Where \(h_{drone}\) and \(h_{base}\) are the height of the drone and base from sea-level, and \(d\) is the horizontal distance between them. The mechanical rig's current elevation must be found to create an error for the elevation control system.

\subsubsection{Detector Sampling}

The sampling program must be able to quickly and accurately measure the analogue output of an optical detector. This requires consistent and accurate timings, triggered automatically by the code through interrupts. 

Acquiring this data from all four detectors requires two distinct operations:
\begin{enumerate}
\item Periodically sampling at the output of the lock-in amplifier. \item Periodically switching the input of the lock-in to the next detector.
\end{enumerate}

The sampling period of the ADC must be suitable for the signal created by the lock-in amplifier. At the demodulated frequency of \(\Delta\omega = 30 Hz\), a suitable sample period is determined such that accurate digitisation is assured. It is estimated that 100 samples per period is sufficient, thus a sampling period \(T_{sampling} = 333\mu s\) is to be used. The multiplexer switching period must be fast enough to capture a full period of the \(\Delta\omega = 30 Hz\) signal, so that the magnitude of the signal can be inferred and thus switching every \(\omega_{m} = 20Hz\) is acceptable.

Once the signal magnitude is found for each detector, the corresponding axis may be compared to infer the beacon position relative to the detector array. By subtracting the 'right' detector's maximum from the 'left' detector's maximum, a differential value related to position is gained, denoted as \(\delta_{azimuth}\). The same applies for elevation, the 'top' minus the 'bottom' detector yields a value for \(\delta_{elevation}\). 

When the light is centred to all four detectors \(\delta_{azimuth}\) and \(\delta_{elevation}\) are both zero. If the beacon moves left, the left detector to receives more signal and the right detector receives less, driving \(\delta_{azimuth}\) positive. The magnitude of this positive value denotes the severity of the beacon's displacement. In a similar fashion, an increasingly negative value of \(\delta_{azimuth}\) denotes movement to the right of the rig. The same methodology is applied for \(\delta_{elevation}\). 

As the beacon moves further away however the magnitude of the captured signal will decrease, causing smaller differentials and hence a less accurate control response. To combat this, two sets of gain multiplexers are controlled by the software, and will automatically increase/decrease the gains dependant on the maximum value seen in a given time frame. 

\subsubsection{Control}

The fundamental design of the control system is to maintain a zero differential between the optical detectors. For rotation about the azimuth, the two horizontal pairs of detectors are considered (detectors 1 \& 2 and 3 \& 4). If the value of \(\delta_{azimuth}\) is positive then the rig must rotate towards the left (-X direction), otherwise, if \(\delta_{azimuth}\) is negative then the control algorithm will make the motor turn so the rig rotates towards the right (+X direction). The concept for the elevation is the same, where a positive \(\delta_{elevation}\) results in the motor making the detectors and antenna point upwards and a negative \(\delta_{elevation}\) makes the detectors and antenna point downwards.

\begin{figure}[H]
    \centering
    \includegraphics[scale=0.9]{Images/differential.jpg}
    \label{fig:differential}
    \caption{Detectors Layout}
\end{figure}

Additionally, the motor speed needs to be controlled according to how far from the centre the beacon is. A larger differential equates to a faster rotation to correct the error. Since the stepper motor drivers only allow for a preset rpm to be used, the only way to control the motor speed is to select the stepping mode. Three stepping modes were created:
\begin{itemize}
\item Full step for fast rotation
\item \(\frac{1}{4}\) step for medium rotation
\end{itemize}
Since \(200RPM\) was the selected velocity, the above mentioned step sizes constitute to \(200RPM\) and \(50RPM\) respectively.

\subsubsection{Software Systems Overview}

Whilst official Arduino hardware was not used, the Arduino integrated development environment (IDE) was. Pre-existing libraries designed for Arduino compatible boards were readily available and large amounts of documentation and support exist aiding development.

The software duties were separated into two distinct systems, the control board micro-controller (MCU) which communicates with all systems and drives the motors, and the detector MCU which samples the optical detector and calculates the required differentials.

\begin{figure}[H]
\centering
\includegraphics[width=1\textwidth]{Images/DigitalSystemOverview.JPG}
\caption{\label{SoftwareSystemOverview}Software Systems Overview}
\end{figure}

\subsubsection{Control Module Micro-controller}

Due to time constraints both detection systems could not be integrated into one main program. As such two main programs run on the Teensy 3.6 depending on the current task.

\textbf{GPS Program}

As mentioned previously, key to the GPS sub system is the retrieval of NMEA data. Algorithm 1 below performs this operation and is called through a timer interrupt every second.
\newline
\begin{lstlisting}[language=Arduino, caption=Base GPS Data Retrieval, basicstyle=\scriptsize,]  
void baseGPSData() {

  cli();
  unsigned long start = millis();

  do {
    //If the serial port connected to the base GPS has data
    while (baseGPSPort.available()) {
    
      //Feed the gps.base object information via .encode
      gpsBase.encode(baseGPSPort.read());    
      BASE_LAT = gpsBase.location.lat();     //Latitude?
      BASE_LON = gpsBase.location.lng();     //Longitude?
      BASE_ALT = gpsBase.altitude.meters();  //Altitude?
      baseDataFlag = true;                   //data flag high
      
    }
  //Allow time for a full NMEA sentence to arrive
  } while (millis() - start < 1000);         
 
  baseDataFlag = false;
  sei();
  
}
\end{lstlisting}

A library called "TinyGPS++" is utilised here for the parsing of the raw NMEA data. Line 9 shows the '.encode' function, which creates the sentences from the received chars, and checks that the sentences are valid. Whilst NMEA contains a multitude of information, it is only the GPGGA string that is required, and it is from this that lines 10, 11 and 12 pick out the latitude, longitude, and altitude of the base station.

Data is fed to the encoding function for a whole second, to allow complete sentences to be formed from the data. A \textit{baseDataFlag} is returned with a boolean value depending on if the operation was successful or not. As the function handles shared global variables, cli(); and sei(); must be used at the start and end of the function to disable and re-enable further interrupts from accessing (and corrupting) the data.

Receiving GPS data from the drone requires Wi-Fi communications through the ESP8266 chip. Algorithm 2 given below receives and formats the drone's GPS data, and is called every time the ESP8266 sets a 'data available' pin high.
\newline
\begin{lstlisting}[language=Arduino, caption=Drone GPS Data Retrieval, basicstyle=\scriptsize,]
void serialEvent() {

  if (esp.available() > 0) {
    BUFFER[j] = esp.read();
    if (BUFFER[j] == 't') {
      //If the string in the buffer terminates with a t it is the laTitude data
      //Convert the current contents of the buffer into an int, save it and reset the buffer
      long LAT = atoi(BUFFER);
      DRONE_LAT = (double)LAT / 1000000;
      bufferReset(BUFFER);
      j = 0;
      
    } else if (BUFFER[j] == 'n') {
      
      ...same idea
      
    } else if (BUFFER[j] == 'a') {
      
      ...same idea
      
    } else if (BUFFER[j] == '\n') {
      //If a new line character is received trigger a data ready flag 
      j = 0;
      flag = 1;
    } else {
      j++;
    }
  }
}
\end{lstlisting}

One consideration is that the ESP8266 may receive chars only. This means the code was required to recombine the individual chars into a string, then convert the sting into a float. Recombining the chars was no issue, as in \(C\) a string is simply an array of chars. However, recombining the chars into the distinct latitude, longitude, and altitude variables proved troublesome.

As implemented above, all characters are placed into a buffer array (line 4) at a position dictated by index 'j'. If a predesignated character is found (such as 't') the buffer is converted into an integer then saved into the corresponding container (line 9). This container now holds the correct data, however it must be cast into the correct coordinate format (line 10). The buffer and its index are reset (line 11, 12). If no predesignated character is found, the index increases and data is placed into the next place in the array (line 35). An example of a string received by the ESP8266 is below:

\begin{center}
\begin{tabular}{ c c c c }
 Received & 12345678t & 12345678n & 20a\\ 
 Translated & Lat 12.345678 & Lon 1.2345678 & Alt 20  
\end{tabular}
\end{center}

From the above, the latitude, longitude and altitude coordinates are found. To calculate both our current and desired bearings we must call the \textit{gpsBearing()} function.
\newline
\begin{lstlisting}[language=Arduino, caption=GPS Bearing Calculations, basicstyle=\scriptsize,] 
void gpsBearing() {

  //Calculates the distance to the drone station. In implementation drone_lat/drone_lon will be recieved in an interrupt
  unsigned long distanceKmToBase =
  (unsigned long)TinyGPSPlus::distanceBetween(BASE_LAT, BASE_LON, DRONE_LAT, DRONE_LON);

  //Calculates the initial bearing that would lead to the drone
  double courseToBase =
    TinyGPSPlus::courseTo(BASE_LAT, BASE_LON, DRONE_LAT, DRONE_LON);

  //Calculates the vertical bearing
  double altitudeDifference = BASE_ALT - DRONE_ALT;
  float verticalBearing = (atan(altitudeDifference / distanceKmToBase)) * 180 / PI;

  //Calculates the horizontal bearing
  mag.readMag(&x, &y, &z);
  
  //Calculates the horizontal error...
  float horizontalError = (courseToBase - (360 - magHeading));
  //...and moves the rig by it
  azimuthRotate(horizontalError);
  
  flag = 0;
  
}
\end{lstlisting}

Again the TinyGPS++ library is used to find the distance and bearing between the base station and the drone on lines 5-8 and 11-14.

The method for calculating the vertical bearing, as laid out in the design section of the report, is implemented lines 17-18.

To read the compass bearing, the "SparkFun MAG3110" library was used. When testing the magnetometer on the Arduino Uno test MCU, the bearing was found without a hitch. When ported to the Teensy 3.6 however seemingly nonsensical results were found. The magnetometer has a 16-bit architecture and sends signed integers of the following form:

\quad 1000 1010 1101 0000

This number represents -30000 using 2's complement, however, if not explicitly defined, integers are as long as the CPU's architecture defines, and as such the 32-bit Teensy saves the value as follows:

\quad 0000 0000 0000 0000 1000 1010 1101 0000 

When converted using 2's compliment this is equal to 35536. To fix this bug, the library had to be edited to explicitly state that all integers are signed 16 bits wide (int16\textunderscore t), removing the 16 empty bits at the start of the number, and allowing the correct bearing value to be found.

After some processing both the current and desired azimuth bearing are compared to calculate the error by which the azimuth controller must move (line 36). This angle is sent through \textit{azimuthRotate()} which simply scales the value by the gear ratio between pinion and belt. A flow diagram illustrating the GPS program is given in Figure \ref{fig:GPS Main Program}.

\begin{figure}[H]
    \centering
    \includegraphics[width=\linewidth]{Images/GPSMainsoftware_flow_13_05_19_v1.pdf}
    \caption{GPS program flow diagram.}
    \label{fig:GPS Main Program}
\end{figure}

\textbf{Optical Tracking Program}

The main program for the optical detection system is somewhat simpler thanks to the reduced number of peripherals that it must communicate with.

Algorithm 4 is triggered by the Teensy 3.2 every time data is available.
\newline
\begin{lstlisting}[language=Arduino, caption=Optical Data Retrieval, basicstyle=\scriptsize,] 
void detectorData() {

  static byte ndx = 0;      //Recieved char index
  char endMarker = '\n';    //endMarker = newline char
  char rc;                  //Recieved char
    
  //Is there data on the detector UART port?  
  if (detectorUART.available() > 0) {
    //If there is read the port and save it into rc
    rc = detectorUART.read();           

    if (rc != endMarker) {        //If we have not reached the end of the sentence
      receivedChars[ndx] = rc;   // then fill the current position of the array with rc
      ndx++;                     //Increment ndx ready for the next char
      if (ndx >= numChars) {     //If the index is greater than or equal to the predefined size of the char array
        ndx = numChars - 1;      //Set the index to (size of array - 1)
      }
    } else {
      receivedChars[ndx] = '\0';   //Otherwise terminate the array
      ndx = 0;                     //Reset the counter
      
      //Convert the received char array into an integer
      differential = atoi(receivedChars);         
      Serial.println(differential);
    }
  }
}
\end{lstlisting}

This is similar to the WiFi serial event detailed in the GPS section. Here, if data is available (line 7) it is read and saved to 'rc' (line 8). If this received character is not a new line character, then it is saved into the current index in the character array (line 11). 

If the current index is greater than or equal to the size of the array, then the index is reduced by 1 (line 14) so that the array does not overflow before a new line is found. When it is, the terminator character is placed into the array and the index is reset (line 17-18). Finally the string is cast into an integer and saved as the detectors 'differential' value.

This differential value is used to drive the motors through the control algorithm, a snippet of which is given below. 
\newline
\begin{lstlisting}[language=Arduino, caption=Differential Motor Control, basicstyle=\scriptsize,] 
  //If at the centre do not move
  if ((differential > 0) && (differential < 30)) {
    digitalWrite(MODE2, 0);
    digitalWrite(MODE1, 0);
    digitalWrite(MODE0, 0);
    
    //Tell motor to stop moving
    stepper.disable();
  }

  //If at the far left
  else if (differential > 400) {
    //Set to full step mode
    digitalWrite(MODE2, 1);
    digitalWrite(MODE1, 0);
    digitalWrite(MODE0, 0);

    //Tell motor to rotate by 5 steps * scaler
    stepper.enable();
    azimuthRotate(5);
  }
\end{lstlisting}

Depending on the differentials magnitude and sign, the motors step mode (line 3-5, 14-16) and the amount of steps is makes (line 8, 19-20) is set. In the above, if the differential is in the 'zero' band, the motor is disabled. If the differential is greater than 400, this indicates a strong signal is the left detector, requiring a large movement to the left accomplished by a full step mode. The flow diagram of this program is found in Figure \ref{fig:Detector Main Program}.

\begin{figure}[H]
    \centering
    \includegraphics[]{Images/DetectorMainsoftware_flow_13_05_19_v1.pdf}
    \caption{Optical tracking program flow diagram.}  \label{fig:Detector Main Program}
\end{figure}

\subsubsection{Lock-in Amplifier Module Micro-controller}

Central to the detector software is the sampling of the lock-in amplifier. Whilst a simple command such as \textit{analogRead()} could be used, its resolution and speed cannot be controlled. Instead, the hardware ADC is directly manipulated. 
\newline
\begin{lstlisting}[language=Arduino, caption=ADC Setup, basicstyle=\scriptsize,]
  adc->setAveraging(16);
  adc->setResolution(12);
  adc->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED); 
  adc->setSamplingSpeed(ADC_SAMPLING_SPEED::HIGH_SPEED);
\end{lstlisting}

An advantage of using the Teensy 3.2 is it's ability to average a defined previous number of samples and return the result. This is set up on line 1, and gives a slight low-pass filter characteristic to the sampling operation. This is advantageous as it lessens noise present in the signal, especially as the resolution is quite high at 12 bits (line 2). From the chip manufacturer, the max ADC sampling and conversion speed recommended for a resolution of 12 is "HIGH" which is set on lines 3 and 4.

Once the ADC is setup, the sampling is preformed by \textit{"timer0 callback()"} every 500\(\mu\)s.
\newline
\begin{lstlisting}[language=Arduino, caption=ADC Sampling, basicstyle=\scriptsize,]
//Begins reading the ADC, preforms 16 sample averaging
void timer0_callback(void) {
  adc->startSingleRead(adcInput, ADC_0);
}
\end{lstlisting}

To change the detector circuit being sampled the \textit{"muxSwitch()"} function is called every 50ms. The function changes the active buffer accordingly and also calculates maximums as well as differentials when appropriate.
\newpage
\begin{lstlisting}[language=Arduino, caption=Detector Multiplexer Switching, basicstyle=\scriptsize,] 
void muxSwitch() {
  rubbishFlag = 1;                              //Set the rubbish buffer to receive data                       
  
  switch (muxIndex) {

    case 0:
    
      detectorMux(1);                           //Switch to detector 1
      left4diff = maximum(leftBufferArray);     //Find max of leftBuffer
        
      rightFlag = 1;                    //Set the right array to collect data                            
      leftFlag = 0;                     //...and disable the left array
      muxIndex++;                       //Force the next switch statement to run next call
      
    case 1:
         
      detectorMux(2);                          //Switch to detector 2
      right4diff = maximum(rightBufferArray);  //Find max of rightBuffer
      aziDiff = left4diff - right4diff;        //Calculate the azimuth differential
      
      rightFlag = 0;                    //Disable the right array
      leftFlag = 1;                     //...endable the left to collect data
      diffFlag = 1;                     //...and notify the main loop data is ready
      muxIndex = 0;                     //Reset the switch index
  }
}
\end{lstlisting}

In the times between the function calls, sample data is placed in an array. Every 50ms, Algorithm 8 changes the active array (line 11-12, 21-22), and attributes the maximum value of the last 50ms sampling period to whichever detector was last active (line 9 and 18). When the function reaches the last detector, the difference is found (line 19) and the main loop is instructed that data is ready to be sent (line 23). The method by which the maximum value of the array is found can be found in Algorithm 10 in the appendix.

In Algorithm 8 there are two detectors being sampled and three arrays being populated: 'rubbish', 'left' and 'right'. The 'rubbish' buffer simply throws away a portion of the received samples after switching. This is to account for the propagation delay of the signal through the lock-in amplifier as after switching, signal is still momentarily received from the previous detector. 
\newpage
This process of attributing samples to detectors (and rubbish heaps) is carried out in the code below, which is called after each sample operation is completed by the ADC.
\newline
\begin{lstlisting}[language=Arduino, caption=ADC Sample Saving, basicstyle=\scriptsize,]
//Called when sampling is complete 
void adc0_isr() {
    
    //Place sample into buffer0 
    int16_t buffer0 = adc->readSingle();
    
    if (rubbishFlag == 1) {
      if (rubbishBufferIndex < (rubbishArraySize - 1)) {
        rubbishBufferArray[rubbishBufferIndex] = buffer0;
      }
      rubbishBufferIndex = 0;
      rubbishFlag = 0;
    }
    
    if (leftFlag == 1) {
      if (leftBufferIndex == (leftArraySize - 1)) {
        leftBufferArray[leftBufferIndex] = buffer0;
        leftBufferIndex = 0;
      } else {
        leftBufferArray[leftBufferIndex] = buffer0;
        leftBufferIndex++;
      }
    }
}
\end{lstlisting}

The calculated sample is stored in a buffer, and then depending on the flag values set by \textit{muxSwitch()} it is placed into one of 3 arrays. The rubbish buffer is always triggered at the start of each new detector sampling period, and consumes 30 out of 100 samples taken per sampling period. After this the \textit{detectorArray} is populated until full.

Due to time constraints, automatic gain control was not able to be implemented as requested by the design. Instead, manual adjustments are made through the functions below.
\begin{verbatim}
    inaGainMux(inaGain);
    outputGainMux(outputGain);
\end{verbatim}
Controlling the AD9833 frequency synthesizer chip on the lock-in board required on chip registers to be written to. Example code created by the chip manufacturer may be found in Algorithm 12 in the appendix.
