#include "ADC.h"
#include "RingBuffer.h"
#include <IntervalTimer.h>
#include <SPI.h>

//Definitions for the muiltiplexer pins
#define outputMuxA0 2
#define outputMuxA1 3
#define outputMuxEN 23

#define inaMuxA2 5
#define inaMuxA1 6
#define inaMuxA0 7
#define inaMuxEN 21

#define inputMuxA0 8
#define inputMuxA1 9
#define inputMuxEN 20

//Pin for triggering the interupt on the main teensy
#define uartDataReady 15

//Detector data variables
//volatile float normalisedMaxAmplitude = 0;
volatile int16_t maxAmplitude = 0;
int16_t left4diff, right4diff, up4diff, down4diff;
int16_t aziDiff, altDiff;

//For automatically switching input channels
volatile int8_t muxIndex = 1;

//Constants for frequency synth
#define SPI_CLOCK_SPEED 12000000      // 12MHz SPI clock
const int FSYNC = 10;                 // pin 10 (SS)
unsigned long MCLK = 20000000;        // AD9833 onboard crystal reference frequency
unsigned long freq = 250030;          // set initial frequency

//Loop indicator LED
const int ledPin = LED_BUILTIN;

//sampling pin (A8 aka 22), and period between samples
const int adcInput = A8;
const int adcReadPeriod = 500; //us

//Period between writes to the serial port
const int mainLoopPeriod = 50000; //us

//Period between mux channel switching and array swapping
const int switchPeriod = 50000; //u

ADC *adc = new ADC(); // adc object

IntervalTimer timer0; //Timer for ADC reading
IntervalTimer timer1; //Timer for switching channels

int startTimerValue0 = 0, startTimerValue1 = 0;

volatile int diffFlag;

#define leftArraySize 70
#define rightArraySize 70
#define rubbishArraySize 30

volatile int16_t leftFlag = 0;
volatile int16_t leftBufferIndex = 0;
volatile int16_t leftBufferArray[leftArraySize];
volatile int16_t rightFlag = 0;
volatile int16_t rightBufferIndex = 0;
volatile int16_t rightBufferArray[rightArraySize];
volatile int16_t rubbishFlag = 0;
volatile int16_t rubbishBufferIndex = 0;
volatile int16_t rubbishBufferArray[rubbishArraySize];

  //////////////////////////////////////////////
 // READ DATA DROM ADC AND CHECK FOR MAXIMUM //
//////////////////////////////////////////////

void timer0_callback(void) {
  adc->startSingleRead(adcInput, ADC_0);
}

// when the measurement finishes this will be called
void adc0_isr() {

  uint8_t pin = ADC::sc1a2channelADC0[ADC0_SC1A & ADC_SC1A_CHANNELS]; // the bits 0-4 of ADC0_SC1A have the channel

  if (pin == adcInput) {
   
    cli();
    
    int16_t buffer0 = adc->readSingle();
    Serial.println(buffer0);

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
    
    if (rightFlag == 1) {
      if (rightBufferIndex == (rightArraySize - 1)) {
        rightBufferArray[rightBufferIndex] = buffer0;
        rightBufferIndex = 0;
      } else { 
        rightBufferArray[rightBufferIndex] = buffer0;
        rightBufferIndex++;
      }
    }

   
    sei();
      
  } else { // clear interrupt anyway
    adc->readSingle();
  }

  // restore ADC config if it was in use before being interrupted by the analog timer
  if (adc->adc0->adcWasInUse) {
    adc->adc0->loadConfig(&adc->adc0->adc_config);  // restore ADC config, and restart conversion
    adc->adc0->adcWasInUse = false;                 // avoid a conversion started by this isr to repeat itself
  }

}

  ////////////////////////////////////////////////////////////////
 // SWITCH DETECTORS VIA ISR & SAVE RELEVENT VALUES FOR PERIOD //
////////////////////////////////////////////////////////////////

void muxSwitch() {
  
  digitalWriteFast(LED_BUILTIN, !digitalReadFast(LED_BUILTIN) );
  
  cli();

  rubbishFlag = 1;
  
  switch (muxIndex) {

    case 0:
    
      detectorMux(1);

      left4diff = maximum(leftBufferArray);
      
      leftFlag = 0;
      rightFlag = 1;
      
      muxIndex++;
      
      sei();
      
      break;                                        //Return

    case 1:
         
      detectorMux(2);

      right4diff = maximum(rightBufferArray);
      
      aziDiff = left4diff - right4diff;             //Azimuth differencia
      
      rightFlag = 0;
      leftFlag = 1;
      diffFlag = 1;
  
      
      muxIndex = 0;
      
      sei();
      
      break;

    default:
      
      sei();
        
      break;
  }
}

  ///////////
 // SETUP //
///////////

void setup() {

  //Setup for the frequency synth
  pinMode (FSYNC, OUTPUT);
  digitalWrite(FSYNC, HIGH);
  SPI.begin();
  AD9833setFrequency(freq);

  //ADC things that I dont fully understand
  pinMode(ledPin, OUTPUT);        // led blinks every loop
  pinMode(ledPin + 1, OUTPUT);    // timer0 starts a measurement
  pinMode(ledPin + 3, OUTPUT);    // adc0_isr, measurement finished for readPin0

  //Setup pins for the output gain multiplexer
  pinMode(outputMuxA0, OUTPUT);
  pinMode(outputMuxA1, OUTPUT);
  pinMode(outputMuxEN, OUTPUT);

  //Setup pins for the instrumentation gain amplifier
  pinMode(inaMuxA0, OUTPUT);
  pinMode(inaMuxA1, OUTPUT);
  pinMode(inaMuxA2, OUTPUT);
  pinMode(inaMuxEN, OUTPUT);

  //PHPD detector selection
  pinMode(inputMuxA0, OUTPUT);
  pinMode(inputMuxA1, OUTPUT);
  pinMode(inputMuxEN, OUTPUT);

  pinMode(adcInput, INPUT);

  //Start the usb serial port
  Serial.begin(9600);
  Serial1.begin(9600);
  pinMode(uartDataReady, OUTPUT);

  delay(1000);

  ///// ADC0 ////
  // reference can be ADC_REFERENCE::REF_3V3, ADC_REFERENCE::REF_1V2 (not for Teensy LC) or ADC_REFERENCE::REF_EXT.
  //adc->setReference(ADC_REFERENCE::REF_1V2, ADC_0); // change all 3.3 to 1.2 if you change the reference to 1V2

  adc->setAveraging(16); // set number of averages
  adc->setResolution(12); // set bits of resolution

  // it can be any of the ADC_CONVERSION_SPEED enum: VERY_LOW_SPEED, LOW_SPEED, MED_SPEED, HIGH_SPEED_16BITS, HIGH_SPEED or VERY_HIGH_SPEED
  // additionally the conversion speed can also be ADACK_2_4, ADACK_4_0, ADACK_5_2 and ADACK_6_2,
  // where the numbers are the frequency of the ADC clock in MHz and are independent on the bus speed.
  adc->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED); // change the conversion speed
  // it can be any of the ADC_MED_SPEED enum: VERY_LOW_SPEED, LOW_SPEED, MED_SPEED, HIGH_SPEED or VERY_HIGH_SPEED
  adc->setSamplingSpeed(ADC_SAMPLING_SPEED::HIGH_SPEED); // change the sampling speed
  // always call the compare functions after changing the resolution!
  //adc->enableCompare(1.0/3.3*adc->getMaxValue(ADC_0), 0, ADC_0); // measurement will be ready if value < 1.0V
  //adc->enableCompareRange(1.0*adc->getMaxValue(ADC_0)/3.3, 2.0*adc->getMaxValue(ADC_0)/3.3, 0, 1, ADC_0); // ready if value lies out of [1.0,2.0] V
  // If you enable interrupts, notice that the isr will read the result, so that isComplete() will return false (most of the time)
  //adc->enableInterrupts(ADC_0);

  // start the timers, if it's not possible, startTimerValuex will be false
  startTimerValue0 = timer0.begin(timer0_callback, adcReadPeriod);
  // wait enough time for the first timer conversion to finish (depends on resolution and averaging),
  // with 16 averages, 12 bits, and ADC_MED_SPEED in both sampling and conversion speeds it takes about 36 us.
  delayMicroseconds(40); // if we wait less than 36us the timer1 will interrupt the conversion
  // initiated by timer0. The adc_isr will restart the timer0's measurement.

  adc->enableInterrupts(ADC_0);

  startTimerValue1 = timer1.begin(muxSwitch, switchPeriod);

  delay(500);
  
  //Choose a detectorChannel between 1 and 4 inclusive
  //int detectorChannel = 2;
  //detectorMux(detectorChannel);

  //Choose an instrumentation-amp gain
  //inaGain may be equal to: 10000, 3030, 920, 279, 84, 26, 7, 2, 1
  int inaGain = 1;
  inaGainMux(inaGain);

  //Choose an output-amp gain
  //outputGain may be equal to: 909, 3, 1
  float outputGain = 1;
  outputGainMux(outputGain);
  
}

  ///////////////
 // MAIN LOOP //
///////////////

void loop() {


  cli();
  
  if (diffFlag == 1) {
    //Serial.println(aziDiff);
    writeInt(aziDiff);
    diffFlag = 0; 
  }

  sei();
  delay(100);
}

  ////////////////////////////////////////////
 // SET OUTPUT GAIN VIA OUTPUT MULTIPLEXER //
////////////////////////////////////////////

void outputGainMux(int outputGain) {

  switch (outputGain) {

    //Channel 1 - gain of 909
    case 909:
      digitalWrite(outputMuxEN, HIGH);
      digitalWrite(outputMuxA0, LOW);
      digitalWrite(outputMuxA1, LOW);
      break;

    //Channel 2 - gain of 1
    case 1:
      digitalWrite(outputMuxEN, HIGH);
      digitalWrite(outputMuxA0, HIGH);
      digitalWrite(outputMuxA1, LOW);
      break;

    //Channel 3 - gain of 3
    case 3:
      digitalWrite(outputMuxEN, HIGH);
      digitalWrite(outputMuxA0, LOW);
      digitalWrite(outputMuxA1, HIGH);
      break;
  }
}

  ///////////////////////////////////////////////////
 // SET DETECTOR CHANNEL VIA DETECTOR MULTIPLEXER //
///////////////////////////////////////////////////

void detectorMux(int detectorChannel) {

  digitalWrite(inputMuxEN, HIGH);

  switch (detectorChannel) {

    case 1:
      digitalWrite(inputMuxA0, LOW);
      digitalWrite(inputMuxA1, LOW);
      break;

    case 2:
      digitalWrite(inputMuxA0, HIGH);
      digitalWrite(inputMuxA1, LOW);
      break;

    case 3:
      digitalWrite(inputMuxA0, LOW);
      digitalWrite(inputMuxA1, HIGH);
      break;

    case 4:
      digitalWrite(inputMuxA0, HIGH);
      digitalWrite(inputMuxA1, HIGH);
      break;
  }

}

  ////////////////////////////////////////////////////////
 // SET INSTRUMENTATION OPAMP GAIN VIA INA MULTIPLEXER //
////////////////////////////////////////////////////////

void inaGainMux(int inaGain) {

  switch (inaGain) {

    //Channel 1 - gain of 10,000
    case 10000:
      digitalWrite(inaMuxEN, HIGH);
      digitalWrite(inaMuxA0, LOW);
      digitalWrite(inaMuxA1, LOW);
      digitalWrite(inaMuxA2, LOW);
      break;

    //Channel 2 - gain of 3,030
    case 3030:
      digitalWrite(inaMuxEN, HIGH);
      digitalWrite(inaMuxA0, HIGH);
      digitalWrite(inaMuxA1, LOW);
      digitalWrite(inaMuxA2, LOW);
      break;

    //Channel 3 - gain of 920
    case 920:
      digitalWrite(inaMuxEN, HIGH);
      digitalWrite(inaMuxA0, LOW);
      digitalWrite(inaMuxA1, HIGH);
      digitalWrite(inaMuxA2, LOW);
      break;

    //Channel 4 - gain of 279
    case 279:
      digitalWrite(inaMuxEN, HIGH);
      digitalWrite(inaMuxA0, HIGH);
      digitalWrite(inaMuxA1, HIGH);
      digitalWrite(inaMuxA2, LOW);
      break;

    //Channel 5 - gain of 84
    case 84:
      digitalWrite(inaMuxEN, HIGH);
      digitalWrite(inaMuxA0, LOW);
      digitalWrite(inaMuxA1, LOW);
      digitalWrite(inaMuxA2, HIGH);
      break;

    //Channel 6 - gain of 26
    case 26:
      digitalWrite(inaMuxEN, HIGH);
      digitalWrite(inaMuxA0, HIGH);
      digitalWrite(inaMuxA1, LOW);
      digitalWrite(inaMuxA2, HIGH);
      break;

    //Channel 7 - gain of 7
    case 7:
      digitalWrite(inaMuxEN, HIGH);
      digitalWrite(inaMuxA0, LOW);
      digitalWrite(inaMuxA1, HIGH);
      digitalWrite(inaMuxA2, HIGH);
      break;

    //Channel 8 - gain of 2
    case 2:
      digitalWrite(inaMuxEN, HIGH);
      digitalWrite(inaMuxA0, HIGH);
      digitalWrite(inaMuxA1, HIGH);
      digitalWrite(inaMuxA2, HIGH);
      break;

    //Not connected - gain of 1
    case 1:
      digitalWrite(inaMuxEN, LOW);
      break;
  }
}

  ///////////////////////////////////////////////
 // CHANGE FREQUENCIES OF AD9833 SIG GEN CHIP //
///////////////////////////////////////////////

void AD9833setFrequency(long frequency) {
  long FreqReg = (frequency * pow(2, 28)) / MCLK;
  int MSB = (int)((FreqReg & 0xFFFC000) >> 14);    // only lower 14 bits are used for data
  int LSB = (int)(FreqReg & 0x3FFF);

  LSB |= 0x4000;                      // DB 15=0, DB14=1
  MSB |= 0x4000;                      // DB 15=0, DB14=1

  WriteRegister(0x21E8);
  WriteRegister(LSB);                   // write lower 16 bits to AD9833 registers
  WriteRegister(MSB);                   // write upper 16 bits to AD9833 registers
  WriteRegister(0xC000);                // write phase register
  WriteRegister(0x0068);
}

//Write to the frequnecy generators registers
void WriteRegister(int data) {
  SPI.beginTransaction(SPISettings(SPI_CLOCK_SPEED, MSBFIRST, SPI_MODE2));
  digitalWrite(FSYNC, LOW);           // set FSYNC low before writing to AD9833 registers
  SPI.transfer16(data);
  digitalWrite(FSYNC, HIGH);          // write done, set FSYNC high
  SPI.endTransaction();
}

  ///////////////////////////////////////////
 // WRITE INTEGERS TO THE MAIN TEENSY 3.6 //
///////////////////////////////////////////

void writeInt(int number) {
  digitalWriteFast(uartDataReady, HIGH);              //Trigger the interupt on the 3.6 to start it collecting data
  Serial1.println(number);                            //Send the number
  //Uncomment to see the data sent on the monitor
  //Serial1.print("\n");
  //Serial.println(number);
    //delay(100);
  digitalWriteFast(uartDataReady, LOW);               //Reset the pin whihc triggers the interupt
  return;
}

volatile int16_t maximum(volatile int16_t array[]){    //finds the max value of the last 16 data inputs
    volatile int16_t maxA = array[0];   
    for (int x = 0; x<69; x++){    //cycles through the array
        if (array[x] != 0){   //filters out zero values 
            if (array[x]>maxA){    //if the value in the array is more than the current max then replace it
                maxA = array[x];   //max = that element in the array
            }
        }
    }    
    return maxA;
}

volatile int16_t  minimum(volatile int16_t array[]){    //finds the minimum value of the last 16 data inputs
    volatile int16_t minA = array[0];
    for (int x = 0; x<69; x++){
        if (array[x] != 0){   //filters 0
            if (array[x]<minA){    
                minA = array[x];   //if the value in the array is less than the current min then replace it
            }
        }
    }    
    return minA;
}
