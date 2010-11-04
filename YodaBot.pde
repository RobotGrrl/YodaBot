/* Yoda Bot Rampage Robot
 * -----------------------
 * Drives around like a crazy robot!
 *
 * Tries to see if there is an object infront of it with
 * a panning sonar sensor.
 *
 * Reads light levels from the TSL235R, tries to determine
 * what colour it is. Colour of the pirana LED (the force!)
 * will mimic what the prediction is. Driving commands could
 * be given, too. Determining the colour will be done one 
 * day with a BFS. Sometime soon. Maybe. :P
 *
 * Code is licenced under CC BY-NC-SA
 * robotgrrl.com
 *
 *
 * v 0.3 - Wednesday November 3rd, 2010
 * ------------------------------------
 * Added some panning code for the sonar sensor to see if
 * there is anything in front. Also added in the correct
 * pins for the servo, which is controlled by the servo
 * library, the bumper, and the sonar itself. There are
 * still a few pins that are mapped over themselves that
 * need to be fixed, though.
 *
 * v 0.2 - Saturday September 11th, 2010
 * -------------------------------------
 * Made a bunch of the code into sepperate functions. Made the
 * RGB LED turn on with different settings for data analysis 
 * with different colours of paper underneath the sensor.
 *
 *
 * v 0.1 - Friday August 27th, 2010
 * --------------------------------
 * Established communication with the TSL235R sensor, received
 * reliable results and fixed up the getUwattCm2 function. Not
 * sure if it's the actual correct math, though.
 *
 */

// Original comment from borrowed code:
/* reads TLS230R module
 * uses interrupt for counting pulses
 * loop() periodically takes snapshots & sends out
 * either as ascii or binary for graphing real-time over serial
 * smooths out spikes inherenet in signal
 * lights an LED for the sensor to pick up
 * author: Oscar Carrillo. Adapted from others code
 * Please see here:
 * http://home.teampaulc.org/arduino/tsl230r-to-arduino-interface
 * and here:
 * http://roamingdrone.wordpress.com/2008/11/13/arduino-and-the-taos-tsl230r-light-sensor-getting-started/
 * If put in binary mode, you can see it graphed in Real-Time with this using Processing:
 * Realtime Graphing of Accelerometer/Gyroscope Data
 * http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1246274143
 * portions of digitalSmoothing used:
 * http://www.arduino.cc/playground/Main/DigitalSmooth
 */


#include <Streaming.h>
#include <Encoder.h>
#include <Motor.h>
#include <Servo.h>

Motor left(5, 6);
Motor right(9, 10);

Servo servo1;

int force_g = 7;
int force_b = 4;
int force_r = 3;

int rgb_g = 13;
int rgb_r = 12;
int rgb_b = 11;

int sonar(4);
int servo(3);
int scoop(5);

// setup the TLS230R to Arduion Mega mapping
#define TSL_FREQ_PIN 21 // output use digital pin2 for interrupt
#define TSL_FREQ_INT 2 //pin 18=int5, 21=int2, 2=int0
#define READ_TM 2 // milliseconds between pulse count tabulation on the interrupt
#define NUM_BUFFER 5 // number of samples in the buffer for smoothing function
#define PERCENTILE 15 //cut off for throwing out outliers: bottom/top percentile

int calcSensitivity; //only for sensor with sensitivity setting (230R)
unsigned long pulseCount = 0; // pulseCount is always in use
unsigned long currentTime = millis();
unsigned long startTime = currentTime;
//I think I need to rethink this hex value for something more appropriate
unsigned int startTag = 0xDEAD;  // Analog port maxes at 1023 so this is a safe termination value
int holder=0; //a dummy value for sending over serial ports on unused signals
unsigned int bufferCounter = 0;
volatile unsigned int buffer[NUM_BUFFER];
unsigned int bufferSnapshot[NUM_BUFFER];
unsigned int nSmoothSample = 0;

// freq will be modified by the interrupt handler so needs to be volatile
// freq holds the latest frequency calculation
unsigned long frequency;
float uWattCm2;
volatile unsigned int curPulseCount;
volatile unsigned int windowCount;
unsigned int windowCountSnapshot;
volatile unsigned int timeElapsed;
volatile unsigned int timeElapsedSample;
unsigned int timeElapsedSnapshot;
unsigned int count=0, i=0;
unsigned int scale;   // holds the TSL scale value, see below
int ledLevel = 0;
float area = 0.0092; // Sensing area of TSL25R device, in cm2

// Just some variables for poking around ^^
int currentReading = 0;
float averageReading = 0.0;
int iterations = 0;
int loopCount = 0;
boolean doTest = true;

int loops;

float sonarReadings[91];

bool rightPan = true;
int currentPos = 90;
bool foundObject = false;

void setup() {
	
	Serial.begin(9600);
	Serial.println("~ Rampage Robot ~");
	
	pinMode(force_r, OUTPUT);
	pinMode(force_g, OUTPUT);
	pinMode(force_b, OUTPUT);
	
	pinMode(rgb_r, OUTPUT);
	pinMode(rgb_g, OUTPUT);
	pinMode(rgb_b, OUTPUT);
	
	pinMode(sonar, INPUT);
	servo1.attach(servo);
	
	pinMode(scoop, INPUT);
		
	startTSL();
	nSmoothSample = getSmoothedSample();
	currentReading = getUwattCm2();
	
	loops = 0;
	
	servo1.write(90);
	
	for(int j=0; j<5; j++) {
	
		for(int i=90; i<180; i++) {
			servo1.write(i);
			sonarReadings[i-90] += _sonarVal();
			delay(10);
		}
	
		for(int i=180; i>90; i--) {
			servo1.write(i);
			delay(10);
		}
	
	}
	
	for(int i=0; i<90; i++) {
		Serial << i << ": " << sonarReadings[i] << endl;
		sonarReadings[i] /= 5;
		Serial << i << ": " << sonarReadings[i] << endl;
	}
	
}

int thresh = 5;

void loop() {
	
	float curSonar = _sonarVal();
	
	while(curSonar < (sonarReadings[currentPos-90] - thresh) || curSonar > (sonarReadings[currentPos-90] + thresh) || foundObject == true) {
		
		foundObject = false;
		
		if(currentPos == 90) {
			rightPan = true;
		} else if(currentPos == 179) {
			rightPan = false;
		}
		
		if(rightPan) {
			currentPos++;
			servo1.write(currentPos);
			curSonar = _sonarVal();
		} else {
			for(int i=180; i>91; i--) {
				currentPos--;
				servo1.write(currentPos);
				delay(10);
			}
		}
		
		delay(10);
		
	}
	
	if(curSonar >= (sonarReadings[currentPos-90] + thresh)) {
		
		digitalWrite(force_b, HIGH);
		Serial << "Current Sonar: " << curSonar << " Sonar avg: " << sonarReadings[currentPos-90] << " Angle: " << currentPos << endl;
		foundObject = true;
		delay(1000);
		
	}
	
	
	//drive();
	
}

float _sonarVal() {
	float s = analogRead(sonar);
	bool print = false;
	if(print) Serial << "Sonar: " << s << endl;
	return s;
}

void sonarStuff() {

	Serial << "Sonar: " << analogRead(sonar) << endl;
	
	delay(100);
	
	for(int i=90; i<180; i++) {
		servo1.write(i);
		Serial << "Angle: " << i << endl;
		Serial << "Sonar: " << analogRead(sonar) << endl;
		delay(10);
	}
	
	for(int i=180; i>90; i--) {
		servo1.write(i);
		Serial << "Angle: " << i << endl;
		Serial << "Sonar: " << analogRead(sonar) << endl;
		delay(10);
	}
	
}

void test() {

	if(doTest) {
		
		// Each test will consist of 100 iterations
		if(iterations % 100 == 0) {
			loopCount++;
			iterations = 0;
		}
		
		// Depending on which 100 we are on, we need to trigger
		// the light to be different each time.
		if(loopCount == 1 && iterations == 0) {
			rgbLED(true, true, true);
			Serial << "White light" << endl;
		} else if(loopCount == 2 && iterations == 0) {
			rgbLED(true, false, false);
			Serial << "Red light" << endl;
		} else if(loopCount == 3 && iterations == 0) {
			rgbLED(false, true, false);
			Serial << "Green light" << endl;
		} else if(loopCount == 4 && iterations == 0) {
			rgbLED(false, false, true);
			Serial << "Blue light" << endl;
		} else if(loopCount == 5 && iterations == 0) {
			rgbLED(false, false, false);
			Serial << "No light" << endl;
		} else if(loopCount == 6 && iterations == 0) {
			Serial << "End of test" << endl;
			doTest = false;
		}
		
		// Have to ask again since it might be changed previous.
		// Not the best way of coding to solve this, but we're 
		// only interested in the results :P
		if(doTest) {
			
			nSmoothSample = getSmoothedSample();
			currentReading = getUwattCm2();
			Serial << currentReading << endl;
			
			delay((READ_TM * NUM_BUFFER));
			iterations++;
			
		}
		
	}
	
}

void startTSL() {
	
	//initialize arrays
	for (i=0;i<NUM_BUFFER;i++) {
		buffer[i] = 0;
		bufferSnapshot[i] = 0;
	}
	
	Serial.println("Start...");
	
	// attach interrupt to pin2, send output pin of TSL235R to arduino 2
	// call handler on each rising pulse
	//something weird may be going on with interrupts with Arduino Mega
	
	//pin 2, doesn't work on Mega, dunno why
	// attachInterrupt(0, add_pulse, RISING);
	attachInterrupt(0, addPulse, RISING); //interrupt 2, matches pin 21
	pinMode(TSL_FREQ_PIN, INPUT);
	scale = 1; // set this to match TSL_S2 and TSL_S3 for 230R, not needed for 235R
	
}

void printTSL() {
	
	nSmoothSample = getSmoothedSample(); //Get smoothed out sample every go round
	
	// this is just for debugging
	// it shows that you can sample freq whenever you need it
	// even though it is calculated 100x a second
	// note that the first time that freq is calculated, it is bogus
	Serial.print("winCount: ");
	Serial.print(windowCount);
	Serial.print(" count: " );
	Serial.print(count);
	Serial.print(" smooth: ");
	Serial.print(nSmoothSample);
	Serial.print(" freq: ");
	Serial.print(getFrequency(nSmoothSample), DEC);
	Serial.print("\tuW/cm: ");
	Serial.print(getUwattCm2(), DEC);
	count++;
	
	delay((READ_TM * NUM_BUFFER)); //delay for msec btwn tabulations * (# of running samples) + 1 to make sure we don't get a repeat
	//delay(500);
	
}

void stopTSL() {
	
	detachInterrupt(0);
	
}

void addPulse() {
	
	// DON'T calculate anything or smooth the data every READ_TM ms
	// just store the pulse count to be used outside of the interrupt
	pulseCount++; // increase pulse count
	currentTime = millis();
	timeElapsed = currentTime - startTime;
	
	//Tabulate pulses if the time elapsed surpasses READ_TM
	if(timeElapsed >= READ_TM) {
		curPulseCount = pulseCount;  // use curPulseCount for calculating freq/uW
		buffer[bufferCounter] = curPulseCount;
		
		//increment and roll over if necessary
		if (bufferCounter == (NUM_BUFFER-1)) {
			bufferCounter = 0; // roll over
			windowCount++; //keep track of windows computed
			timeElapsedSample = timeElapsed; //could poll this if you want to see how close this is to READ_TM
		} else {
			bufferCounter++; //increment
		}
		
		//reset
		pulseCount = 0;
		startTime = millis();
	}
	
}

/**
 * This gets the frequency (counts/sec)
 * getFrequency(unsigned int)
 */
double getFrequency(unsigned int sample) {
	return (sample*(1000.0/timeElapsedSnapshot));
}

/**
 * This returns the irradiance of light based on things
 * known about the 230R sensor. Could be adapted to 235R possibly.
 */
long getUwattCm2() {
	
	// copy pulse counter and multiply.
	// the multiplication is necessary for the current
	// frequency scaling level.
	frequency = curPulseCount * scale;
	
	// get uW observed - assume 640nm wavelength
	// calc_sensitivity is our divide-by to map to a given signal strength
	// for a given sensitivity (each level of greater sensitivity reduces the signal
	// (uW) by a factor of 10)
	// float uw_cm2 = (float) frequency / (float) calcSensitivity;
	
	// extrapolate into entire cm2 area
	// uWattCm2  = uw_cm2  * ( (float) 1 / (float) 0.0136 );
	
	// return(uWattCm2);
	
	float irradiance;
	irradiance = frequency / area;		    // Calculate Irradiance (uW/cm2)
	return (irradiance);
	
}

/**
 * Gets rid of spikes in buffer and returns a smoothed
 * value. It also takes a snapshot of the timeElapsed
 * for this particular snapshot of the buffer
 * so it can be used elsewhere
 */
unsigned int getSmoothedSample() {
	
	static unsigned int sorted[NUM_BUFFER];
	unsigned int intAvg=0, mod=0;
	unsigned int j=0, nValid=0, temp=0, top=0, bottom=0, total=0;    
	boolean done;
	
	//This is probably overkill: copy, copy, sort
	//duplicate samples in the buffer at this point in time
	for (i=0;i<NUM_BUFFER;i++) {
		bufferSnapshot[i] = buffer[i];
	}
	
	//copy the value for use within loop
	timeElapsedSnapshot = timeElapsedSample;
	windowCountSnapshot = windowCount;
	
	//copy the data into yet another array for sorting
	for (i=0;i<NUM_BUFFER;i++) {
		sorted[i] = bufferSnapshot[i];
	}
	
	// simple swap sort, sorts numbers from lowest to highest
	for (done=0;done!=1;) {
		done = 1;
		for (j = 0; j < (NUM_BUFFER - 1); j++){
			if (sorted[j] > sorted[j + 1]){     // numbers are out of order - swap
				temp = sorted[j + 1];
				sorted [j+1] =  sorted[j] ;
				sorted [j] = temp;
				done = 0;
			}
		}
	}
	
	// throw out top and bottom PERCENTILE of samples - limit to throw out at least one from top and bottom
	bottom = max(((NUM_BUFFER * PERCENTILE)  / 100), 1);
	top = min((((NUM_BUFFER * (100-PERCENTILE)) / 100) + 1  ), (NUM_BUFFER - 1));   // the + 1 is to make up for asymmetry caused by integer rounding
	
	for (j=bottom; j<top; j++){
		total += sorted[j];  // total remaining indices
		nValid++; // # of good values
	}
	
	intAvg = total/nValid;
	mod = total%nValid;
	
	//this is simulating floating point math, a bit more accurate than just rounding
	//takes the divisor, multiples the modulus by # of samples and divides it by how
	//many samples it is based on
	return ((intAvg * NUM_BUFFER) + ((mod * NUM_BUFFER)/(nValid)));
}

void rgbLED(boolean r, boolean g, boolean b) {
	
	digitalWrite(rgb_r, r);
	digitalWrite(rgb_g, g);
	digitalWrite(rgb_b, b);
	
}

void forceLED(boolean r, boolean g, boolean b) {
	
	digitalWrite(force_r, r);
	digitalWrite(force_g, g);
	digitalWrite(force_b, b); 
	
}

// This is an old function for controlling the wheels
void drive() {
	
	for(int i=0; i<256; i++) {
		
		left.rate(i);
		right.rate(i);
		
		left.drive(true);
		right.drive(true);
		
		delay(10);
		
	}
	
	for(int i=255; i>0; i--) {
		
		left.rate(i);
		right.rate(i);
		
		left.drive(true);
		right.drive(true);
		
		delay(10);
		
	}
	
	left.stop();
	right.stop();
	
	delay(500);
	
	for(int i=0; i<256; i++) {
		
		left.rate(i);
		right.rate(i);
		
		left.drive(false);
		right.drive(false);
		
		delay(10);
		
	}
	
	for(int i=255; i>0; i--) {
		
		left.rate(i);
		right.rate(i);
		
		left.drive(false);
		right.drive(false);
		
		delay(10);
		
	}
	
	left.stop();
	right.stop();
	
	delay(500);
	
}


// ------ Older notes
// Trying to figure out what pins the encoders are on
// 17 = left encoder, 16 = right encoder (doesn't really work)
// 13 = top right pot, bottom right pot = ?
// 14 = top left pot, 15 = bottom left pot
// Serial << " 12: " << analogRead(12) << " 13: " << analogRead(13) << " 14: " << analogRead(14) << " 15: " << analogRead(15) << " 16: " << analogRead(16) << " 17: " << analogRead(17) << endl;
// delay(100);

