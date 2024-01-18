/*
This is a multi-FX box hardware designed by Keyi Ding in C++ using the Bela platform. 
*/

#include <Bela.h>
#include <libraries/Fft/Fft.h>
#include <libraries/Scope/Scope.h>
#include <libraries/Gui/Gui.h>
#include <libraries/GuiController/GuiController.h>
#include <libraries/Oscillator/Oscillator.h>
#include <libraries/Biquad/Biquad.h>
#include <libraries/OnePole/OnePole.h>
#include <libraries/Trill/Trill.h>
#include <cmath>
#include <cstring>
#include <vector>
#include <algorithm>
#include <libraries/OscSender/OscSender.h>
#include <oscpkt.hh>
#include "decimator.h"
#include "Bela_BNO055.h"

//LED 
#define NUM_LED 8 
// Location of touches on Trill RING
float gTouchLocation= 0.0;
// Size of touches on Trill bar
float gTouchSize = 0.0;;
// Number of active touches
int gNumActiveTouches = 0;
// Digital pins assigned to LEDs used for visualisation
unsigned int gLedPins[NUM_LED] = { 0, 1, 2, 3, 4, 5, 8, 9};
// Status of LEDs (1: ON, 0: OFF)
bool gLedStatus[NUM_LED] = { 0, 0, 0, 0, 0, 0, 0, 0};


//Biquad Low-Pass Filter object
Biquad lpFilter;
OnePole analogSmooth;

// FFT-related variables
Fft gFft; 
//Fft gFftMag;
// Fft gFftPhi;
// Fft gFftRect;// FFT processing object
const int gFftSize = 1024;	// FFT window size in samples
int gHopSize = 512 ;	// How often we calculate a window
int gHopSizeRange[2] = {128, 512}; 
int hopSize = 512;
// Spectual Gating 
float gDenoiseThreshold = 0;

//Crosssynth effect dry/wet mix factor
float gDryWetMix =0;

// Circular buffer and pointer for assembling a window of samples
const int gBufferSize = 16384;
std::vector<float> gInputBufferMag;
std::vector<float> gInputBufferPhi;
int gInputBufferPointer = 0;
int gHopCounter = 0;

// Circular buffer for collecting the output of the overlap-add process
std::vector<float> gOutputBuffer;
int gOutputBufferWritePointer = gFftSize + gHopSize;
int gOutputBufferReadPointer = 0;
float gDelayFeedback = 0.1;

//Filter parameters 
float gLPfreq;	// Cut-off frequency for low-pass filter (Hz)

float gFilterQ = 0.707; // Quality factor for the biquad filters to provide a Butterworth response

// Buffer to hold the windows for FFT analysis and synthesis
std::vector<float> gAnalysisWindowBuffer;
std::vector<float> gSynthesisWindowBuffer;

// Name of the sound file (in project folder)
std::string gFilenameMag = "zFox.wav"; 
std::string gFilenamePhi = "longsample.wav"; 

// Thread for FFT processing
AuxiliaryTask gFftTask;
int gCachedInputBufferPointer = 0;

void process_fft_background(void *);

// Create Trill Object
std::vector<Trill*> gTouchSensors;
Trill touchSensor;

// Buffer for storing mag and phase information 
std::vector<float> magInput(gFftSize);
std::vector<float> phasInput(gFftSize);
std::vector<float> outPutMag(gFftSize);
std::vector<float> outPutPhi(gFftSize);

// Lfo object for flanger 
Oscillator lfo; 

// Circular buffer and readpointer
std::vector<float> gDelayBuffer;
unsigned int gWritePointer = 0;
unsigned int gReadPointer = 0;
unsigned int gOffset = 0;
float gDelayTime = 0;
float gDelayRange[2] = {0.0, 0.5}; 
float lfoFreq = 0.0; 
float lfoAmp = 0.0; 
float lfoAmpRange[2] = {0.0, 10.0}; 

//Create Decimator object 
Decimator decimator;

//OSC object 
OscSender oscSender;

// Function for reading values on  Trill sensors
void readLoop(void*)
{
	while(!Bela_stopRequested())
	{
		for(unsigned int n = 0; n < gTouchSensors.size(); ++n)
		{
			Trill* t = gTouchSensors[n];
			t->readI2C();
			std::string addr = "/trill/readings/" + std::to_string(n) + "/touches";
			// Read locations from Trill sensor
			oscpkt::Message msg(addr);
			msg.pushFloat(t->getNumTouches());
			for(unsigned int i = 0; i < t->getNumTouches(); i++) {
				msg.pushFloat(t->touchLocation(i));
				msg.pushFloat(t->touchSize(i));
			}
			oscSender.sendNonRt(msg);
		}
		usleep(10000);
	}
}

//Bno-055 Accelerameter
int readInterval = 100;

I2C_BNO055 bno; // IMU sensor object
int buttonPin = 14; // calibration button pin
int lastButtonValue = 0; // using a pulldown resistor

// Quaternions and Vectors
imu::Quaternion gCal, gCalLeft, gCalRight, gIdleConj = {1, 0, 0, 0};
imu::Quaternion qGravIdle, qGravCal, quat, steering, qRaw;

imu::Vector<3> gRaw;         
imu::Vector<3> gGravIdle, gGravCal;
imu::Vector<3> ypr; //yaw pitch and roll angles

// // variables for synth
float gInverseSampleRate; 

int calibrationState = 0; // state machine variable for calibration
int setForward = 0; // flag for setting forward orientation

// variables handling threading
AuxiliaryTask i2cTask;		// Auxiliary task to read I2C
AuxiliaryTask gravityNeutralTask;		// Auxiliary task to read gravity from I2C
AuxiliaryTask gravityDownTask;		// Auxiliary task to read gravity from I2C

int readCount = 0;			// How long until we read again...
int readIntervalSamples = 0; // How many samples between reads

int printThrottle = 0; // used to limit printing frequency

// function declarations
void readIMU(void*);
void getNeutralGravity(void*);
void getDownGravity(void*);
void calibrate();
void resetOrientation();

//Analog inputs
int gAudioFramesPerAnalogFrame = 0;
//Sample amplitude 
float gAmp = 0.0; 

bool setup(BelaContext *context, void *userData)
{
	 // Set and schedule auxiliary task for reading sensor data from the I2C bus
	// Bela_runAuxiliaryTask(loop);
	unsigned int i2cBus = 1;
	for(uint8_t addr = 0x20; addr <= 0x50; ++addr)
	{
		Trill::Device device = Trill::probe(i2cBus, addr);
		if(Trill::NONE != device && Trill::CRAFT != device)
		{
			gTouchSensors.push_back(new Trill(i2cBus, device, addr));
			gTouchSensors.back()->printDetails();
		}
	}
	oscSender.setup(5678, "192.168.7.1");
	Bela_runAuxiliaryTask(readLoop);
	

	// Set up the FFT and its buffers
	gFft.setup(gFftSize);
	gInputBufferMag.resize(gBufferSize);
	gInputBufferPhi.resize(gBufferSize);
	gOutputBuffer.resize(gBufferSize);
	
	//pre-compute Hann window for Analysis 
	gAnalysisWindowBuffer.resize(gFftSize);
	for(int n = 0; n < gFftSize; n++) {
		// Hann window
		gAnalysisWindowBuffer[n] = 0.5f * (1.0f - cosf(2.0 * M_PI * n / (float)(gFftSize - 1)));
	}
	//pre-compute Hann window for synthesis 
	gSynthesisWindowBuffer.resize(gFftSize);
	for(int n = 0; n < gFftSize; n++) {
		// Hann window
		gSynthesisWindowBuffer[n] = 0.5f * (1.0f - cosf(2.0 * M_PI * n / (float)(gFftSize - 1)));
	}
	// Set up the thread for the FFT
	gFftTask = Bela_createAuxiliaryTask(process_fft_background, 50, "bela-process-fft");

	//Allocate circular buffer 
    gDelayBuffer.resize(0.5 * context->audioSampleRate);
    
    //Oscillator for flanger 
	lfo.setup(context->audioSampleRate, Oscillator::sine);

	//Decimator initialization
	decimator.Init();

	//Bno-055 set up
	if(!bno.begin()) {
		rt_printf("Error initialising BNO055\n");
		return false;
	}
	
	rt_printf("Initialised BNO055\n");
	
	// use external crystal for better accuracy
  	bno.setExtCrystalUse(true);
  	
	// get the system status of the sensor to make sure everything is ok
	uint8_t sysStatus, selfTest, sysError;
  	bno.getSystemStatus(&sysStatus, &selfTest, &sysError);
	rt_printf("System Status: %d (0 is Idle)   Self Test: %d (15 is all good)   System Error: %d (0 is no error)\n", sysStatus, selfTest, sysError);

	
	// set sensor reading in a separate thread
	// so it doesn't interfere with the audio processing
	i2cTask = Bela_createAuxiliaryTask(&readIMU, 5, "bela-bno");
	readIntervalSamples = context->audioSampleRate / readInterval;
	
	gravityNeutralTask = Bela_createAuxiliaryTask(&getNeutralGravity, 5, "bela-neu-gravity");
	gravityDownTask = Bela_createAuxiliaryTask(&getDownGravity, 5, "bela-down-gravity");
	
	// set up button pin
	pinMode(context, 0, buttonPin, INPUT); 
	
	// Set all digital pins corresponding to LEDs as outputs
	for(unsigned int l = 0; l < NUM_LED; l++)
		pinMode(context, 0, gLedPins[l], OUTPUT);
		
	// Check if analog channels are enabled
	if(context->analogFrames == 0 || context->analogFrames > context->audioFrames) {
		rt_printf("Error: this example needs analog enabled, with 4 or 8 channels\n");
		return false;
	}
	gAudioFramesPerAnalogFrame = context->audioFrames / context->analogFrames;
	analogSmooth.setup(50, context->analogSampleRate);
	
	return true;
}

// This function handles the FFT processing in this example once the buffer has
// been assembled.

//Cross Synthesis 
void crosspec(std::vector<float>& magInput, std::vector<float>& phasInput, std::vector<float>& outPutMag,std::vector<float>& outPutPhi, int fftSize){
	
	int i;
	float mag, phi;
	magInput.resize(fftSize);
	phasInput.resize(fftSize);
	outPutMag.resize(fftSize);
	outPutPhi.resize(fftSize);
	//take care of real-valued points at 0hz and Nyquist
	outPutMag[0] = magInput[0];
	outPutMag[1] = magInput[1];
	
	for(i = 2; i < fftSize; i++){
		//get the magnitudes if one input
		mag = (float) sqrt(magInput[i] * magInput[i] + magInput[i+1]* magInput[i+1]);
		//get the phase of the other
		phi = (float) atan2(phasInput[i+1], phasInput[i]);
		//combine them and convert to rectangular form
		//outPutMag[i] = (float)(mag*cos(phi));
		if(mag>=gDenoiseThreshold){
			outPutMag[i] = (float)(mag*cos(phi));
			outPutPhi[i] = (float)(mag*sin(phi));
		}
		else{
			outPutMag[i] = 0;
			outPutPhi[i] = 0;
		}
	}
}

void process_fft(std::vector<float> const& inBufferMag,std::vector<float> const& inBufferPhi, unsigned int inPointer, std::vector<float>& outBuffer, unsigned int outPointer)
{
	static std::vector<float> unwrappedBufferMag(gFftSize);
	static std::vector<float> unwrappedBufferPhi(gFftSize);// Container to hold the unwrapped values
	
	// Copy buffer into FFT input 
	for(int n = 0; n < gFftSize; n++) {
		// Use modulo arithmetic to calculate the circular buffer index
		int circularBufferIndex = (inPointer + n - gFftSize + gBufferSize) % gBufferSize;
		unwrappedBufferMag[n] = inBufferMag[circularBufferIndex] * gAnalysisWindowBuffer[n];
		unwrappedBufferPhi[n] = inBufferPhi[circularBufferIndex] * gAnalysisWindowBuffer[n];
		
	}
	
	// Process the FFT based on the time domain input
	gFft.fft(unwrappedBufferMag);
	
	for(int i = 0; i< gFftSize; i++){
		magInput[i] = gFft.fdr(i);
	}
	gFft.fft(unwrappedBufferPhi);
	
	for(int i = 0; i< gFftSize; i++){
		phasInput[i] = gFft.fdi(i);
	}
	
	// Call the cross function
	crosspec(magInput, phasInput, outPutMag,outPutPhi, gFftSize);
	
	// Do Inverse FFT 
	gFft.ifft(outPutMag, outPutPhi);

	// Add timeDomainOut into the output buffer
	for(int n = 0; n < gFftSize; n++) {
		int circularBufferIndex = (outPointer + n - gFftSize + gBufferSize) % gBufferSize;
		//outBuffer[circularBufferIndex] += (gFft.td(n)* gDryWetMix + (1.0 - gDryWetMix) * unwrappedBufferMag[n] ) * gSynthesisWindowBuffer[n];
		outBuffer[circularBufferIndex] += gFft.td(n) * gSynthesisWindowBuffer[n];
	}
}

// This function runs in an auxiliary task on Bela, calling process_fft
void process_fft_background(void *)
{
	process_fft(gInputBufferMag,gInputBufferPhi,  gCachedInputBufferPointer, gOutputBuffer, gOutputBufferWritePointer);

	// Update the output buffer write pointer to start at the next hop
	gOutputBufferWritePointer = (gOutputBufferWritePointer + gHopSize) % gBufferSize;
}

void recalculate_window(unsigned int length){
	if(length > gAnalysisWindowBuffer.size()){
		length = gAnalysisWindowBuffer.size();
	}
	if(length > gSynthesisWindowBuffer.size()){
		length = gSynthesisWindowBuffer.size();
	}
	for(int n =0; n < length; n++){
		gAnalysisWindowBuffer[n] = 0.5f * (1.0f - cosf(2.0 * M_PI * n / (float)(length - 1)));
		gSynthesisWindowBuffer[n] = gAnalysisWindowBuffer[n]; 
	}
}

void render(BelaContext *context, void *userData)
{	
	// Active sections of the Trill RING 
	bool activeSections[NUM_LED] = { false };
	//Read trill Bar 
	float trillBarTouchLocation = gTouchSensors[4]->touchLocation(0);
	float gLastRead = gDelayTime; 
	int section = floor( NUM_LED * map(gLastRead, gDelayRange[0], gDelayRange[1], 0.0, 1.0));
	gDelayTime = map(gTouchSensors[1]->touchLocation(0), 0, 1, gDelayRange[0], gDelayRange[1]);
	//if hope size changes, recalculate the window to an overlap factor of 4 
	float LastDenoiseRead = gDenoiseThreshold;
	gDenoiseThreshold = map(gTouchSensors[0]->touchHorizontalLocation(0), 0.0, 1.0, 0.0, 30.0);
	if(gTouchSensors[0]->compoundTouchSize() == 0){
		gDenoiseThreshold = LastDenoiseRead;
	
	}
	int LastHopSizeRead = hopSize;
	hopSize = map(gTouchSensors[0]->touchLocation(0), 0.0, 1.0, int(gHopSizeRange[0]), int(gHopSizeRange[1]));
	if(gTouchSensors[0]->compoundTouchSize() == 0){
		hopSize = LastHopSizeRead;
	}
	if(hopSize != gHopSize){
		int newLength = hopSize*4;
		if(newLength > gFftSize){
			newLength = gFftSize;
		} 
		recalculate_window(newLength);
		
		gHopSize = hopSize;
	}
	
	for(unsigned int n = 0; n < context->audioFrames; n++) {
        
        float analogIn = 0;
		float lpQ;
        // Read the next sample from the buffer
		if(gAudioFramesPerAnalogFrame && !(n % gAudioFramesPerAnalogFrame)) {
				gAmp = map(analogRead(context, n/gAudioFramesPerAnalogFrame, 0), 0.0, 1.0, 0.0, 5.0);
				//gDryWetMix = analogRead(context, n/gAudioFramesPerAnalogFrame, 1);
			   gDelayFeedback = map(analogRead(context, n/gAudioFramesPerAnalogFrame, 1), 0.0, 1.0, 0.0, 0.9);
			  
		}
		for(unsigned int n = 0; n < context->analogFrames; ++n){
			  analogIn = analogSmooth.process(analogRead(context, n, 2));
			  lpQ = analogSmooth.process(analogRead(context, n, 3));
	    }
		float cutoff = powf(2, analogIn * 13) + 20;
		float q = 0.2 + lpQ * 0.6;
		lpFilter.setup({
			.fs = context->audioSampleRate,
			.type = BiquadCoeff::lowpass,
			.cutoff = cutoff,
			.q = q,
			.peakGainDb = 1,
		});
		
		
		for(int i = 0;i <= NUM_LED; i++){
			if (i<= section){
				activeSections[i] = 1;
			}
			else{
				activeSections[i] = 0;
			}
				
		}
		for(unsigned int l = 0; l < NUM_LED; l++) {
			gLedStatus[l] = activeSections[l];
			digitalWrite(context, n, gLedPins[l], gLedStatus[l]);
		}
		
        float inPhi = audioRead(context, n, 1)* gAmp;
        float inMag = audioRead(context, n, 0)* gAmp;
		// Store the sample ("in") in a buffer for the FFT
		// Increment the pointer and when full window has been 
		// assembled, call process_fft()
		gInputBufferMag[gInputBufferPointer] = inMag;
		gInputBufferPhi[gInputBufferPointer] = inPhi;
		if(++gInputBufferPointer >= gBufferSize) {
			// Wrap the circular buffer
			// Notice: this is not the condition for starting a new FFT
			gInputBufferPointer = 0;
		}
	
		// Get the output sample from the output buffer
		float out = gOutputBuffer[gOutputBufferReadPointer];
		
		// Then clear the output sample in the buffer so it is ready for the next overlap-add
		gOutputBuffer[gOutputBufferReadPointer] = 0;
		
		// Scale the output down by the overlap factor 
		out *= (float)gHopSize / (float)gFftSize;
		
		// Delay 
		if(gTouchSensors[1]->touchSize(0) == 0){
    		gDelayTime = gLastRead;
    	}
    	int delayInSamples = int(gDelayTime * context->audioSampleRate) + lfo.process(lfoFreq) * lfoAmp; 
    	gReadPointer = (gWritePointer - delayInSamples + gDelayBuffer.size()) % gDelayBuffer.size();
    	
    	int indexBelow = floorf(gReadPointer);
    	int indexAbove = indexBelow +1;
    	if (indexAbove >= gDelayBuffer.size()){
    		indexAbove = 0;
    	}
    	
    	float fractionAbove = gReadPointer - indexBelow;
    	float fractionBelow = 1.0 - fractionAbove;

    	//Circular Buffer 
    	float outDelay = fractionBelow *gDelayBuffer[indexBelow] + fractionAbove * gDelayBuffer[indexAbove];
    	gDelayBuffer[gWritePointer] = out + outDelay *gDelayFeedback; 
    	
    	// Incremetn and wrap both pointers
    	gWritePointer++;
    	if(gWritePointer >= gDelayBuffer.size()) 
    		gWritePointer = 0;
    	gReadPointer++;
    	if(gReadPointer >= gDelayBuffer.size())
    		gReadPointer = 0;
		// Increment the read pointer in the output cicular buffer
		gOutputBufferReadPointer++;
		if(gOutputBufferReadPointer >= gBufferSize)
			gOutputBufferReadPointer = 0;
		
		// Increment the hop counter and start a new FFT if we've reached the hop size
		if(++gHopCounter >= gHopSize) {
			gHopCounter = 0;
			
			gCachedInputBufferPointer = gInputBufferPointer;
			Bela_scheduleAuxiliaryTask(gFftTask);
		}
		
		// this schedules the imu sensor readings
		if(++readCount >= readIntervalSamples) {
			readCount = 0;
			Bela_scheduleAuxiliaryTask(i2cTask);
		}
		
		//read the value of the button
		int buttonValue = digitalRead(context, 0, buttonPin); 

		// if button wasn't pressed before and is pressed now
		if( buttonValue != lastButtonValue && buttonValue == 1 ){
			// then run calibration to set looking forward (gGravIdle) 
			// and looking down (gGravCal)
			switch(calibrationState) {
			case 0: // first time button was pressed
				setForward = 1;
				// run task to get gravity values when sensor in neutral position
				Bela_scheduleAuxiliaryTask(gravityNeutralTask);
				calibrationState = 1;	// progress calibration state
				break;
			case 1: // second time button was pressed
				// run task to get gravity values when sensor 'looking down' (for head-tracking) 
			 	Bela_scheduleAuxiliaryTask(gravityDownTask);
				calibrationState = 0; // reset calibration state for next time
				break;
			} 
		}
		lastButtonValue = buttonValue;
		
		// map yaw, pitch, or roll to frequency for synth
		// change ypr[0] to ypr[1] or ypr[2] to access the other axes
		//gFrequency = map(ypr[0], M_PI*-0.5, M_PI*0.5, 100, 800);
		float downSampleFactor = map(ypr[1], M_PI*-0.5, M_PI*0.5, 0.0, 1.0);
		//float bitcrushFactor = map(ypr[2], M_PI*-0.5, M_PI*0.5, 0.8, 1.0);
		decimator.SetDownsampleFactor(downSampleFactor);
		//decimator.SetBitcrushFactor(bitcrushFactor);
		out = decimator.Process(out);
		outDelay = decimator.Process(outDelay);
        out = lpFilter.process(out);
        outDelay = lpFilter.process(outDelay);
        
		// Write the audio to the output
		audioWrite(context, n, 0, out);
    	audioWrite(context, n, 1, outDelay);
		// for(unsigned int channel = 0; channel < context->audioOutChannels; channel++) {
		// 	audioWrite(context, n, channel, out);
		// }
		
	}
}


// Auxiliary task to read from the I2C board
void readIMU(void*)
{
	// get calibration status
	uint8_t sys, gyro, accel, mag;
	bno.getCalibration(&sys, &gyro, &accel, &mag);
	// status of 3 means fully calibrated
	//rt_printf("CALIBRATION STATUSES\n");
	//rt_printf("System: %d   Gyro: %d Accel: %d  Mag: %d\n", sys, gyro, accel, mag);
	
	// quaternion data routine from MrHeadTracker
  	imu::Quaternion qRaw = bno.getQuat(); //get sensor raw quaternion data
  	
  	if( setForward ) {
  		gIdleConj = qRaw.conjugate(); // sets what is looking forward
  		setForward = 0; // reset flag so only happens once
  	}
		
  	steering = gIdleConj * qRaw; // calculate relative rotation data
  	quat = gCalLeft * steering; // transform it to calibrated coordinate system
  	quat = quat * gCalRight;

  	ypr = quat.toEuler(); // transform from quaternion to Euler
}

// Auxiliary task to read from the I2C board
void getNeutralGravity(void*) {
	// read in gravity value
  	imu::Vector<3> gravity = bno.getVector(I2C_BNO055::VECTOR_GRAVITY);
  	gravity = gravity.scale(-1);
  	gravity.normalize();
  	gGravIdle = gravity;
}

// Auxiliary task to read from the I2C board
void getDownGravity(void*) {
	// read in gravity value
  	imu::Vector<3> gravity = bno.getVector(I2C_BNO055::VECTOR_GRAVITY);
  	gravity = gravity.scale(-1);
  	gravity.normalize();
  	gGravCal = gravity;
  	// run calibration routine as we should have both gravity values
  	calibrate(); 
}

// calibration of coordinate system from MrHeadTracker
// see http://www.aes.org/e-lib/browse.cfm?elib=18567 for full paper
// describing algorithm
void calibrate() {
  	imu::Vector<3> g, gravCalTemp, x, y, z;
  	g = gGravIdle; // looking forward in neutral position
  
  	z = g.scale(-1); 
  	z.normalize();

  	gravCalTemp = gGravCal; // looking down
  	y = gravCalTemp.cross(g);
  	y.normalize();

  	x = y.cross(z);
  	x.normalize();

  	imu::Matrix<3> rot;
  	rot.cell(0, 0) = x.x();
  	rot.cell(1, 0) = x.y();
  	rot.cell(2, 0) = x.z();
  	rot.cell(0, 1) = y.x();
  	rot.cell(1, 1) = y.y();
  	rot.cell(2, 1) = y.z();
  	rot.cell(0, 2) = z.x();
  	rot.cell(1, 2) = z.y();
  	rot.cell(2, 2) = z.z();

  	gCal.fromMatrix(rot);

  	resetOrientation();
}

// from MrHeadTracker
// resets values used for looking forward
void resetOrientation() {
  	gCalLeft = gCal.conjugate();
  	gCalRight = gCal;
}

void cleanup(BelaContext *context, void *userData)
{
	for(auto t : gTouchSensors)
		delete t;
}
