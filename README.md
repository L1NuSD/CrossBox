# CrossBox
 CrossBox is an audio effect box with live visual controls built for live performances. It is built using Bela micro controller with I2C sensors. The program runs on Bela API using C++ and could be booted on Bela to be performed as a standalone device. 

The box uses two audio input signals and applies cross-synthesis to modulate one signal with the other. The cross-synthesis algorithm is adapted from The Audio Programming Book by Richard Boulanger, and the code uses Bela’s FFT library with auxiliary tasks to run in real time.

The Trill Captive Sensors controls effects including spectral gating , feedback delay, flanger, and a low-pass resonant filter. LEDs are added around the Trill Ring sensor to imply finger touch position when the user controls the delay time.

An Adafruit BNO-055 gyroscope is added to control the bit-crusher effect. The Trill sensors are designed to control live visual effects in TouchDesigner through OSC. Electronic components are soldered on solder board to assure stability and performance. 

CrossBox is aimed to give the performer an intuitive control over audio and visual effects. The performer is allowed to interact with sound and visual using body gestures and finger touches. The design is proved to work well with real time audio in a live performance with modular synth. 
