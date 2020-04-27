// list available devices
ServerOptions.devices.do(_.postln)

// set the desired device *BEFORE* booting the server
// if you change these, the server needs to be rebooted!
// ALSO: make sure each device is running at the same sample rate
s.options.inDevice = "MacBook Pro Microphone";
s.options.outDevice = "MacBook Pro Speakers";

// set the number of hardware input and output channels you want
// supercollider to access:
s.options.numOutputBusChannels_(2); // default is 2 in/out
s.options.numInputBusChannels_(2);

// after the above server options are set, boot the server
// and confirm in the post window that the server booted
// using the devices you chose
s.boot;

/* test sound */

// pan noise between 2 outputs
(
x = {
	Pan2.ar(
		PinkNoise.ar(-9.dbamp),
		SinOsc.kr(4.reciprocal)
)
}.play(output: 0); // play out, starting with the first harware output (0)
)

x.free; // stop the panning noise

// pan noise between N outputs
(
x = {
	PanAz.ar(
		4, // numchannels = 4
		PinkNoise.ar(-9.dbamp),
		LFSaw.kr(4.reciprocal)
)
}.play(output: 0); // play out, starting with the first harware output (0)
)

x.free; // stop the panning noise


