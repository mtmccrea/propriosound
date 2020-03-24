/*
Settings for propriosound.

Author: Michael McCrea, 2020
*/

s = Server.default;

o = s.options;
o.numInputBusChannels = 2;
o.numOutputBusChannels = 2;
o.hardwareBufferSize = 512;
o.sampleRate = 44100;
// o.device = "";

/* Choose a binarual decoder */

~binDecoder = IdentityDictionary(know:true)
.put(\which, \cipic) // \cpic, \listen, or \spherical (no elevation)
.put(\subjectID, 3)  // cipic: 3-165, listen: 1002-1059, spherical: 0-9
;

~useTestInput = true;
/* choose which soundfiles to test with as input */
~testfileIdxs = \rand; // [0,2]


"Available hardward devices:".postln;
ServerOptions.devices.do(_.postln);

~sfPath = ~projectPath +/+ "snd/samples/";

~testFiles = ~sfPath.files.collect(_.fullPath);

// hardware I/O channels
~hwInChans = [0, 1];
~hwOutChan = 0;


s.onQuitAdd({
	~settingsLoaded = false;
	"Propriosound settings cleared.".postln;
});

~settingsLoaded = true;
"Settings loaded.".postln;
