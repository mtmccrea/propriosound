
/* RUNTIME - PROCESS */

/* master output */
~decoderSynth.amp_(-6.dbamp)
~decoderSynth.limit_(-1.5.dbamp) // limiter

/* groups */
~inGrp.set(0.0, \amp, 0.2)     // set all in group
~xfGrp.set(0.0, \amount, pi/2) // set all in group
~xfGrp.set(0.0, \amount, pi/4) // set all in group

/* Encoders */
~encoderSynths[0].run
~encoderSynths[0].pause
~encoderSynths[1].run
~encoderSynths[1].pause

~directorSynths[0].theta_(~dirBussesL[0]).phi_(~dirBussesL[1]).rho_(~dirBussesL[2]);
~directorSynths[0].theta_(~dirBussesL[0]).phi_(~dirBussesL[1]);
~directorSynths[0].theta_(45.degrad).phi_(~dirBussesL[1]);
~directorSynths[0].theta_(~dirBussesL[0]).phi_(0.degrad);
// ~directorSynths[0].theta_(15.degrad).phi_(-10.degrad);

/* Resonators */
// wet mix
~resonSynths.do(_.mix_(0)); // 1 = all wet
~resonSynths.do(_.mix_(1)); // 1 = all wet
~resonSynths.do(_.mix_(0.5)); // 1 = all wet

// lag time (freqs and amps)
~resonSynths.do(_.lagtime_(0.05))

( // resonant amps
var amplo = -40;
~resonSynths.do{ |synth|
	synth.amps_(Array.exprand(4, amplo.dbamp, (amplo+8).dbamp).postln);
};
)
( // ring times
var ring = 0.5;
~resonSynths.do{ |synth|
	synth.rings_(4.collect{ring * rrand(0.7, 1.3)})
}
)
( // tuning
var kind, root, scale, ring;
// 'major' 'minor' 'harm-minor' 'melo-minor'
// 'blues' 'blues-major' 'pentatonic' 'chromatic' 'quartertones'
// tuning tables: 'just' 'fifth' 'pythagorean' 'werckmeister' 'indian' 'arabic'
// tuned scales: 'just-major' 'just-minor' 'fifth-major' 'fifth-minor'
kind = 'fifth';
root = "c6";
scale = Array.makeScaleMidi( root.notemidi, kind ).midicps;
~resonSynths.do{ |synth| synth.freqs_(4.collect{scale.choose}) };
)

/* Reverb */
(
~reverbSynth
.mix_(0.4)
.predelay_(0.1).cutoff_(800)
.t60low_(1.0).t60high_(0.8)
.diffusion_(0.707).diffusionLR_(0.5).diffusionFB_(0.7)
.mod_(0.3);
)

~reverbSynth.run

// create dummy busses to simulate movement
(
~azOscL = CtkControl.lfo(LFNoise1, 0.3, -10.degrad, 180.degrad).play;
~azOscR = CtkControl.lfo(LFNoise1, 0.3, -180.degrad, 10.degrad).play;
~elOscL = CtkControl.lfo(LFNoise1, 0.3, -70.degrad, 80.degrad).play;
~elOscR = CtkControl.lfo(LFNoise1, 0.3, -70.degrad, 80.degrad).play;
)

// assign control busses to az/el controls
( // Dummy busses
~directorSynths[0].theta_(~azOscL).phi_(~elOscL);
~directorSynths[1].theta_(~azOscR).phi_(~elOscR);
)
( // actual direction busses
// ~dirBussesL/R theta, phi, rho
~directorSynths[0].theta_(~dirBussesL[0]).phi_(~dirBussesL[1]).rho_(~dirBussesL[2]);
~directorSynths[1].theta_(~dirBussesR[0]).phi_(~dirBussesR[1]).rho_(~dirBussesR[2]);
)

// visualize
ControlPlotter(~azOscL.bus, 1).start

// visualize: head orientation bus
ControlPlotter(~headBus.bus, 3).start

// visualize: hand position busses
ControlPlotter(~posBusL.bus, 3).start
ControlPlotter(~posBusR.bus, 3).start

// visualize: hand direction busses
ControlPlotter(~dirBusL.bus, 3).start
ControlPlotter(~dirBusR.bus, 3).start

// get a bus value
~dirBussesL[0].getSynchronous; // L hand dir, theta
~dirBussesL[1].getSynchronous; // L hand dir, phi
~dirBussesL[2].getSynchronous; // L hand dir, rho

// scope a bus
~azOscR.scope

// change freq or range of dummy ctl busses
~azOscL.freq = 3.reciprocal
~azOscR.freq = 3.reciprocal

~azOscL.low = -10.degrad
~azOscL.high = 160.degrad
~azOscR.low = -160.degrad
~azOscR.high = 10.degrad


// encode input in specific directions
~directorSynths[0].theta_(45.degrad).phi_(70.degrad)
~directorSynths[1].theta_(-15.degrad).phi_(-45.degrad)


~encoderSynths.do(_.pause)
~encoderSynths.do(_.run)
~rotatorSynths
~directorSynths
~decoderSynth


/* CLEANUP */
// To restart synths/processes
(
~grps.do(_.free); ~grps = nil;		// synth groups
~busses.do(_.free); ~busses = nil;	// free busses
[~azOscL, ~azOscR, ~elOscL, ~elOscR].do{ |me| me !? me.free }
)

// To restart config initialization
(
~testBuffers !? { ~testBuffers.do(_.free); ~testBuffers = nil; };
~decoderKernel.free;
~encoderKernels.do(_.free);
)