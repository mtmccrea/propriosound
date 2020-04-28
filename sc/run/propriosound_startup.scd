/*
* Main process for propriosound.
*
* Author: Michael McCrea, 2020
*/
(
~projectPath = PathName(
	"/Users/dyne/Documents/Courses/Emergent User interfaces/"
);

// load settings
this.executeFile((~projectPath +/+ "sc/run/propriosound_settings.scd").fullPath);

// load synths
this.executeFile((~projectPath +/+ "sc/run/propriosound_server.scd").fullPath);
)

/* UTILS */

// inspect available synthdefs
~synthdefs.synthdefs.collect(_.name).postlns;


/* STARTUP */

(

var resetDur = 1.5;
var resetCnt = 3;    // reset position after this many taps (sum of both hands)
var triggered = false;
var tapCnt = 0;

~countTap = {
	tapCnt = tapCnt+1;
	tapCnt.postln;
	SystemClock.sched(resetDur, {
		tapCnt = 0;
		// "tapCnt reset".postln
	});
	if (tapCnt > (resetCnt-1) and: triggered.not) {
		triggered = true;
		~synthdefs[\resetBell].note.freq_(700).dur_(0.5).amp_(-12.dbamp).play;
		~resetPosL = ~resetPosR = true;
		"TRIGGERED reset".postln;
		SystemClock.sched(resetDur, {
			triggered = false;
		});
	};
};


// GROUPS
fork {
	block { |break|
		var whichEncoder, whichDirector;
		var stepcnt = 0; // loading status vars
		var logProgress;
		var dirclipTheta;

		logProgress = { |str|
			~postStatus.if{ "[%] %\n".postf(stepcnt = stepcnt+1, str) };
		};

		// ~~~ GROUPS ~~~~~~~~~~~~~~~~~
		// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~
		~grps  !? { ~grps.do(_.free) };
		~decGrp = CtkGroup(addAction: \tail, target: 1).play;			s.sync;
		~xfGrp  = CtkGroup(addAction: \before, target: ~decGrp).play;	s.sync;
		~encGrp = CtkGroup(addAction: \before, target: ~xfGrp).play;	s.sync;
		~inGrp  = CtkGroup(addAction: \before, target: ~encGrp).play;	s.sync;
		~grps   = [~decGrp, ~xfGrp, ~encGrp, ~inGrp];
		logProgress.("Groups playing");

		// ~~~ INPUT SYNTHS & BUSSES ~~
		// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~
		~inputBus !? { ~inputBus.free };

		~busses !? { ~busses.do(_.free) };
		~busses = List.with(
			~posBusL  = CtkControl.play(3), // Serial input routine writes here
			~posBusR  = CtkControl.play(3),
			~dirBusL  = CtkControl.play(3),
			~dirBusR  = CtkControl.play(3),
			~headBus  = CtkControl.play(3),
		);
		s.sync;

		// create individual busses for rho, theta, phi for individual access
		~dirBussesL = 3.collect{ |i|
			var bus = CtkControl.play(1, bus: ~dirBusL.index+i);
			~busses.add(bus);
			bus; // return
		};
		~dirBussesR = 3.collect{ |i|
			var bus = CtkControl.play(1, bus: ~dirBusR.index+i);
			~busses.add(bus);
			bus; // return
		};
		s.sync;

		// Set up a decoder bus
		~decBus !? { ~decBus.free };
		~busses.add(
			~decBus = CtkAudio.play(4)
		);
		s.sync;

		// Choose input: test or hardware input

		if (~useTestInput) {
			/* use test buffers */
			var bufs = if (~testfileIdxs.isKindOf(Symbol)) {
				switch (~testfileIdxs,
					'rand', {
						~testBuffers.scramble[0..~hwInChans.size-1]; // randomize buffer order
					}, { "Unrecognized input channels!".warn; break.() }
				)
			} { // ~testfileIdxs is an array of indices into ~testBuffers[]
				~testBuffers[~testfileIdxs]
			};

			~busses.add(
				~inputBus = CtkAudio.play(bufs.size)
			);
			s.sync;

			~inSynths = bufs.collect{ |buf, i|
				~synthdefs[\sfPlayback].note(addAction: \head, target: ~inGrp)
				.outbus_(~inputBus.index+i)
				.bufnum_(buf)
				.amp_(1.0)
			}
		} {
			/* use hardware input */
			~busses.add(
				~inputBus = CtkAudio.play(~hwInChans.size)
			);
			s.sync;

			~inSynths = ~hwInChans.collect{ |inchan, i|
				~synthdefs[\hwInput].note(addAction: \head, target: ~inGrp)
				.inbusnum_(inchan) // In.ar(s.options.numOutputBusChannels + inbusnum)
				.outbus_(~inputBus.index+i)
				.recoutbus_(2+i) // channel to record to
				.amp_(1.0)
			}
		};
		logProgress.("Input synths created");


		// ~~~ RESONATOR SYNTHS ~~~
		// ~~~~~~~~~~~~~~~~~~~~~~~~

		~resonSynths = ~inSynths.collect{ |inSynth, i|
			~synthdefs[\resonance].note(addAction: \tail, target: ~inGrp)
			.inoutbus_(inSynth.outbus) // Uses ReplaceOut
			.mix_(0.3)
			.freqs_([300, 600, 900, 1200])
			.amps_([ 0.0164, 0.0112, 0.0152, 0.0158])
			.rings_([1, 2, 1, 1.2])
			.lagtime_(1.3)
			;
		};
		logProgress.("Input processing synths created");

		// ~~~ ENCODER SYNTHS & BUSSES ~~~
		// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

		// encoders write to ~encBus
		~encBus !? { ~encBus.free };
		~busses.add(
			~encBus = CtkAudio.play(~inSynths.size * 4)
		);
		s.sync;

		whichEncoder = ("enc_"++~encodingType).asSymbol;

		~encoderSynths = ~inSynths.collect{ |inSynth, i|
			~synthdefs[whichEncoder].note(addAction: \head, target: ~encGrp)
			.inbus_(inSynth.outbus)
			.outbus_(~encBus.index + (i*4))
			.amp_(1.0);
		};
		logProgress.("Encoder synths created");

		// ~~~ CONTROL BUS ENCODER/FILTER SYNTHS ~~~
		// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

		// position to direction control bus encoders
		dirclipTheta = [[-15, 85], [-85, 15]].degrad; // [[leftLo, leftHi], [rightLo, rightHi]]
		~positionSynths = [
			[~posBusL, ~dirBusL],
			[~posBusR, ~dirBusR]
		].collect{ |posdirbuspair, i|
			var clipLo, clipHi;
			#clipLo, clipHi = dirclipTheta[i];
			~synthdefs[\pos2dir].note(addAction: \tail, target: ~inGrp)
			.inbus_(posdirbuspair[0])   				// position bus
			.outbus_(posdirbuspair[1]) 					// direction bus
			.clipLoTheta_(clipLo).clipHiTheta_(clipHi)
			.clipLoPhi_(-45.degrad).clipHiPhi_(45.degrad)
		};
		// head orientation filtering synth
		~headingSynth =~synthdefs[\heading].note(addAction: \tail, target: ~inGrp)
		.inoutbus_(~headBus); // reads/writes to same bus

		logProgress.("Position control bus synths created");

		// ~~~ ROTATOR & DIRECTOR SYNTHS ~~~
		// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

		// rotators read from and (over)write back to ~encBus
		~rotatorSynths = ~encoderSynths.collect{ |encSynth, i|

			~synthdefs[\rtt_auto].note(addAction: \head, target: ~xfGrp)
			.inbus_(encSynth.outbus)
			.outbus_(encSynth.outbus)
			.randPeriod_(3) // update _period_ of LFDNoise
			.randRange_(2pi)
		};
		logProgress.("Rotator synths created");

		// directors read from ~encBus, write to ~decBus
		whichDirector = ("dir_"++~directorType).asSymbol;
		~directorSynths = ~encoderSynths.collect{ |encSynth, i|

			~synthdefs[whichDirector].note(addAction: \tail, target: ~xfGrp)
			.inbus_(encSynth.outbus)
			.outbus_(~decBus)
			.theta_(0.0).phi_(0.0)
			.amount_(pi/2)
			.amp_(1.0)
		};
		logProgress.("Director synths created");


		// ~~~ REVERB, DECODER SYNTH & BUSSES ~~~
		// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

		~reverbSynth = ~synthdefs[\reverb].note(addAction: \tail, target: ~xfGrp)
		.inoutbus_(~decBus)   // uses ReplaceOut
		.mix_(0.4)
		.predelay_(0.1).cutoff_(800)
		.t60low_(1.0).t60high_(0.8)
		.diffusion_(0.707).diffusionLR_(0.5).diffusionFB_(0.7)
		.mod_(0.3)
		;
		logProgress.("reverb synth created");

		~decoderSynth = ~synthdefs[\decoder].note(addAction: \tail, target: ~decGrp)
		.inbus_(~decBus)
		.outbus_(~hwOutChan)
		.limit_(-1.5.dbamp) // limiter
		.amp_(1.0)
		;
		logProgress.("Decoder synth created");

		// ~~~ HEAD COMPENSATION SYNTH ~~~
		// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

		~headCompSynth = ~synthdefs[\head_comp].note(addAction: \head, target: ~decGrp)
		.inoutbus_(~decBus)  // read/write to same FOA bus
		.headbus_(~headBus)  // yaw, pitch, roll of head orientation
		;
		logProgress.("Head compensation synth created");

		// ~~~ PLAY ~~~
		// ~~~~~~~~~~~~~
		(
			~inSynths ++
			~positionSynths ++
			~headingSynth ++
			~encoderSynths ++
			~resonSynths ++
			~rotatorSynths ++
			~directorSynths ++
			~reverbSynth ++
			~headCompSynth ++
			~decoderSynth
		).do({ |synth| synth.play; s.sync; });
		s.sync;

		logProgress.("Synths playing");
		~postStatus.if{ fork{ 0.3.wait; s.queryAllNodes } };

	} // end block
} // end RUNTIME fork
)

/* RUNTIME - basic process */

/* master output */
~decoderSynth.amp_(-6.dbamp)
~decoderSynth.limit_(-1.5.dbamp) // limiter

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