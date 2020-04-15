/*
Main process for propriosound.

Author: Michael McCrea, 2020
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


/* RUNTIME - STARTUP */

(
// GROUPS
fork {
	block { |break|
		var whichEncoder, whichDirector;
		var postStatus = true;
		~busses = List();

		// ~~~ GROUPS ~~~~~~~~~~~~~~~~~
		// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~
		~grps  !? { ~grps.do(_.free) };
		~decGrp = CtkGroup(addAction: \tail, target: 1).play;			s.sync;
		~xfGrp  = CtkGroup(addAction: \before, target: ~decGrp).play;	s.sync;
		~encGrp = CtkGroup(addAction: \before, target: ~xfGrp).play;	s.sync;
		~inGrp  = CtkGroup(addAction: \before, target: ~encGrp).play;	s.sync;
		~grps   = [~decGrp, ~xfGrp, ~encGrp, ~inGrp];
		postStatus.if{ "[1] Groups playing".postln };

		// ~~~ INPUT SYNTHS & BUSSES ~~
		// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~
		~inputBus !? { ~inputBus.free };

		// Choose input: test or hardware input

		if (~useTestInput) { // use test buffers
			var bufs;
			bufs = if (~testfileIdxs.isKindOf(Symbol)) {
				switch (~testfileIdxs,
					'rand', {
						~testBuffers.scramble[0..~hwInChans.size-1]; // randomize buffer order
					}, { "Unrecognized input channels!".warn; break.() }
				)
			} { // ~testfileIdxs is an array of indices into ~testBuffers[]
				~testBuffers[~testfileIdxs]
			};
			// ("\tDEBUG : bufs %\n").postf(bufs);

			~busses.add(
				~inputBus = CtkAudio.play(bufs.size);
			);
			s.sync;

			~inSynths = bufs.collect{ |buf, i|
				~synthdefs[\sfPlayback].note(addAction: \head, target: ~inGrp)
				.outbus_(~inputBus.index+i)
				.bufnum_(buf)
				.amp_(1.0)
			}
		} {
			// use hardware input
			~busses.add(
				~inputBus = CtkAudio.play(~hwInChans.size);
			);
			s.sync;

			~inSynths = ~hwInChans.collect{ |inchan, i|
				~synthdefs[\hwInput].note(addAction: \head, target: ~inGrp)
				.inbusnum_(inchan)
				.outbus_(~inputBus.index+i)
				.amp_(1.0)
			}
		};

		postStatus.if{ "[2] Input synths created".postln };

		// ~~~ ENCODER SYNTHS & BUSSES ~~~
		// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

		// encoders write to ~encBus
		~encBus !? { ~encBus.free };
		~busses.add(
			~encBus = CtkAudio.play(~inSynths.size * 4);
		);

		whichEncoder = ("enc_"++~encodingType).asSymbol;

		~encoderSynths = ~inSynths.collect{ |inSynth, i|

			~synthdefs[whichEncoder].note(addAction: \head, target: ~encGrp)
			.inbus_(inSynth.outbus)
			.outbus_(~encBus.index + (i*4))
			.amp_(1.0);
		};

		postStatus.if{ "[3] Encoder synths created".postln };

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

		postStatus.if{ "[4] Rotator synths created".postln };

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

		postStatus.if{ "[5] Director synths created".postln };

		// ~~~ DECODER SYNTHS & BUSSES ~~~
		// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

		~decBus !? { ~decBus.free };
		~busses.add(
			~decBus = CtkAudio.play(4);
		);
		~decoderSynth = ~synthdefs[\decoder].note(addAction: \tail, target: ~decGrp)
		.inbus_(~decBus)
		.outbus_(~hwOutChan)
		.amp_(1.0);

		postStatus.if{ "[6] Decoder synth created".postln };

		// ~~~ PLAY ~~~
		// ~~~~~~~~~~~~~
		(
			~inSynths ++
			~encoderSynths ++
			~rotatorSynths ++
			~directorSynths ++
			~decoderSynth
		).do(_.play);
		s.sync;

		postStatus.if{ "[7] Synths playing".postln };
		postStatus.if{ 0.3.wait; s.queryAllNodes };

	} // end block
} // end RUNTIME fork
)

/* RUNTIME - PROCESS */
~inGrp.set(0.0, \amp, 0.2)     // set all in group
~xfGrp.set(0.0, \amount, pi/2) // set all in group
~xfGrp.set(0.0, \amount, pi/4) // set all in group

// create dummy busses to simulate movement
(
~azOscL = CtkControl.lfo(LFNoise1, 0.3, -10.degrad, 180.degrad).play;
~azOscR = CtkControl.lfo(LFNoise1, 0.3, -180.degrad, 10.degrad).play;
~elOscL = CtkControl.lfo(LFNoise1, 0.3, -70.degrad, 80.degrad).play;
~elOscR = CtkControl.lfo(LFNoise1, 0.3, -70.degrad, 80.degrad).play;
)

// assign control busses to az/el controls
(
~directorSynths[0].theta_(~azOscL).phi_(~elOscL);
~directorSynths[1].theta_(~azOscR).phi_(~elOscR);
)

// visualize a bus
ControlPlotter(~azOscL.bus, 1).start
// get a bus value
~azOscR.getSynchronous
// scope a bus
~azOscR.scope

// change freq or range of ctl busses
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