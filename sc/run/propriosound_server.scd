/*
SynthDefs for propriosound.

Load this file and access through the ~synthdefs variable.

Author: Michael McCrea, 2020
*/

/*
  NOTES
=========
DDWSnippets = ctrl + ~
*/

s.waitForBoot({
	block { |break|
		var binDecoder;
		var kernelTimeout = 2;
		var start;

		if (~settingsLoaded.not) {
			"Settings are not loaded".warn; break.()
		} {
			"Loading Propriosound server resources:".postln;
		};

		/////// KERNEL DECODER

		~decoderKernel !? { ~decoderKernel.free };
		~decoderKernel = switch(~binDecoder.which,
			\cipic,     { FoaDecoderKernel.newCIPIC(~binDecoder.subjectID) },
			\listen,    { FoaDecoderKernel.newListen(~binDecoder.subjectID) },
			\spherical, { FoaDecoderKernel.newSpherical() },
			{ "Invalid decoder".warn; break.() }
		);
		s.sync; // let the kernels load
		start = Main.elapsedTime;
		while { ~decoderKernel.kernelInfo.size == 1 } {
			if (Main.elapsedTime - start > kernelTimeout) {
				"Couldn't load decoder kernel".warn; break.();
			};
			0.05.wait;
		};
		postf("Decoder:\t%\n", ~decoderKernel);


		/////// KERNEL ENCODERS

		~encoderKernels !? { ~encoderKernels.do(_.free) };
		~spreadKernel   = FoaEncoderKernel.newSpread(subjectID: 6,  kernelSize: 512);
		~diffuseKernel  = FoaEncoderKernel.newDiffuse(subjectID: 5, kernelSize: 512);
		~encoderKernels = [~spreadKernel, ~diffuseKernel];
		s.sync;

		"Encoders loaded".postln;


		/////// SOUNDFILES

		~testBuffers !? { ~testBuffers.do(_.free) };
		~testBuffers = ~testFiles.collect{ |str|
			CtkBuffer.playbuf(str).load(sync: true)
		};
		s.sync;


		/////// SYNTHDEFS             /////////////////////////////
		///////////////////////////////////////////////////////////
		~synthdefs = CtkProtoNotes(

			/* input */

			SynthDef(\hwInput, { |inbusnum = 0, outbus = 0, amp = 1.0|
				var sig;
				sig = In.ar(s.options.numOutputBusChannels + inbusnum);
				Out.ar(outbus, sig * amp)
			}),

			SynthDef(\sfPlayback, { |outbus = 0, bufnum, amp = 1.0, t_reset = 1|
				var sig;
				sig = PlayBuf.ar(1, bufnum, trigger: t_reset, loop: 1);
				Out.ar(outbus, sig * amp)
			}),


			/* input processing */

			// NOTE: uses ReplaceOut
			SynthDef(\compander, {
				arg inbus = 0, outbus = 0,
				threshdb = -6, slopeBelow = 1.0, slopeAbove = 1.0,
				clampTime = 0.002, releaseTime = 0.1, amp = 1.0;
				var sig, comp;

				sig = In.ar(inbus);
				comp = Compander.ar(
					in: 		sig,
					control:	sig,
					thresh:		threshdb.dbamp,
					slopeBelow:	slopeBelow,
					slopeAbove:	slopeAbove,
					clampTime:	clampTime,
					relaxTime:	releaseTime,
					mul:		amp
				);
				ReplaceOut.ar(outbus, comp)
			}),

			// NOTE: uses ReplaceOut
			SynthDef(\gate, {
				arg inbus = 0, outbus = 0,
				threshdb = -6, slopeBelow = 1.0, slopeAbove = 1.0,
				clampTime = 0.002, releaseTime = 0.1, amp = 1.0;
				var sig, comp;

				sig = In.ar(inbus);
				comp = Compander.ar(
					in: 		sig,
					control:	sig,
					thresh:		threshdb.dbamp,
					slopeBelow:	slopeBelow,
					slopeAbove:	slopeAbove,
					clampTime:	clampTime,
					relaxTime:	releaseTime,
					mul:		amp
				);
				ReplaceOut.ar(outbus, comp)
			}),

			/* Encoders */

			// TODO: SynthDef.wrap() these up

			SynthDef(\enc_omni, { arg inbus, outbus = 0, amp = 1.0;
				var sig;
				sig = FoaEncode.ar(
					In.ar(inbus, 1), FoaEncoderMatrix.newOmni(), amp
				);
				Out.ar(outbus, sig)
			}),

			SynthDef(\enc_diffuse, { arg inbus, outbus = 0, amp = 1.0;
				var sig;
				sig = FoaEncode.ar(
					In.ar(inbus, 1), ~diffuseKernel, amp
				);
				Out.ar(outbus, sig)
			}),

			SynthDef(\enc_spread, { arg inbus, outbus = 0, amp = 1.0;
				var sig;
				sig = FoaEncode.ar(
					In.ar(inbus, 1), ~spreadKernel, amp
				);
				Out.ar(outbus, sig)
			}),

			/* Transforms */

			// TODO: SynthDef.wrap these up
			//		 \_inline versions with ReplaceOut

			SynthDef(\dir_push, {
				arg inbus, outbus = 0, amp = 1,
					theta = 0.0, phi = 0.0, amount = (pi/4);
				var foa;
				foa = In.ar(inbus, 4); // FOA input
				foa = FoaPush.ar(foa, amount, theta, phi, amp);
				Out.ar(outbus, foa)
			}),

			SynthDef(\dir_press, {
				arg inbus, outbus = 0, amp = 1,
					theta = 0.0, phi = 0.0, amount = (pi/4);
				var foa;
				foa = In.ar(inbus, 4); // FOA input
				foa = FoaPress.ar(foa, amount, theta, phi, amp);
				Out.ar(outbus, foa)
			}),

			SynthDef(\dir_focus, {
				arg inbus, outbus = 0, amp = 1,
					theta = 0.0, phi = 0.0, amount = (pi/4);
				var foa;
				foa = In.ar(inbus, 4); // FOA input
				foa = FoaFocus.ar(foa, amount, theta, phi, amp);
				Out.ar(outbus, foa)
			}),

			// NOTE: this uses ReplaceOut
			SynthDef(\rtt_auto, {
				arg inbus, outbus = 0, randPeriod = 3, randRange = 2pi;
				var foa, freq;
				foa = In.ar(inbus, 4); // FOA input
				freq = randPeriod.reciprocal;
				foa = FoaRTT.ar(foa,
					LFDNoise1.kr(freq).range(0, randRange),
					LFDNoise1.kr(freq).range(0, randRange),
					LFDNoise1.kr(freq).range(0, randRange)
				);
				ReplaceOut.ar(outbus, foa)
			}),

			/* Decoder */

			SynthDef(\decoder, { arg inbus, outbus = 0, amp = 1;
				var foa, decode;
				foa = In.ar(inbus, 4); // FOA input
				decode = FoaDecode.ar(
					foa, ~decoderKernel, mul: amp // see ~binDecoder in settings
				);
				Out.ar(outbus, decode)
			}),
		);
		s.sync;

		"SynthDefs loaded.".postln;
	}
}); // end .waitForBoot