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

		if (~settingsLoaded.not) { "Settings are not loaded".warn; break.() };

		/////// KERNEL DECODER
		~decoder !? { ~decoder.free };
		~decoder = switch(~binDecoder.which,
			\cipic,     { FoaDecoderKernel.newCIPIC(~binDecoder.subjectID) },
			\listen,    { FoaDecoderKernel.newListen(~binDecoder.subjectID) },
			\spherical, { FoaDecoderKernel.newSpherical() },
			{ "Invalid decoder".warn; break.() }
		);
		s.sync; // let the kernels load
		start = Main.elapsedTime;
		while { ~decoder.kernelInfo.size == 1 } {
			if (Main.elapsedTime - start > kernelTimeout) {
				"Couldn't load decoder kernel".warn; break.();
			};
			0.05.wait;
		};
		postf("Decoder:\t%\n", ~decoder);

		/////// SOUNDFILES
		~testBuffers !? { ~testBuffers.do(_.free) };
		~testBuffers = ~testFiles.collect(CtkBuffer.playbuf(_));
		s.sync;

		/////// SYNTHDEFS
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


			/* Transforms */

			// TODO: SynthDef.wrap these up
			//		 \_inline versions with ReplaceOut
			SynthDef(\push, {
				arg inbus, outbus = 0, amp = 1,
					theta = 0.0, phi = 0.0, amount = (pi/4);
				var foa;
				foa = In.ar(inbus, 4); // FOA input
				foa = FoaPush.ar(foa, amount, theta, phi, amp);
				Out.ar(outbus, foa)
			}),

			SynthDef(\press, {
				arg inbus, outbus = 0, amp = 1,
					theta = 0.0, phi = 0.0, amount = (pi/4);
				var foa;
				foa = In.ar(inbus, 4); // FOA input
				foa = FoaPress.ar(foa, amount, theta, phi, amp);
				Out.ar(outbus, foa)
			}),

			SynthDef(\focus, {
				arg inbus, outbus = 0, amp = 1,
					theta = 0.0, phi = 0.0, amount = (pi/4);
				var foa;
				foa = In.ar(inbus, 4); // FOA input
				foa = FoaFocus.ar(foa, amount, theta, phi, amp);
				Out.ar(outbus, foa)
			}),

			/* Decoder */

			SynthDef(\decoder, { arg inbus, outbus = 0, amp = 1;
				var foa, decode;
				foa = In.ar(inbus, 4); // FOA input
				decode = FoaDecode.ar(
					foa, ~decoder, mul: amp // ~decoder in settings
				);
				Out.ar(outbus, decode)
			}),
		);
		s.sync;

		"SynthDefs loaded.".postln;
	}
}); // end .waitForBoot