/*
* SynthDefs for propriosound.
*
* Load this file and access through the ~synthdefs variable.
*
* Author: Michael McCrea, 2020
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

			SynthDef(\hwInput, { |inbusnum = 0, outbus = 0, recoutbus = 2 amp = 1.0|
				var sig;
				sig = In.ar(s.options.numOutputBusChannels + inbusnum);
				Out.ar(outbus, sig * amp);
				Out.ar(recoutbus, sig * amp);
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

			SynthDef(\resonance, {
				arg inoutbus, mix = 0.5, lagtime = 1.8,
					freqs (#[300, 600, 900, 1200]),
					amps (#[1, 0.3, 0.2, 0.05]),
					rings (#[1, 1, 1, 2]);
				var in, reson, pwrmix;
				in = In.ar(inoutbus, 1);
				// in = LPF.ar(Crackle.ar( MouseX.kr(1,2), 0.001 ), 1800);
				reson = DynKlank.ar(
					`[Lag.kr(freqs, lagtime), Lag.kr(amps, lagtime), rings], in
				);
				pwrmix = mix*2-1; // map (0,1) > (-1,1)
				ReplaceOut.ar(inoutbus,
					(reson * ((1+pwrmix)*0.5).sqrt)  // wet
					+  (in * ((1-pwrmix)*0.5).sqrt)  // dry
				);
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
					theta = 0.0, phi = 0.0, amount = (pi/4),
					rho; // unused
				var foa;
				foa = In.ar(inbus, 4); // FOA input
				foa = FoaPush.ar(foa, amount, theta, phi, amp);
				Out.ar(outbus, foa)
			}),

			SynthDef(\dir_press, {
				arg inbus, outbus = 0, amp = 1,
					theta = 0.0, phi = 0.0, amount = (pi/4),
					rho; // unused
				var foa;
				foa = In.ar(inbus, 4); // FOA input
				foa = FoaPress.ar(foa, amount, theta, phi, amp);
				Out.ar(outbus, foa)
			}),

			SynthDef(\dir_focus, {
				arg inbus, outbus = 0, amp = 1,
					theta = 0.0, phi = 0.0, amount = (pi/4),
					rho; // unused
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

			// Head rotation compensation
			// NOTE: this uses ReplaceOut
			SynthDef(\head_comp, {
				arg inoutbus, headbus;
				var foa, look, yaw, pitch, roll;
				foa = In.ar(inoutbus, 4); // FOA input
				// head look direction
				look = In.kr(headbus, 3); // head's yaw, pitch, roll
				#yaw, pitch, roll = look;
				// TODO: this may need to be individual UGens to get order correct
				foa = FoaTumble.ar(foa, pitch.neg);
				foa = FoaTilt.ar(foa, roll.neg);
				foa = FoaRotate.ar(foa, yaw.neg);
				// overwrites input bus
				ReplaceOut.ar(inoutbus, foa)
			}),

			//
			SynthDef(\reverb, {
				arg inoutbus = 0, mix = 0.25, predelay = 0.05,
					cutoff = 3000, t60low = 2.2, t60high = 1.5,
					diffusion = 1.0, diffusionLR = 1.0, diffusionFB = 1.0,
					mod = 1.0;
				var foa;
				foa = In.ar(inoutbus, 4); // FOA input
				foa = BFormatReverb.ar(foa,
					mix: mix,
					predelay: predelay,
					cutoff: cutoff,
					t60low: t60low,
					t60high: t60high,
					diffusion: diffusion,
					diffusionLR: diffusionLR,
					diffusionFB: diffusionFB,
					mod: mod,
				);
				ReplaceOut.ar(inoutbus, foa)
			}),

			/* Decoder */

			SynthDef(\decoder, { arg inbus, outbus = 0, amp = 1, limit = 0.85;
				var foa, decode;
				foa = In.ar(inbus, 4); // FOA input
				decode = FoaDecode.ar(
					foa, ~decoderKernel, mul: amp // see ~binDecoder in settings
				);
				Out.ar(outbus, Limiter.ar(decode, limit))
			}),


			/* Control signal processing (position, orientation) */

			// translate cartesian hand positions to encoding directions
			SynthDef(\pos2dir, {
				arg inbus, outbus = 0,
				    clipLoTheta=(-15.degrad), clipHiTheta=(100.degrad),
					clipLoPhi=(-75.degrad), clipHiPhi=(75.degrad);
				var x, y, z, xsq, ysq, zsq, rho, theta, phi;
				var cartesian, cartesianSq;

				// cartesian in
				cartesian = In.kr(inbus, 3);
				#x, y, z = cartesian;
				cartesianSq = cartesian.squared;

				// to spherical
				theta = atan2(y, x).clip(clipLoTheta, clipHiTheta);
				phi = atan2(z, cartesianSq[0..1].sum.sqrt).clip(clipLoPhi, clipHiPhi);
				rho = cartesianSq.sum.sqrt;

				Out.kr(outbus, [theta, phi, rho])
			}),

			// capture and filter head orientation
			SynthDef(\heading, { arg inoutbus;
				var headbus = In.kr(inoutbus, 3);

				// add filtering if needed...

				// overwrite input bus
				Out.kr(inoutbus, headbus)
			}),

			// reset signal
			SynthDef(\resetBell, { |outbus = 0, freq = 450, dur = 0.5, amp = 0.4|
				var env, sin;
				env = EnvGen.kr(Env([0,1,0.4,0],[0.01,0.1,0.8], \sin), levelScale: amp, timeScale: dur, doneAction: 2);
				sin = SinOsc.ar(freq * [1,6,8], mul: [0, -6, -12].dbamp).sum * env * 0.333;
				Out.ar(outbus, sin!2);
			}),
		);
		s.sync;

		"SynthDefs loaded.".postln;
	}
}); // end .waitForBoot