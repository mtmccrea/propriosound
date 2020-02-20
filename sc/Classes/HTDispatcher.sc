HTDispatcher {
	var lowcc, hicc; // copyArgs

	var <euler, <q;
	var lastlo = 0, lasthi=0;
	var lowbits, hibits;
	var lowp5, hip3;
	var norm, e;
	var postCounter = 0;
	var <>postEvery = 10;
	var <>postYPR = false;
	var <rotationSequence = \rpy; // rpy: roll -> pitch -> yaw  (extrinsic rotations)
	//							     ypr:  yaw -> pitch -> roll (extrinsic rotations)
	var <>invertQuaternions = false;
	var <>normalizeQuaternions = true;

	// lowccs, hiccs:
	// 			indicies of fisrt ccnums of the two quaternion blocks
	// 			e.g. 16, 48
	// Each block expected to be consecutive for now
	*new { |lowcc, hicc|
		^super.newCopyArgs(lowcc, hicc).init;
	}

	init {
		lowbits = 0!4;
		hibits  = 0!4;
		q       = 0!4;
		euler   = 0!3;

		lowp5 = lowcc + 5; // to test if message is less than this
		hip3 = hicc + 3;
		norm = 2.pow(14).reciprocal;
		this.rotationSequence_(rotationSequence);
	}

	rotationSequence_ { |seqSymbol|

		switch(seqSymbol,
			\rpy, { e = -1 }, // roll -> pitch -> yaw (extrinsic rotations)
			\ypr, { e =  1 }, //  yaw -> pitch -> roll (extrinsic rotations)
			{ "invalid rotation sequence: \rpy or \ypr only".warn }
		);
		rotationSequence = seqSymbol;
	}

	setVal { |idx, val|
		if (idx < lowp5) { // low bit
			lowbits[idx-lowcc] = val;
		} { // high bit
			hibits[idx-hicc] = val;
			this.checkLastHiSet(idx);
		}
	}

	// the high bits should be set after the low bits,
	// so check if the last high bit that was set is the
	// last in the block, or if it's less than the last
	// non-dispatched set (meaning the block was completed
	// without setting the highest bits (if they didn't
	// change so weren't sent...rare);
	checkLastHiSet { |justSetIdx|
		if (justSetIdx == hip3) { // last high bit in the block was set
			this.dispatch;
		} {
			if (justSetIdx < lasthi) { // last block wasn't filled, send it as is
				this.dispatch;
			} {
				lasthi = justSetIdx;
			}
		}
	}

	dispatch {
		this.calcq;
		this.quatToEuler;
		if (postYPR) {postf("Y: %\n\tP: %\n\t\tR: %\n", *euler.raddeg) };
		lasthi = 0; // reset hi counter

		postCounter = mod(postCounter + 1, postEvery);
	}

	calcq {
		// qwVal = (128 * message.getControllerValue() + qwLsb) * (1.0f / 16384);
		q = ((128 * lowbits) + hibits) * norm;
	}

	quatMagnitude {
		^sqrt(q.squared.sum)
	}

	quatNormalize {
		var mag = this.quatMagnitude;
		if (mag != 0.0) {
			q = q * mag.reciprocal;
		}
	}

	quatGetConj {
		^q * [1,-1,-1,-1]
	}

	quatToEuler {
		var p0, p1, p2, p3;
		var t0, t1, ypr;


		// ~~ via SceneRotatorAudioProcessor ~~~~~~~~~~~~~~~
		// og author: Daniel Rudrich
		// https://git.iem.at/audioplugins/IEMPluginSuite/blob/master/SceneRotator/Source/PluginProcessor.cpp
		// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
		if (normalizeQuaternions) {
			this.quatNormalize; // normalizes q
		};

		if (invertQuaternions) {
			q = this.quatGetConj;
		};

		#p0, p1, p2, p3 = q;

		ypr = Array.newClear(3);

		// pitch (y-axis rotation)
		t0 = 2.0 * ((p0 * p2) + (e * p1 * p3));
		ypr[1] = asin(t0);

		if ((ypr[1] == pi) || (ypr[1] == -pi)) {
			ypr[2] = 0.0;
			ypr[0] = atan2(p1, p0);
		} {
			// yaw (z-axis rotation)
			t0 =        2.0 * ((p0 * p1) - (e * p2 * p3));
			t1 = 1.0 - (2.0 * ((p1 * p1) + (p2 * p2)));
			ypr[0] = atan2(t0, t1);

			// roll (x-axis rotation)
			t0 =        2.0 * ((p0 * p3) - (e * p1 * p2));
			t1 = 1.0 - (2.0 * ((p2 * p2) + (p3 * p3)));
			ypr[2] = atan2(t0, t1);
		};

		euler = ypr;

		/*
		// ~~ via ArduinoQuaternion ~~~~~~~~~~~~~~~~~~~~~~~
		// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

		euler[0] = atan2(  // psi
			2 * ((q[1] * q[2]) - (q[0] * q[3])),
			2 * ((q[0] * q[0]) + (q[1] * q[1])) - 1
		);

		euler[1] = asin(   // theta
			2 * ((q[1] * q[3]) + (q[0] * q[2]))
		).neg;

		euler[2] = atan2(  // phi
			2 * ((q[2] * q[3]) - (q[0] * q[1])),
			2 * ((q[0] * q[0]) + (q[3] * q[3])) - 1
		);
		*/
	}
}
