/* Position tracking via acceleration and gyroscope.
* Includes quaternion math, function to generate
* serial message hand position responder, and tap resetting.
*/
var posOriginL = [30,  5, -10] / 981; // tap reset position of left hand [NWU cm]
var posOriginR = [30, -5, -10] / 981; // tap reset position of right hand [NWU cm]

// id: L = 0, R = 1
~genListenerFunc = { |id|
	/* Razor default "#TAQ=" Time, Acceleration, Quaternions */
	// state vars
	var time_prev, acc_prev = [0,0,0];
	var pos_prev = [0,0,0], vel_prev = [0,0,0];
	var toSecs = 1000.reciprocal;
	var listenerFunc;

	listenerFunc = {
		|...args|
		var data, time, acc_imu, quat_imu;
		var pos, vel, acc_world, elapsedTime;

		try{       												// TODO: DEBUG, remove try
			var key = args[0].keep(5);
			if (key == "#TAP=") { // TAP
				// 62.postln;
				// data = args[0].drop(5).split($,).asFloat; // drop "#TAP="
				// 62.postln;
				postf("TAP\t%\n", data);
				// 63.postln;
				// ~countTap !? {~countTap.()};
				// 64.postln;
			} { // Orientation: "#TAQ="
				// args: [ "#TAQ=time,acc1,acc2,acc3,quat1,quat2,quat3,quat4" ]
				data = args[0].drop(5).split($,).asFloat; // drop "#TAQ="
				time = data[0].asFloat;
				acc_imu  = data[1..3].asFloat;
				quat_imu = data[4..].asFloat;
				time_prev ?? { time_prev = time + 10 }; // default init sample period 10ms
				elapsedTime = (time - time_prev) * toSecs; // sample period

				#pos, vel, acc_world = ~calcPos.(
					acc_imu, quat_imu, pos_prev, vel_prev, elapsedTime
				);

				// external reset control, write position to bus
				switch(id,
					0, {
						if (~resetPosL) {
							pos_prev  = posOriginL;
							vel_prev  = [0,0,0];
							~resetPosL = false;
						} {
							pos_prev = pos;
							vel_prev = vel;
						};
						// write to bus
						~posBusL !? {
							~posBusL.set(
								pos_prev
								// * [-1,1,-1]  // convert to ambi coords
								* 981  				  // convert to cm
							)
						}
					},
					1, {
						if (~resetPosR) {
							pos_prev  = posOriginR;
							vel_prev  = [0,0,0];
							~resetPosR = false;
						} {
							pos_prev = pos;
							vel_prev = vel;
						};
						// write to bus
						~posBusR !? {
							~posBusR.set(pos_prev * 981) // convert to cm
						}
					}
				);
				time_prev = time;
			}
		} { |err|
			"pos listener problem.".warn;
			~posListenerL !? {~posListenerL.action_({})};
		};
	};
	listenerFunc // return the function
};

~setMoveThresh = { |thresh|
	~moveThresh = thresh;
	~moveMeter !? {
		~moveMeter.setThreshold(0, thresh); // color
	};
};


// Movement threshold meter
~moveMeter !? {~moveMeter.close};
fork({ 0.5.wait; // wait for previous to close
	~moveMeter = LevelMeter(label: "Move mag",
		labelAlign: \topLeft, rangeLabelAlign: \right, levelLabelAlign: \bottomLeft)
	.spec_([0.0001, 1, \exp].asSpec)
	.showPeak_(true)
	.levelFontSize_(18).rangeFontSize_(8)
	.rangeFont_(protoString: "-00") // -90 is lowest value, so only need this much space
	.defaultColor_(Color.blue.alpha_(0.5))
	.addThreshold(~moveThresh ?? 0.15, Color.green.alpha_(0.5))
	.decimals_(3).stepped_(false) // no steps in threshold, whole meter follows value
	.value_(0.001).front;
	~moveMeter.meterView.background_(Color.gray.alpha_(0.1));
	~moveMeter.bounds = ~moveMeter.bounds.height = 300;
	~moveMeter.onClose_({~moveMeter = nil});
}, AppClock);

// zero out left/right position
~resetPosL = ~resetPosR = true;

~tapCnt = 0;
~triggered = false;
~resetCnt = 4;
~countTap = {
	~tapCnt = ~tapCnt+1;
	~tapCnt.postln;
	SystemClock.sched(2, {
		~tapCnt = 0;
		// "tapCnt reset".postln
	});
	if (~tapCnt > (~resetCnt-1) and: {~triggered.not}) {
		~triggered = true;
		~synthdefs !? {
			~synthdefs[\resetBell].note.freq_(700).dur_(0.5).amp_(-12.dbamp).play;
		};
		~resetPosL = ~resetPosR = true;
		"TRIGGERED reset".postln;
		SystemClock.sched(1.5, {
			~triggered = false;
		});
	};
};
