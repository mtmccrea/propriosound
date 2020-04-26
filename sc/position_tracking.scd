/* RAZOR IMU - Serial
* Absolut position tracking.
*
* Position algorithm and quaternion functions ported and adapted from:
* https://github.com/xioTechnologies/Gait-Tracking-With-x-IMU
*
* Author: Michael McCrea, 2020
*/

SerialPort.listDevices

~posListenerL = ArduinoSMS("/dev/tty.usbmodem14101", 9600); // Razor position firmware default
// ~posListenerR = ArduinoSMS("/dev/tty.usbmodem14201", 9600); // Razor position firmware default

~posListenerL.close
~posListenerR.close

/* clean up Serial */
SerialPort.closeAll
~posListenerL.free
~posListenerR.free

/* zero-velocity movement threshold */
~moveThresh = 0.19;
~postAccMag = true;		// post acceleration magnitude to set ~moveThresh
~postAccMag = false;
~postEvery = 25;		// post acceleration data every N messages
~postMoveEvery = 3;		// post movement status every M messages

/* quick resets */
~posListenerL.action_({|msg| msg.postln});
~posListenerL.action_({});  	// disable left serial responder
~posListenerR.action_({});  	// disable right serial responder
~resetPosL = ~resetPosR = true;	// reset positions

/* **** Helper Functions **** */
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~
((
	var sqrt2_2 = sqrt(2)/2; // precompute
	var posOriginL = [30, -5, -10] / 981; // tap reset position of left hand [NWU cm]
	var posOriginR = [30,  5, -10] / 981; // tap reset position of right hand [NWU cm]
	var qConj, qProd, qRotate, axisAngle2q, calcPos;
	var ned2enuQuat, xyRot45Quat;
	var moveCnt = 0;

	~moveThresh = ~moveThresh ?? 0.15;    // ensure these vars are initialized
	~postAccMag = ~postAccMag ?? false;
	~postEvery = ~postEvery ?? 25;
	~postMoveEvery = ~postMoveEvery ?? 3;

	/* quaternion functions */
	qConj = { |q| q * [1,-1,-1,-1] };

	qProd = { |a, b|
		var ab = Array.newClear(4);
		var a0, a1, a2, a3;
		var b0, b1, b2, b3;
		#a0, a1, a2, a3 = a;
		#b0, b1, b2, b3 = b;

		ab[0] = (a0*b0) - (a1*b1) - (a2*b2) - (a3*b3);
		ab[1] = (a0*b1) + (a1*b0) + (a2*b3) - (a3*b2);
		ab[2] = (a0*b2) - (a1*b3) + (a2*b0) + (a3*b1);
		ab[3] = (a0*b3) + (a1*b2) - (a2*b1) + (a3*b0);
		ab // return
	};

	qRotate = { |vec3, q|
		var vXYZ = qProd.(
			qProd.(q, [0] ++ vec3),
			qConj.(q)
		);
		vXYZ[1..3]
	};

	axisAngle2q = { |axis, angle|
		var halfAngle = angle/2;
		// return [q0, q1, q2, q3]
		[cos(halfAngle)] ++ (axis * sin(halfAngle)).neg;    // TODO: move .neg inside on sin op
	};

	// --- Precalculate rotation quaternions ---
	ned2enuQuat = axisAngle2q.([sqrt2_2, sqrt2_2, 0], pi);  // 180˚ rotation about 45˚ unit vecot on XY plane
	xyRot45Quat = axisAngle2q.([0, 0, 1], -pi/4);           // 45˚ rotation about Z unit vector
	// 													    // TODO: if these are correct, they can likely be combined

	calcPos = { |acc_imu, quat_imu, pos_prev = ([0,0,0]), vel_prev = ([0,0,0]), timestep = 0.01|
		var acc_imu_enu, quat_imu_enu;
		var acc_world, vel, pos, moving, accmag;

		// --- Convert sensor acceleration: NED (Razor) -> ENU (x-IMU/Gait tracker) ---
		// acc_imu_enu = [acc_imu[1], acc_imu[0], acc_imu[2].neg];
		acc_imu_enu = acc_imu;

		// --- Convert sensor orientation: NED (Razor) -> ENU (x-IMU/Gait tracker) ---
		quat_imu_enu = qProd.(ned2enuQuat, quat_imu);

		// --- Rotate body accelerations to Earth frame ---
		acc_world = qRotate.(acc_imu_enu, quat_imu_enu);
		acc_world = qRotate.(acc_world, xyRot45Quat);       // correction term for 45˚ xy plane offset rotation problem

		// --- Remove gravity from measurements    			// TODO check reasoning behind + rather than -
		acc_world[2] = acc_world[2] + 1;       	   			// TODO unnecessary due to velocity integral drift compensation ?

		// --- Detect movement from acceleration magnitude ---
		accmag = acc_world.squared.sum.sqrt; // TODO: use a cheaper norm? rm .sqrt
		moving = accmag > ~moveThresh;
		// 													// TODO: the threshold test could move before the acceleration
		// 													// coversions, but needs to handle gravity properly

		// --- Integrate twice to yield position ---
		if (moving) {
			vel = vel_prev + (acc_world * timestep); // integrate acceleration to yield velocity
			pos = pos_prev + (vel * timestep); 		 // integrate velocity to yield position

			if (~postAccMag and: { (moveCnt % ~postMoveEvery) == 0 }, { 1.post });
		} {
			vel = [0, 0, 0]; // force zero velocity when foot stationary
			pos = pos_prev;
		};

		// post useful values
		if (moveCnt == 0) {
			"\n% %\n".postf(
				// acc_world, accmag
				pos * 981, // conver to cm
				accmag
			)
		};
		moveCnt = (moveCnt + 1) % ~postEvery;

		// // % Compute integral drift during non-stationary periods
		// velDrift = zeros(size(vel));
		// stationaryStart = find([0; diff(stationary)] == -1);
		// stationaryEnd = find([0; diff(stationary)] == 1);
		// (1..stationaryEnd.size).do{ |i|
		// 	driftRate = vel(stationaryEnd(i)-1, :) / (stationaryEnd(i) - stationaryStart(i));
		// 	enum = 1:(stationaryEnd(i) - stationaryStart(i));
		// 	drift = [enum'*driftRate(1) enum'*driftRate(2) enum'*driftRate(3)];
		// 		velDrift(stationaryStart(i):stationaryEnd(i)-1, :) = drift;
		// 	};
		// 	// % Remove integral drift
		// 	vel = vel - velDrift;

		[pos, vel, acc_world]	// return
	};

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
				// unpack data
				// args: [ "#TAQ,time,acc1,acc2,acc3,quat1,quat2,quat3,quat4" ]
				data = args[0].drop(6).split($,).asFloat; // drop "#TAQ="
				time = data[0].asFloat;
				acc_imu  = data[1..3].asFloat;
				quat_imu = data[4..].asFloat;
				time_prev ?? { time_prev = time + 10 }; // default init sample period 10ms
				elapsedTime = (time - time_prev) * toSecs; // sample period

				#pos, vel, acc_world = calcPos.(
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
							~posBusL.set(pos_prev * 981) // convert to cm
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
			};
		};
		listenerFunc // return the function
	};

	// zero out left/right position
	~resetPosL = ~resetPosR = true;

	// Debug: for external access/testing
	// cleanup vars first
	~quaternConj = ~quaternProd = ~quaternRotate = ~axisAngle2quatern = ~calcPos = nil;
	~quaternConj = qConj;
	~quaternProd = qProd;
	~quaternRotate = qRotate;
	~axisAngle2quatern = axisAngle2q;
	~calcPos = calcPos;

)) // .. end function definitions



/* initialize position serial listener actions */

~posListenerL.action_(~genListenerFunc.(id: 0)) // left, id: 0
~posListenerR.action_(~genListenerFunc.(id: 1)) // right, id: 1

~genListenerFunc.(id: 0)


/* one-liners for quick tweaks */
~posListenerL.action_({});  // disable left serial responder
~posListenerR.action_({});  // disable right serial responder
~resetPos = true;			// reset positions

~moveThresh = 0.15;
~postAccMag = true;			// post acceleration magnitude to set ~moveThresh
~postAccMag = false;

~moveThresh = 0.1;

/* **** SANDBOX ***** */

/* Debug: standalone serial listener, position data recording */
(
/* Razor default "#TAQ=" Time, Acceleration, Quaternions */
// p_ previous frame values
var time_prev, pos_prev = [0,0,0], vel_prev = [0,0,0];
var toSecs = 1000.reciprocal;
var cnt = 1, storeCnt = 0;
var maxStoreFrames;

// store a buffer of the sensor data
maxStoreFrames = 2000; // duration ~ nfrm * 10 / 1000
~dataStore !? { ~dataStore = nil };
~dataStore = (0!8) ! maxStoreFrames; // [[timestamp, x, y, z, q0, q1, q2, q3 ], ... ]
~resetPos = true;

~posListenerL.action_({
	|...args|
	var data, time, acc_imu, quat_imu;
	var pos, vel, acc_world, elapsedTime;
	var bundle;

	try {       												// TODO: DEBUG, remove try
		// unpack data
		// args: [ "#TAQ,time,acc1,acc2,acc3,quat1,quat2,quat3,quat4" ]
//1.postln;
		data = args[0].drop(6).split($,).asFloat; // drop "#TAQ="
		time = data[0].asFloat;
		acc_imu  = data[1..3].asFloat;
		quat_imu = data[4..].asFloat;
		time_prev ?? { time_prev = time + 10 }; // default init sample period 10ms
		elapsedTime = (time - time_prev) * toSecs; // sample period
//2.postln;

		// store raw data for plotting/writing to file, etc.
		if (storeCnt < maxStoreFrames) {
			~dataStore[storeCnt] = data;              // TODO: DEBUG, remove data logging
		};
//3.postln;

		#pos, vel, acc_world = ~calcPos.(
			acc_imu, quat_imu, pos_prev, vel_prev, elapsedTime
		);
// "calculated pos, vel, acc:\n\t%\n\t%\n\t%\n".postf(pos, vel, acc_world);
		// external reset control
		if (~resetPos) {
			pos_prev  = [0,0,0];
			vel_prev  = [0,0,0];
			~resetPos = false;
		} {
			pos_prev = pos;
			vel_prev = vel;
		};
		time_prev = time;
//4.postln;
		// write to bus
		~posBusL !? { ~posBusL.set(pos_prev) };

		// post every n iterations                   		// TODO: DEBUG, remove posting
		cnt = cnt % 25; // post every 25 vals
		if (cnt == 0) {
			var acc, accm, accstr;
			acc = acc_world;
			accstr = acc.round(0.001).collect({ |me| me.asString.padLeft(6) });
			accm = acc.squared.sum.sqrt;
			postf("% %\n", accstr, accm);
		};
//5.postln;

		// increment post/log counter
		cnt = cnt+1;
		if (storeCnt < maxStoreFrames) { // stop counting after writing full buffer
			storeCnt = storeCnt+1;
		};
	} { |error| ~posListenerL.action_({}); error.reportError };

})
)

~posListenerL.action_({});         // disable serial responder
~resetPos = true;

( // write logged data to a test file:
f = CSVFileWriter("~/Desktop/test.csv".standardizePath);
~dataStore.do(f.writeLine(_));
f.close;
)
// plot logged data
(
var accdata = ~dataStore.collect({|me| me[1..3] });
accdata.shape;
p = accdata.flop.plot
)
~dataStore.size


/* Utils */

// time message frequency
~prev = Main.elapsedTime;
a.action_({|msg| var now; /*msg.postln;*/ now = Main.elapsedTime; ~period = now - ~prev; ~prev = now})
~period.reciprocal


// test: write position to a bus
s.boot;
~posBusL = CtkControl(3).play;
ControlPlotter(~posBusL.bus, 3).start; // 3-in-1 scope
~posBusL.set([1,2,3]/2)

// individual scopes
(
~scopes = [\x,\y,\z].collect{ |ax, i|
	var newLoc, scrn = Window.availableBounds;
	var cp = ControlPlotter(~vBus.bus+i, 1).start;

	cp.mon.plotter.parent.name = ax;
	newLoc = scrn.copy.width_(scrn.width/3).height_(scrn.height/3.3);
	newLoc.top = scrn.height-(scrn.height/3 * (i+1));
	cp.mon.plotter.parent.bounds_(newLoc);
	// cp.mon.plotter.plots[0].bounds_(newLoc);

	cp // return
}
)
~scopes.do({|scp, i| scp.bounds_(-2,2) })
~scopes.do({|scp, i| scp.bounds_(\auto) })


// test ~calcPos function
(
var pos, vel;
#pos, vel = ~calcPos.(~acc, ~quat, ~pos, ~vel, ~time-~time_prev/1000);
~pos = pos;
~vel = vel;
)

