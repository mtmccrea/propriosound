/* RAZOR IMU - Serial
*
* Algorithm ported from:
* https://github.com/xioTechnologies/Gait-Tracking-With-x-IMU
*/

SerialPort.listDevices

// a = ArduinoSMS("/dev/tty.usbmodem14201", 57600); // Razor_AHRS
a = ArduinoSMS("/dev/tty.usbmodem14201", 9600); // Razor position firmware default
a.close

~moveThresh = 0.15;

/* Helper Functions */
(
var sqrt2_2 = sqrt(2)/2; // precompute

~quaternConj = {|q| q * [1,-1,-1,-1] };

~quaternProd = {|a, b|
	var ab = Array.newClear(4);
	var a0, a1, a2, a3;
	var b0, b1, b2, b3;

	#a0, a1, a2, a3 = a;
	#b0, b1, b2, b3 = b;

	// an = a.neg;
	// bn = b.neg;
	// ab(:,1) = a(:,1).*b(:,1) - a(:,2).*b(:,2) - a(:,3).*b(:,3) - a(:,4).*b(:,4);
	// ab(:,2) = a(:,1).*b(:,2) + a(:,2).*b(:,1) + a(:,3).*b(:,4) - a(:,4).*b(:,3);
	// ab(:,3) = a(:,1).*b(:,3) - a(:,2).*b(:,4) + a(:,3).*b(:,1) + a(:,4).*b(:,2);
	// ab(:,4) = a(:,1).*b(:,4) + a(:,2).*b(:,3) - a(:,3).*b(:,2) + a(:,4).*b(:,1);

	ab[0] = (a0*b0) - (a1*b1) - (a2*b2) - (a2*b3);
	ab[1] = (a0*b1) + (a1*b0) + (a2*b3) - (a2*b2);
	ab[2] = (a0*b2) - (a1*b3) + (a2*b0) + (a2*b1);
	ab[3] = (a0*b3) + (a1*b3) - (a2*b1) + (a2*b0);
	ab // return
};

~quaternRotate = { |vec3, q|
	var v0XYZ = ~quaternProd.(
		~quaternProd.(q, [0] ++ vec3),
		~quaternConj.(q)
	);
	v0XYZ[1..3];
};

// axis:  cartesian unit vector [X, Y, Z] for rotation axis
// angle: rotation angle about axis
~axisAngle2quatern = { |axis, angle|
	var halfAngle = angle/2;
	// return [q0, q1, q2, q3]
	[cos(halfang)] ++ (axis * sin(halfAngle).neg);
};


~calcPos = { |acc_imu, quat_imu, pos = ([0,0,0]), vel_prev = ([0,0,0]), timestep = 0.02|
	var acc_imu_enu, quat_imu_enu;
	var acc_world, vel, pos, moving;

	// *** Compute translational accelerations ***

	// --- Convert sensor acceleration: NED (Razor) -> ENU (x-IMU/Gait tracker) ---
	acc_imu_enu = [acc_imu[1], acc_imu[0], acc_imu[2].neg];

	// --- Convert sensor orientation: NED (Razor) -> ENU (x-IMU/Gait tracker) ---
	quat_imu_enu = ~quaternProd(
		~axisAngle2quatern.([sqrt2_2, sqrt2_2, 0], pi), // unit rotation vector: 45 degrees, 180 rotation
		quat_imu                                        // input quaternion (sensor orientation)
	);

	// --- Rotate body accelerations to Earth frame ---
	acc_world = ~quaternRotate.(acc_imu_enu, ~quaternConj.(quat_imu_enu));

	// --- Remove gravity from measurements    			// TODO check reasoning behind + rather than -
	acc_world[2] = acc_world[2] + 1;       	   			// TODO unnecessary due to velocity integral drift compensation ?

	// --- Detect movement from acceleration magnitude ---
	// 													// TODO: this could move before the acceleration
	// 													// coversions, but needs to handle gravity properly
	moving = acc_imu.squared.sum.sqrt > ~moveThresh;  	// TODO: use a cheaper norm?

	// --- Integrate twice to yield velocity ---
	if (moving) {
		vel = vel_prev + (acc_world * timestep); // Integrate acceleration to yield velocity
		pos = pos_prev + (vel * timestep); 		 // Integrate velocity to yield position
	} {
		vel = [0, 0, 0]; // force zero velocity when foot stationary
		pos = pos_prev;
	}

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

	[pos, vel, acc_world] // return
};

~resetPos = true; // zero out position
)

~scopes.do({|scp, i| scp.bounds_(-2,2) })
~scopes.do({|scp, i| scp.bounds_(\auto) })

(
/* Razor default "#TAQ=" Time, Acceleration, Quaternions */
// p_ previous frame values
var time_prev = 0, acc_prev = [0,0,0], quat_prev = [0,0,0];
var pos_prev = [0,0,0], vel_prev = [0,0,0];
var cnt = 1;
var storeCnt = 1;
var maxStoreFrames = 2000;
var toSecs = 1000.reciprocal;

~dataStore !? { ~dataStore = nil };
~dataStore = [];
~resetPos = true;

a.action_({
	|...args|
	var data, time, acc_imu, quat_imu;
	var pos, vel, accel_comp;

	try{       												// TODO: DEBUG, remove try
		// unpack data
		// args: [ "#TAQ,time,acc1,acc2,acc3,quat1,quat2,quat3,quat4" ]
		data = args[0].drop(6).split($,).asFloat; // drop "#TAQ="
		time = data[0];
		acc_imu  = data[1..3];
		quat_imu = data[4..];

		// store raw data for plotting/writing to file, etc.
		if (storeCnt < maxStoreFrames) {
			~dataStore = ~dataStore.add(data);              // TODO: DEBUG, remove data logging
		};

		#pos, vel, acc_world = ~calcPos.(
			acc_imu, quat_imu, pos_prev, vel_prev,
			(time - time_prev) * toSecs // sample period
		);

		// external reset control
		if (~resetPos) {
			pos_prev  = [0,0,0];
			vel_prev  = [0,0,0];
			~resetPos = false;
		} {
			pos_prev = pos;
			vel_prev = vel;
		};

		// write to bus
		~vBus !? {
			~vBus.set(acc_world);        					// TODO: write position
		};

		// post every n iterations                   		// TODO: DEBUG, remove posting
		cnt = cnt % 25; // post every 25 vals
		if (cnt == 0) {
			// accel_comp
			acc_world
			.round(0.001).collect({ |me| me.asString.padLeft(6) }).postln;
		};

		time_prev = time;
		acc_prev  = acc;
		quat_prev = quat;
		cnt = cnt+1;
		if (storeCnt < maxStoreFrames) {
			storeCnt = storeCnt+1;
		};
	};

})
)
a.action_({});         // disable serial responder
~resetPos = true;
~dataStore.size

// write recorded data to .csv file
(
var data = ~dataStore.collect({|me| me[1..3] });
data.shape;
p= data.flop.plot
)


~prev = Main.elapsedTime;
// time message frequency
a.action_({|msg| var now; /*msg.postln;*/ now = Main.elapsedTime; ~period = now - ~prev; ~prev = now})
~period.reciprocal

// post messages
a.action_({|...args| args.postln })
a.action_({|...args| args[0].postln })
a.action_({})



// write position to a bus
s.boot;
~vBus = CtkControl(3).play;
ControlPlotter(~vBus.bus, 3).start; // 3-in-1 scope


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
~vBus.set([1,2,3]/2)




(
// write a test file:
f = CSVFileWriter("~/Desktop/rccwTiltRrccwFwUp.csv".standardizePath);
~dataStore.do(f.writeLine(_));
// f.write(
// 	"Some,comma,delimited,items, in line 1
//
// 	and then, some more, with several commas,,,, in line 3
// 	"
// );
f.close;
)


p.plots.dump
p.minval_(-2).maxval_(2).refresh

p.plots.do(_.plotBounds_(-2,2))
p.plotBounds_(-2,2)
p.bounds_(-2,2).refresh


Slop

~time
~acc
~quat
~time_prev
~acc_prev
~quat_prev

(
var pos, vel;
#pos, vel = ~calcPos.(~acc, ~quat, ~pos, ~vel, ~time-~time_prev/1000);
~pos = pos;
~vel = vel;
)

	//accXYZ, quat, pos = ([0,0,0]), vel = ([0,0,0]), timestep = 0.02|


SerialPort.cleanupAll
a.free
