/*
Algorithm ported from:
https://github.com/xioTechnologies/Gait-Tracking-With-x-IMU
*/

(
~quaternConj = {|q|
	q * [1,-1,-1,-1]
};

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
	var v0XYZ = ~quaternProd.(~quaternProd.(q, [0] ++ vec3), ~quaternConj.(q));
	v0XYZ[1..3];
};

~calcPos = { |accXYZ, quat, pos = ([0,0,0]), vel = ([0,0,0]), timestep = 0.02|
	var acc;

	// TEST: see if accel is correct
	accXYZ = [accXYZ[1], accXYZ[0], accXYZ[2].neg]; //  NED (Razor) -> ENU (x-IMU)

	// ---- Compute translational accelerations	----
	// Rotate body accelerations to Earth frame
	acc = ~quaternRotate.(accXYZ, ~quaternConj.(quat));
	// acc = accXYZ;

	// % Remove gravity from measurements
	// acc = acc - [0,0,1];    // % unnecessary due to velocity integral drift compensation

	// % Convert acceleration measurements to m/s/s
	// acc = acc * 9.81;

	// % --------------------------------
	// % Compute translational velocities
	// acc[2] = acc[2] - 9.81;

	// % Integrate acceleration to yield velocity
	// vel = zeros(size(acc));
	// vel = prev_vel + (accel*timestep)
	vel = vel + (acc * timestep);

	// if(stationary(t) == 1)
	// vel(t,:) = [0 0 0];     % force zero velocity when foot stationary
	// end

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

	// % -------------------------------------------------------------------------
	// % Compute translational position: Integrate velocity to yield position
	pos = pos + (vel * timestep);

	[pos, vel, acc] // return
};
~resetPos = true;

)
~scopes.do({|scp, i| scp.bounds_(-2,2) })
~scopes.do({|scp, i| scp.bounds_(\auto) })

/* RAZOR IMU - Serial */
// sloppy ATM...

SerialPort.listDevices


// a = ArduinoSMS("/dev/tty.usbmodem14201", 57600); // Razor_AHRS
a = ArduinoSMS("/dev/tty.usbmodem14201", 9600); // Razor position firmware default
a.close


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
// ControlPlotter(~vBus.bus, 3).start; // 3-in-1 scope

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

(  /* Razor default "#TAQ=" Time, Acceleration, Quaternions */
// p_ previous frame values
var p_time = 0, p_acc = [0,0,0], p_quat = [0,0,0];
var p_pos = [0,0,0], p_vel = [0,0,0];
var cnt = 0;
var storeCnt = 1;
var maxStoreFrames = 2000;
var toSecs = 1000.reciprocal;
~dataStore !? {~dataStore=nil};
~dataStore = [];
~resetPos = true;

a.action_({|...args|
	var data, time, acc, quat;
	var pos, vel, accel_comp;

	// args = [ "#TAQ,time,acc1,acc2,acc3,quat1,quat2,quat3,quat4" ]
	try{ // TODO: remove try
		data = args[0].drop(6).split($,).asFloat; // drop "#TAQ="
		// unpack data
		time = data[0];
		acc  = data[1..3];
		quat = data[4..];

		// store raw data for plotting/writing to file, etc.
		if (storeCnt < maxStoreFrames) {
			~dataStore = ~dataStore.add(data);
		};

		// postf("%\n\tA: %\n\tQ: %\n", time, acc, quat);
		// postf("fs = %\n", ((time-p_time) / 1000).reciprocal.round);
		// time.postln;
		// postf("\t%\n", p_time);

		// if (~reset) {
		// };

		#pos, vel, accel_comp = ~calcPos.(acc, quat, p_pos, p_vel, time-p_time * toSecs);


		if (~resetPos) {
			p_pos = [0,0,0];
			p_vel = [0,0,0];
			~resetPos = false
		} {
			p_pos = pos;
			p_vel = vel;
		};

		// vel.round(0.001).collect({|me|me.asString.padLeft(6)}).postln;
		// ~vBus.set(vel.round(0.001));

		// write to bus / plot
		if (cnt % 25 == 0) {
			// accel_comp.round(0.001).collect({|me|me.asString.padLeft(6)}).postln;
			acc.round(0.001).collect({|me|me.asString.padLeft(6)}).postln;
			cnt = 0;
		};
		~vBus.set(acc.round(0.001));
		// ~vBus.set(accel_comp.round(0.001));

		// ~time = time;
		// ~acc = acc;
		// ~quat = quat;
		// ~p_time = p_time;
		// ~p_acc = p_acc;
		// ~p_quat = p_quat;

		p_time = time;
		p_acc  = acc;
		p_quat = quat;
		cnt = cnt+1;
		if (storeCnt < maxStoreFrames) {
			storeCnt = storeCnt+1;
		};
	};

})
)
a.action_({})
~resetPos = true;
~dataStore.size
a.action_({|msg|  })


~dataStore.size

~dataStore[0].size

(
var data = ~dataStore.collect({|me| me[1..3] });
data.shape;
p= data.flop.plot
)


~dataStore.shape

csv

(
// write a test file:
f = CSVFileWriter("~/Desktop/CSVReadTest.csv".standardizePath);
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
Plot

Slop

~time
~acc
~quat
~p_time
~p_acc
~p_quat

(
var pos, vel;
#pos, vel = ~calcPos.(~acc, ~quat, ~pos, ~vel, ~time-~p_time/1000);
~pos = pos;
~vel = vel;
)

	//accXYZ, quat, pos = ([0,0,0]), vel = ([0,0,0]), timestep = 0.02|


SerialPort.cleanupAll
a.free
