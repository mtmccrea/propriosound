/* RAZOR IMU - Serial
* AHRS head orientation tracking.
*
* Author: Michael McCrea, 2020
*/

SerialPort.listDevices

~orienListener = ArduinoSMS("/dev/tty.usbmodem14201", 57600); // Razor_AHRS    // TODO: confirm baud rate
~orienListener.close


/* **** Helper Functions **** */
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~

(
~resetOrientation = true;

~orienListener.action_({
	|...args|
	var ypr;

	try{       												// TODO: DEBUG, remove try
		// unpack data
		// args: [ "#YPR= ... " ]
		ypr = msg.drop(5).split($,); // drop "#YPR="
		ypr.do{ |val| (val++"\n\t").post };
		"".postln;

		~headBus !? { ~headBus.set(ypr) };
	};

})
)

~orienListener.action_({});  // disable serial responder
~resetPos = true;


/* Utils */

// time message frequency
~prev = Main.elapsedTime;
~orienListener.action_({|msg| var now; /*msg.postln;*/ now = Main.elapsedTime; ~period = now - ~prev; ~prev = now})
~period.reciprocal


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

~scopes.do({|scp, i| scp.bounds_(-2,2) })
~scopes.do({|scp, i| scp.bounds_(\auto) })

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
