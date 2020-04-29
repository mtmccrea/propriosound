/* RAZOR IMU - Serial
* Absolute position tracking.
*
* Position algorithm and quaternion functions ported and adapted from:
* https://github.com/xioTechnologies/Gait-Tracking-With-x-IMU
*
* Author: Michael McCrea, 2020
*/

/* load position tracking functions */
(
~projectPath = PathName(
	"/Users/dyne/Documents/Courses/Emergent User interfaces/"
);
[
	"sc/functions/position_calc_func",     // calc funcs before listener
	"sc/functions/position_listener_func",
	"sc/functions/position_viz_func",
	"sc/run/propriosound_settings", 		// settings before server
	"sc/run/propriosound_server",
].do({ |name| this.executeFile((~projectPath +/+ format("%.scd", name)).fullPath) });

)
(
s.waitForBoot({
	[ ~posBusL, ~dirBusL, ~posSynthL,
		~posBusR, ~dirBusR, ~posSynthR
	].do{ |me| me !? {me.free} };
	~posBusL = CtkControl(3).play;
	~dirBusL = CtkControl(3).play;
	~posBusR = CtkControl(3).play;
	~dirBusR = CtkControl(3).play;
	~inGrp   = CtkGroup.play();
	s.sync;

	~posSynthL = ~synthdefs[\pos2dir].note(addAction: \tail, target: ~inGrp)
	.inbus_(~posBusL).outbus_(~dirBusL)
	.clipLoTheta_(-15.degrad).clipHiTheta_(85.degrad) // clip the direction
	.clipLoPhi_(-45.degrad).clipHiPhi_(45.degrad)
	.play;
	~posSynthR = ~synthdefs[\pos2dir].note(addAction: \tail, target: ~inGrp)
	.inbus_(~posBusR).outbus_(~dirBusR)
	.clipLoTheta_(-85.degrad).clipHiTheta_(-15.degrad) // clip the direction
	.clipLoPhi_(-45.degrad).clipHiPhi_(45.degrad)
	.play;

	[~posBusL, ~posBusR].do{ |bus, i|
		var cp = ControlPlotter(bus.bus, 3).start; // 3-in-1 scope
		cp.plotColors = [Color.blue, Color.red, Color.green];
		cp.mon.plotter.parent.bounds = Rect(0.0 + (270*i), 0.0, 357, 427);
		cp.mon.plotter.parent.view.children[0].bounds = cp.mon.plotter.parent.bounds.insetBy(5).left_(5);
		// ~posBusL.set([1,2,3]/2)
	};

	~setMoveThresh.(0.08);  // zero-velocity movement threshold
})
)


SerialPort.listDevices
SerialPort.closeAll // clean up Serial

~posListenerL = ArduinoSMS("/dev/tty.usbmodem14401", 9600); // Razor position firmware default
~posListenerR = ArduinoSMS("/dev/tty.usbmodem14301", 9600); // Razor position firmware default

[~posListenerL, ~posListenerR].do{|me| me !? {me.close} };

// test: write position to a bus
/* initialize position serial listener actions */
~posListenerL.action_(~genListenerFunc.(id: 0)) // left, id: 0
~posListenerR.action_(~genListenerFunc.(id: 1)) // left, id: 0

// graphic visualization of position
~pv = ~visualizePosition.(historyLen: 10, updateRate: 10);

(
~postAccMag = true;		// post acceleration magnitude to set ~moveThresh
~postEvery = 25;		// post acceleration data every N messages
~postMoveEvery = 3;		// post movement status every M messages
)
~postAccMag = false;

~resetPosL = ~resetPosR = true;	// reset positions

~setMoveThresh.(0.03);  // zero-velocity movement threshold

~moveMeter.spec_([0,1,3].asSpec)
c = ControlSpec(warp: -6)
c.map(0.3)

~posBusL.set([30,40,45])


/* quick resets */
~posListenerL.action_({|msg| msg.postln});
~posListenerL.action_({});  	// disable left serial responder


/* position -> direction */

// creat posDir synth
// scope the control bus
// visualize in cube



~positionSynths[0].clipLoTheta_(20.degrad).clipHiTheta_(25.degrad)
~positionSynths[1].clipLoTheta_(-25.degrad).clipHiTheta_(20.degrad)
~positionSynths[0..1].do({|synth| synth.clipLoPhi_(-45.degrad).clipHiPhi_(45.degrad) })



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





/* Debug: standalone serial listener, position data recording */

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



// test ~calcPos function
(
var pos, vel;
#pos, vel = ~calcPos.(~acc, ~quat, ~pos, ~vel, ~time-~time_prev/1000);
~pos = pos;
~vel = vel;
)


// test tap counting
(
~posListenerL.action_({
	|msg|
	// msg.postln;
	var key, data;
	try{
		key = msg.keep(5);
		if (key == "#TAP=") { // TAP
			data = msg.drop(5).split($,).asFloat; // drop "#TAP="
			postf("TAP\t%\n", data.asFloat);
			~countTap.()
		}
	} {|err| "oops".postln }
});
)