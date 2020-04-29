/* Plotting positions in 3D
*
* Author: Michael McCrea, 2020
*/

(
~historyLength = 10;
~pv = PointView();
{
	~pv.axisScale = 1;
	~cube = [
		1, 1, 1,
		1, 1, -1,
		1, -1, -1,
		1, -1, 1,
		-1, 1, 1,
		-1, 1, -1,
		-1, -1, -1,
		-1, -1, 1,
	].clump(3).collect(_.asCartesian);
	~pv.points_(~cube
		++ (4.collect{ [0,0,0].asCartesian } ! ~historyLength).flat, // add 4 default hand positions (real, clipped)
		resetConnections: false
	);
	// cube vertices are clear
	~pv.pointColors = (Color.clear!8)
	++ (
		[
		Color.blue, Color.red,
		Color.blue.alpha_(0.2), Color.red.alpha_(0.2)
	] ! ~historyLength
	).flop.flat;
	~pv.showIndices = false;
	// outline the bounding cube
	~pv.connections_([
		[0,1,2,3,0], [4,5,6,7,4],
		[0,4], [1,5], [3,7], [2,6]
	]);
	~pv.front;
	{ ~pv.bounds = Rect(62.0, 45.0, 358.0, 305.0) }.defer(0.5)
}.defer(0.3);

) // END load PointView

// Update hand positions in PointView
(
~rate = 8;
~updateVis !? { ~updateVis.stop };
~updateVis = fork({
	var pospnts = [0,0,0].asCartesian ! ~historyLength ! 2;
	var clippospnts = [0,0,0].asCartesian ! ~historyLength ! 2;
	var thPhRh, cart;
	var maxCm = 90; // max centimeter distance to normalize display
	var norm = maxCm.reciprocal;
	var wrPos = 0;
	1000.do {
		// collect position points
		[~posBusL, ~posBusR].do{ |bus, i|
			bus !? {
				pospnts[i][wrPos] = 3.collect({ |j|
					bus.getSynchronous(j) * norm // normalize to maxCm radius
				}).asCartesian;
			}
		};
		// collect (filtered) direction points
		[~dirBusL, ~dirBusR].do{ |bus, i|
			bus !? {
				thPhRh = 3.collect({ |j|
					bus.getSynchronous(j) // radians
				});
				cart = Spherical(
				thPhRh[2] * norm, thPhRh[0], thPhRh[1]).asCartesian;
				clippospnts[i][wrPos] = cart;
			}
		};

		// update pointview
		~pv.points_(~cube ++ pospnts.flat ++ clippospnts.flat,
			resetConnections: false);
		~rate.reciprocal.wait;
		wrPos = (wrPos+1) % ~historyLength;
	}
}, AppClock);
)

~positionSynths[0].clipLoTheta_(20.degrad).clipHiTheta_(25.degrad)
~positionSynths[1].clipLoTheta_(-25.degrad).clipHiTheta_(20.degrad)
~positionSynths[0..1].do({|synth| synth.clipLoPhi_(-45.degrad).clipHiPhi_(45.degrad) })

( // update colors
~pv.pointColors = (Color.clear!8)
++ (
	[
		Color.blue, Color.red,
		Color.blue.alpha_(0.2), Color.red.alpha_(0.2)
	] ! ~historyLength
).flop.flat;
)

~updateVis.stop // stop updating


// dummy busses to test with
(
// if ~posBusL and ~posBusR aren't yet allocated
s.waitForBoot({
	~posBusL !? { ~posBusL.free };
	~posBusL = CtkControl(3).play; // for allocating the consecutive busses
	~posBusR !? { ~posBusR.free };
	~posBusR = CtkControl(3).play;
	~dirBusL !? { ~dirBusL.free };
	~dirBusL = CtkControl(3).play;
	~dirBusR !? { ~dirBusR.free };
	~dirBusR = CtkControl(3).play;
	s.sync;
});
)

(
// map "dummy signals onto the ~posBusL/R
s.waitForBoot({
	~dummies !? { ~dummies.do(_.free) };
	~dummies = [];

	// left hand position busses
	3.do{ |i|
		~dummies= ~dummies.add(
			CtkControl.lfo(LFNoise2, 2.reciprocal,
				low: -10, high: 70,   // centimeters
				bus: ~posBusL.index+i // overlay on ~posBusL
			).play
		)
	};

	s.sync;
	// right hand position busses, in cm
	[[-10, 70], [-90, 0],[-10, 70]].do{ |range, i|
		~dummies= ~dummies.add(
			CtkControl.lfo(LFNoise2, 2.reciprocal,
				low: range[0], high: range[1], // centimeters
				bus: ~posBusR.index+i // overlay on ~posBusL
			).play
		)
	};
});
)

ControlPlotter(~posBusL.bus, 3).start; // 3-in-1 scope
ControlPlotter(~posBusR.bus, 3).start; // 3-in-1 scope

ControlPlotter(~dirBusL.bus, 2, overlay: false).start; // 3-in-1 scope
ControlPlotter(~dirBusR.bus, 2, overlay: false).start; // 3-in-1 scope



/* SCRATCH */


~dummies.do{ |bus| bus.high_(90) }
ControlPlotter(~posBusL.bus, 3).start; // 3-in-1 scope
// cleanup
(~dummies ++ ~posBusL).do(_.free); ~dummies = ~posBusL = nil


// test dummy data in PointView
(
var pnt = 3.collect{rrand(-0.5,0.5)};
var step = 3.collect{rrand(-1,1.0) * 0.08};

f = Routine.run({
	2000.do{
		step = 3.collect{rrand(-1,1.0)*0.03};
		pnt = pnt + step;
		~pv.points_(~cube ++ pnt.asCartesian, resetConnections: false);
		0.1.wait;
	};
}, clock: AppClock);
)

f.stop