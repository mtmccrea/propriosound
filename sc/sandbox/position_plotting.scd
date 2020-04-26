/* Plotting positions in 3D
*
* Author: Michael McCrea, 2020
*/

(
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
	~pv.points_(
		~cube ++ 2.collect{ [0,0,0].asCartesian }, // add 2 default hand positions
		resetConnections: false
	);
	// cube vertices are clear
	~pv.pointColors = (Color.clear!8) ++ [Color.blue, Color.red];
	~pv.showIndices = false;
	// outline the bounding cube
	~pv.connections_([
		[0,1,2,3,0], [4,5,6,7,4],
		[0,4], [1,5], [3,7], [2,6]
	]);
	~pv.front
}.defer(0.3);


// highlight hand positions
// { ~pv.highlightPoints((~cube.size..~pv.points.size-1)) }.defer(0.5)

) // END load PointView

// Update hand positions in PointView
(
~rate = 10;
f = fork({
	1000.do {
		var pnts = [];
		[~posBusL, ~posBusR].do{ |bus|
			bus !? {
				pnts = pnts.add(
					3.collect({ |i|
						bus.getSynchronous(i) / 90 // normalize to 90cm radius
					}).asCartesian;
				)
			}
		};
		// update pointview
		~pv.points_(~cube ++ pnts, resetConnections: false);
		~rate.reciprocal.wait;
	}
}, AppClock);
)

f.stop // stop updating

// dummy busses to test with
(
// if ~posBusL and ~posBusR aren't yet allocated
s.waitForBoot({
	~posBusL !? { ~posBusL.free };
	~posBusL = CtkControl(3).play; // for allocating the consecutive busses
	~posBusR !? { ~posBusR.free };
	~posBusR = CtkControl(3).play; // for allocating the consecutive busses
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