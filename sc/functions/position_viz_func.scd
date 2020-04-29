/* Function to generate PointView visualization of hand positions */

~visualizePosition = { |historyLen = 10, updateRate = 10|
	var pv ;       // Point View
	var updateVis; // update routine (fork)
	var rateRecip = updateRate.reciprocal; // update waittime
	var cube = [
		1, 1, 1, 1, 1, -1,
		1, -1, -1, 1, -1, 1,
		-1, 1, 1, -1, 1, -1,
		-1, -1, -1, -1, -1, 1,
	].clump(3).collect(_.asCartesian);
	// outline the bounding cube
	var cubeConns = [
		[0,1,2,3,0], [4,5,6,7,4],
		[0,4], [1,5], [3,7], [2,6]
	];
	// _point colors_
	// cube verts: clear; left position: blue; left direction: blue w/alpha
	// right position: red; right direction: red w/alpha
	var pColors = (Color.clear!8)
	++ ([
		// Color.blue, Color.red,
		Color.clear, Color.clear, // TEMP: "mute" abs pos for now
		Color.blue.alpha_(0.2), Color.red.alpha_(0.2) // clipped position
	] ! historyLen ).flop.flat;

	pv = PointView();
	defer ({
		pv.axisScale = 1;
		pv.points_(cube
			++ (4.collect{ [0,0,0].asCartesian } ! historyLen).flat, // add 4 default hand positions (real, clipped)
			resetConnections: false
		);
		// cube vertices are clear
		pv.pointColors = pColors;
		pv.showIndices = false;
		pv.connections_(cubeConns);

		pv.front;
		defer ({
			pv.bounds = Rect(62.0, 45.0, 358.0, 305.0);
			// view orientation
			pv
			.skewX_(0).skewY_(-0.95)
			.translateX_(0).translateY_(1.187)
			.originDist_(-0.5).eyeDist_(4)
			;
			pv.extrinsicRotation = true;
			pv.rotate_(0.95);
		}, 0.5);
		pv.onClose = {
			updateVis.stop; updateVis = nil;
		};
	}, 0.3);


	// Update hand positions in PointView

	updateVis = fork({
		var pospnts = [0,0,0].asCartesian ! historyLen ! 2;
		var clippospnts = [0,0,0].asCartesian ! historyLen ! 2;
		var thPhRh, cart;
		var maxCm = 90; // max centimeter distance to normalize display
		var norm = maxCm.reciprocal;
		var wrPos = 0;

		1.wait; // wait for PointView to load
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
						(thPhRh[2] * norm).clip(0.1, 1),
						thPhRh[0],
						thPhRh[1]
					).asCartesian;
					clippospnts[i][wrPos] = cart;
				}
			};

			// update pointview
			pv.points_(cube ++ pospnts.flat ++ clippospnts.flat,
				resetConnections: false);
			rateRecip.wait;
			wrPos = (wrPos+1) % historyLen;
		}
	}, AppClock);

	pv // return the view
};
