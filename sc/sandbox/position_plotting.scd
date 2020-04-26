/* Plotting positions in 3D
*
* Author: Michael McCrea, 2020
*/

(
~pv = PointView().front;
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
	~pv.points_(~cube);
	~pv.pointColors = Color.gray;
	~pv.showIndices = false;
}.defer(0.3)
)

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

~pv.connections_([
	[0,1,2,3,0], [4,5,6,7,4],
	[0,4], [1,5], [3,7], [2,6]
]);
{~pv.highlightPoints((~cube.size..~pv.points.size-1))}.defer(0.5)
)

f.stop
