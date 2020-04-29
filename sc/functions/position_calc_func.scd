var sqrt2_2 = sqrt(2)/2; // precompute
var qConj, qProd, qRotate, axisAngle2q, calcPos;
var ned2enuQuat, xyRot45Quat;
var moveCnt = 0;

~moveThresh = ~moveThresh ?? 0.15;    // ensure these vars are initialized
~postAccMag = ~postAccMag ?? false;
~postEvery = ~postEvery ?? 50;
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

~calcPos = { |acc_imu, quat_imu, pos_prev = ([0,0,0]), vel_prev = ([0,0,0]), timestep = 0.01|
	var acc_imu_enu, quat_imu_enu;
	var acc_world, vel, pos, moving, accmag;
	var postNow;

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
	acc_world = acc_world * [-1,1,-1]; // to ambi coords
	// --- Detect movement from acceleration magnitude ---
	accmag = acc_world.squared.sum.sqrt; // TODO: use a cheaper norm? rm .sqrt

	postNow = ((moveCnt % ~postMoveEvery) == 0);

	if(~moveMeter.notNil and: postNow, { {~moveMeter.value = accmag}.defer });
	moving = accmag > ~moveThresh;
	// 													// TODO: the threshold test could move before the acceleration
	// 													// coversions, but needs to handle gravity properly

	// --- Integrate twice to yield position ---
	if (moving) {
		vel = vel_prev + (acc_world * timestep); // integrate acceleration to yield velocity
		pos = pos_prev + (vel * timestep); 		 // integrate velocity to yield position
		if (~postAccMag and: postNow, { 1.post });
	} {
		vel = [0, 0, 0]; // force zero velocity when foot stationary
		pos = pos_prev;
	};

	// post useful values
	if (~postAccMag and: {moveCnt == 0}) {
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

// // Debug: for external access/testing
// // cleanup vars first
// ~quaternConj = ~quaternProd = ~quaternRotate = ~axisAngle2quatern = ~calcPos = nil;
// ~quaternConj = qConj;
// ~quaternProd = qProd;
// ~quaternRotate = qRotate;
// ~axisAngle2quatern = axisAngle2q;
// ~calcPos = calcPos;