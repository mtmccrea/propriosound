/* RAZOR IMU - Serial */
// sloppy ATM...

SerialPort.listDevices
SerialPort.cleanupAll
// a = ArduinoSMS("/dev/tty.usbmodem14201", 57600); // Razor_AHRS
a = ArduinoSMS("/dev/tty.usbmodem14201", 115200); // Razor default
a.close
// a.parser.parse

// a.dispatch
a.free

~prev = Main.elapsedTime;

// time message frequency
a.action_({|msg| var now; /*msg.postln;*/ now = Main.elapsedTime; ~period = now - ~prev; ~prev = now})
// post messages
a.action_({|...args| args.postln })
a.action_({|...args| args[0].postln })
a.action_({|...args| args[0].class.postln })
a.action_({|msg| msg.postln })
a.action_({|msg|  })


(  /* Razor default */
	var p_time=0, p_acc=[0,0,0], p_quat=[0,0,0];
a.action_({|...args|
	var data, time, acc, quat;

	// args = [ "#TAQ,time,acc1,acc2,acc3,quat1,quat2,quat3,quat4" ]
	try{ // TODO: remove try
		data = args[0].drop(6).split($,).asFloat; // drop "#TAQ="
		// unpack data
		time = data[0];
		acc  = data[1..3];
		quat = data[4..];

		// postf("%\n\tA: %\n\tQ: %\n", time, acc, quat);
		// postf("fs = %\n", ((time-p_time) / 1000).reciprocal.round);
		// time.postln;
		// postf("\t%\n", p_time);

		p_time = time;
		p_acc  = acc;
		p_quat = quat;
	};

	// try{
	// 	data = args[0].drop(4).split($,); // drop "#Qx="
	// 	data.asFloat[0].postln;
	// };
	// try{ args[idx].post; args[idx].class.postln };
	// try{ args.do{|me,i|me.class.postln} };
	// try{ args.split.size.postln } ;
	// 41810, 0.00, 0.07, -0.99, 0.79, 0.43, 0.06, 0.0340, 0.9530, 0.3006, 0.0135
	// data.do{ |val| (val++"\t").post };
	// "".postln;
})
)

(  /* Razor_AHRS */
a.action_({|msg|
	var ypr = msg.drop(5).split($,); // drop "#YPR="
	// x = ypr.postln;
	ypr.do{ |val| (val++"\n\t").post };
	"".postln;
})
)
// reset message action
a.action_({})

~period.reciprocal

a.close
x[0].split($,).postlns