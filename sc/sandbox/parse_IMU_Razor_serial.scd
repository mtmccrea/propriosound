/* RAZOR IMU - Serial */
// sloppy ATM...

SerialPort.listDevices

a = ArduinoSMS("/dev/tty.usbmodem14101", 57600);

~prev = Main.elapsedTime;
// time message frequency
a.action_({|msg| var now; /*msg.postln;*/ now = Main.elapsedTime; ~period = now - ~prev; ~prev = now})
// post messages
a.action_({|msg| msg.postln })
(
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