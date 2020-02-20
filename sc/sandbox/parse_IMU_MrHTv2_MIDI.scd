/* IEM HEADTRACKER - MIDI */

/* Init and connect MIDI */
(
MIDIClient.init;
MIDIIn.connectAll;

d = HTDispatcher.new(lowcc: 16, hicc: 48);

MIDIdef.cc(\quat,
	{| val, num, chan, src |
		// postf("val, num, chan, src\n\t% % % %\n", val, num, chan, src);
		// val.postln;
		d.setVal(num, val); // HTDispatcher posts internally
		defer{
			~rotaries !? {
				d.euler.do{ |e, i| ~rotaries[i].value_(raddeg(e)) };
			};
			~sliders !? {
				d.q.do{ |qi, i|
					~sliders[i].value_(qi);
					~slTxts[i].string_(
						qi.round(0.01).asString ++ " (" ++
						qi.round(0.01).linlin(0,1,-1,1).asString ++ ")"
					);
				};
			}
		};
	},
	// ccNum: 51,
	chan: 1,
	srcID: -1313038983
)

)

/* UI Monitor */
(
// load utility functions (UI)
var utilpath = "~/src/propriosound/sandbox/headtracker/utils.scd".standardizePath;
this.executeFile(utilpath);

~rotaries = ~makeRotaries.(3);
~sliders  = ~makeSliders.(4);
~slTxts   = ~makeSlTxts.(4);
~labels   = ["yaw", "pitch", "roll"].collect(StaticText().string_(_));

w = Window(bounds: Rect(0,0,325,550)).front;
w.layout_(
	HLayout(
		VLayout(*([~rotaries, ~labels].lace(6)++nil)),
		VLayout(*([~slTxts, ~sliders].lace(8)++nil))
	)
)
)

/* Inspect/adjust MIDI message dispatcher */

d.euler 	// current ypr value
d.q.postlns // post quaternion value

d.normalizeQuaternions = false; // NOTE: IEM SceneRotator plugin = false
d.invertQuaternions = false;	// flips yaw direction (?)

// limit refresh rate of UI
(
~rotaries.do{ |r,i|
	r.limitRefresh = true;
	r.maxRefreshRate = 15;
}
)


/* Cleanup */

MIDIIn.disconnectAll;
MIDIdef(\quat).free;
MIDIdef.freeAll;


/* Debug */

//MIDI
MIDIClient.sources;
MIDIFunc.trace(true)
MIDIFunc.trace(false)

// debug: just post an individual midi cc's value
MIDIdef.cc(\qa,
	{| val, num, chan, src | val.postln},
	ccNum: 51,
	chan: 1,
	srcID: -1313038983
)

d.dump