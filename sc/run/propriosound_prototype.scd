/*
Main process for propriosound.

Author: Michael McCrea, 2020
*/

~projectPath = PathName(
	"/Users/dyne/Documents/Courses/CS-E4200 - Emergent User interfaces/"
);

// load settings
this.executeFile((~projectPath +/+ "sc/propriosound_settings.scd").fullPath);

// load synths
this.executeFile((~projectPath +/+ "sc/propriosound_server.scd").fullPath);


/* UTILS */

// inspect available synthdefs
~synthdefs.synthdefs.collect(_.name).postlns;

/* RUNTIME */

// GROUPS
fork {
	block { |break|
		~grps !? { ~grps.do(_.free) }
		~decGrp = CtkGroup(addAction: \tail, target: 1).play; s.sync;
		~encGrp = CtkGroup(addAction: \before, target: ~decGrp).play;  s.sync;
		~inGrp = CtkGroup(addAction: \before, target: ~encGrp).play;  s.sync;
		~grps = [~decGrp, ~encGrp, ~inGrp];

		// BUSSES
		~inputBus !? { ~inputBus.free }

		// SYNTHS
		if (~useTestInput) {
			if (~testfileIdxs.isKindOf(Symbol)) {
				switch(~testfileIdxs
					'rand', {
						~inputBus = CtkAudio.play(~hwInChans.size);
						s.sync;
						~testBuffers.scramble; // randomize buffer order

						~inSynths = ~hwInChans.size.collect{ |i|
							~synthdefs[\sfPlayback].new(addAction: \head, target: ~inGrp)
							.outbus_(~inputBus.index+i)
							.bufnum_(~testBuffers[i])
							.amp_(1.0)
						}
					}, { "Unrecognized input channels!".warn; break.() }
				)

			} { // ~testfileIdxs is an array of indices into ~testBuffers[]
				~inputBus = CtkAudio.play(~testfileIdxs.size);
				s.sync;
				~inSynths = ~testfileIdxs.collect{ |bufidx, i|
					~synthdefs[\sfPlayback].new(addAction: \head, target: ~inGrp)
					.outbus_(~inputBus.index+i)
					.bufnum_(~testBuffers[bufidx])
					.amp_(1.0)
				}
			}
		} {
			~inputBus = CtkAudio.play(~hwInChans.size);
			s.sync;
			~inSynths = ~hwInChans.collect{ |inchan, i|
				~synthdefs[\hwInput].new(addAction: \head, target: ~inGrp)
				.inbusnum_(inchan).outbus_(~inputBus.index+i).amp_(1.0)
			}
		};


	} // end block
} // end fork