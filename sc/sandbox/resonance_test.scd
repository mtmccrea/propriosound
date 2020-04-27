(
d = CtkSynthDef('dynKlank', {
	arg
		inoutbus,
	    freqs (#[100, 200, 300, 400]),
	    amps (#[1, 0.3, 0.2, 0.05]),
	    rings (#[1, 1, 1, 2]);
	var in, reson;
	// in = In.ar(inoutbus, 1);
	in = LPF.ar(Crackle.ar( MouseX.kr(1,2), 0.001 ), 1800);
	reson = DynKlank.ar(`[freqs, Lag.kr(amps, 2.8), rings], in);
	ReplaceOut.ar(inoutbus, reson)
})
)

(
x = d.note.inoutbus_(0).play;
y = d.note.inoutbus_(1).play;
)

(
var amplo = -4, amphi = -9;
x.amps_(Array.exprand(4, amplo.dbamp, amphi.dbamp).postln);
y.amps_(Array.exprand(4, amplo.dbamp, amphi.dbamp).postln);
)

(
var ring = 0.2;
x.rings_(4.collect{ring * rrand(0.7, 1.3)});
y.rings_(4.collect{ring * rrand(0.7, 1.3)});
)

// change amplitudes of partials ... some changes sound like attacks
y.setn(\amps, Array.exprand(4, 0.01, 1));
[a,x].do{|me|me.setn(\amps, Array.exprand(4, 0.01, 1)) }

(
var amplo, amphi, waitMul;
r !? {r.stop};
q !? {q.stop};
amplo = 4;
amphi = 22;
waitMul = 3.2;

r = Routine.run({
	inf.do{
		y.amps_(Array.exprand(4, amplo.dbamp, amphi.dbamp).reverse);
		(rrand(0.5, 0.1)* waitMul).wait;
	}
});
q = Routine.run({
	inf.do{
		x.amps_(Array.exprand(4, amplo.dbamp, amphi.dbamp).reverse);
		(rrand(0.5, 0.1)*waitMul).wait;
	}
});
)
r.stop; q.stop;

(
var kind, root, scale, ring;
kind =
// 'major'
// 'minor'
// 'harm-minor'
// 'melo-minor'//only up, use 'minor' for down
// 'blues'
// 'blues-major'
// 'pentatonic'
// 'chromatic'
// 'quartertones'
// //// tuning tables:
// 'just'
'fifth'
// //// tuning tables from Apple Logic Pro:
// 'pythagorean'
// 'werckmeister'
// 'indian'
// 'arabic'
// //// tuned scales:
// 'just-major'
// 'just-minor'
// 'fifth-major'
// 'fifth-minor'
;
root = "c5";
scale = Array.makeScaleMidi( root.notemidi, kind ).midicps;

x.freqs_(4.collect{scale.choose});
y.freqs_(4.collect{scale.choose});

ring = 2;
x.rings_(4.collect{ring * rrand(0.7, 1.3)});
y.rings_(4.collect{ring * rrand(0.7, 1.3)});
)



s.recChannels_(2)
s.recSampleFormat_("int16")
s.prepareForRecord
s.record
s.stopRecording
