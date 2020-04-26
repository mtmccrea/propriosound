s.options.numOutputBusChannels_(12);
s.boot;
{Pan2.ar(PinkNoise.ar(-9.dbamp), SinOsc.kr(4.reciprocal))}.play;
{PanAz.ar(10, PinkNoise.ar(-9.dbamp), SinOsc.kr(4.reciprocal))}.play;

s.meter
s.scope(2, 0)

s.meter