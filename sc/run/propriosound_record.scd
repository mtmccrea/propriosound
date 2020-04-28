/* RECORDING */

s.recChannels_(6)
s.recSampleFormat_("float")
// s.recSampleFormat_("int16")
s.prepareForRecord("~/recordings/".standardizePath ++ Date.getDate.stamp ++ ".wav", numChannels: 4)

(
var recTime = 2*60;
fork{
	s.record;
	recTime.wait;
	s.stopRecording;
}
)