/* utilities for RazorIMU testing */

/* UI functions */
~makeRotaries = { |howMany|
	howMany.collect{
		var r, setupRot, numRots = 8;

		r = RotaryView(bounds: Size(300, 300).asRect,
			spec: [-180, 180].asSpec,
			initVal: 0,
			startAngle: pi, sweepLength: 2pi,
			innerRadiusRatio: 0.3,
			outerRadiusRatio: 0.95
		); //.front;

		r.bipolar_(true).centerValue_(0); // bipolar center at 0dB

		// range
		r.range.fillColor_(Color.new(0.9,0.9,0.9));

		// assign bipolar colors
		r.level.strokeColor = { |p| if (p.val < 0) { Color.red } { Color.green } };
		r.level.fillColor = { |p| if (p.val < 0) { Color.red.alpha_(0.2) } { Color.green.alpha_(0.2) } };
		r.level.strokeType_(\outside).strokeWidth_(0.05);

		// handle
		r.handle.strokeColor_(Color.gray.alpha_(0.4)).anchor_(1).length_(1);

		// text
		r.text.fontSize_(0.15).align_(\center).color_(Color(*0.6!3));
		r.text.round(1);

		// ticks
		r.ticks.show_(true).majorColor_(Color.gray).majorWidth_(0.05).minorWidth_(0.02).capStyle_(\round);
		r.ticksEveryVal_(45, 1); // AtValues_([0], (-24, -21 .. 12));

		// refresh rate
		r.limitRefresh = true;
		r.maxRefreshRate = 15;

		// size
		r.minWidth_(75);
	}
};
~makeSliders =  { |howMany|
	howMany.collect{
		Slider().orientation_('horizontal').maxHeight_(20);
	}
};

~makeSlTxts = { |howMany|
	howMany.collect{
		StaticText().align_('center').string_(0.asString);
	}
};
