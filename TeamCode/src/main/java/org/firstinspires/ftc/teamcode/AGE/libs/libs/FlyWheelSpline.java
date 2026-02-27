package org.firstinspires.ftc.teamcode.AGE.libs.libs;

import org.apache.commons.math3.analysis.interpolation.SplineInterpolator;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;

public class FlyWheelSpline {

    public PolynomialSplineFunction splineCurve;
    /// de la mic la mare
    private final double[] TESTED_DISTANCES = { 24.0, 36.0, 48.0, 60.0, 72.0 };
    private final double[] TESTED_RPMS =      { 1500.0, 1850.0, 2100.0, 2450.0, 2800.0 };

    public FlyWheelSpline() {
        SplineInterpolator interpolator = new SplineInterpolator();
        splineCurve = interpolator.interpolate(TESTED_DISTANCES, TESTED_RPMS);
    }

    public double getTargetRPM(double currentDistance) {

        return splineCurve.value(currentDistance);
    }
}