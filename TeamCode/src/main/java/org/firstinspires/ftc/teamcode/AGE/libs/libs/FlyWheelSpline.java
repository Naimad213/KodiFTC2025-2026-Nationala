package org.firstinspires.ftc.teamcode.AGE.libs.libs;

import com.qualcomm.robotcore.util.Range;

import org.apache.commons.math3.analysis.interpolation.SplineInterpolator;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;

public class FlyWheelSpline {

    public PolynomialSplineFunction splineCurve;

    public double minDistance = 0;
    public double maxDistance = 80;

    double minRPM = 0;
    double maxRPM= 3000;
    /// de la mic la mare
    private final double[] TESTED_DISTANCES = { minDistance, 24.0, 36.0, 48.0, 60.0, 72.0 , maxDistance};
    private final double[] TESTED_RPMS =      { minRPM,1500.0, 1850.0, 2100.0, 2450.0, 2800.0 , maxRPM };

    public FlyWheelSpline() {
        SplineInterpolator interpolator = new SplineInterpolator();
        splineCurve = interpolator.interpolate(TESTED_DISTANCES, TESTED_RPMS);
    }

    public double getTargetRPM(double currentDistance) {
        double rpm = Math.max(minDistance,Math.min(maxDistance , currentDistance));
        return splineCurve.value(rpm);
    }
}