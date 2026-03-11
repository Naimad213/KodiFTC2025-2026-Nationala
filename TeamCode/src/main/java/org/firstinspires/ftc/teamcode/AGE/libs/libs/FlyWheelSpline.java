package org.firstinspires.ftc.teamcode.AGE.libs.libs;

import com.qualcomm.robotcore.util.Range;

import org.apache.commons.math3.analysis.interpolation.SplineInterpolator;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;

public class FlyWheelSpline {

    public PolynomialSplineFunction splineCurve;

    public double minDistance = 0;
    public double maxDistance = 75;

    double minRPM = 0;
    double maxRPM= 1450;
    /// de la mic la mare
    private final double[] TESTED_DISTANCES = { minDistance,28, 30.0, 45, 50, 60.0, 70 , maxDistance};
    private final double[] TESTED_RPMS =      { minRPM, 1200,1200, 1400, 1400, 1400, 1400 , maxRPM };

    public FlyWheelSpline() {
        SplineInterpolator interpolator = new SplineInterpolator();
        splineCurve = interpolator.interpolate(TESTED_DISTANCES, TESTED_RPMS);
    }

    public double getTargetRPM(double currentDistance) {
        double rpm = Math.max(minDistance,Math.min(maxDistance , currentDistance));
        return splineCurve.value(rpm);
    }
}