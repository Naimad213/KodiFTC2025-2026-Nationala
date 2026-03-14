package org.firstinspires.ftc.teamcode.AGE.libs.libs;

import com.qualcomm.robotcore.util.Range;

import org.apache.commons.math3.analysis.interpolation.SplineInterpolator;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;

public class FlyWheelSpline {

    public PolynomialSplineFunction splineCurve;

    public double minDistance = 0;
    public double maxDistance = 130;

    double minRPM = 150;
    double maxRPM= 1500;
    /// de la mic la mare
    private final double[] TESTED_DISTANCES = { minDistance,71, 92, 99, 104, 116, 124 , maxDistance};
    private final double[] TESTED_RPMS =      { minRPM, 1350,1445, 1470, 1475, 1450, 1500 , maxRPM };

    public FlyWheelSpline() {
        SplineInterpolator interpolator = new SplineInterpolator();
        splineCurve = interpolator.interpolate(TESTED_DISTANCES, TESTED_RPMS);
    }

    public double getTargetRPM(double currentDistance) {
        double rpm = Math.max(minDistance,Math.min(maxDistance , currentDistance));
        return splineCurve.value(rpm);
    }
}