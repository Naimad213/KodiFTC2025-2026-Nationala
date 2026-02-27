package org.firstinspires.ftc.teamcode.AGE.libs.libs;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class KodiVoltageSensor {

    VoltageSensor voltageSensor;

    private static final double voltajNominal =12.0;
    private static final double threshold=10.0;
    public double currentVoltage ;
     public MecanumDrive drive;

    public KodiVoltageSensor(HardwareMap hardwareMap) {
        voltageSensor = hardwareMap.voltageSensor.get("voltageSensor");
        currentVoltage = voltageSensor.getVoltage();
    }

    public void driveWithVoltageCompensation(MecanumDrive drive, double x, double y, double r, double theta) {
        double currentVoltage = voltageSensor.getVoltage();

        // 1. Handle low voltage or edge cases early
        if (currentVoltage < threshold) {
            drive.driveRobotCentric(x, y, r);
            return;
        }

        // 2. Calculate scale (e.g., if battery is 13V, scale is ~0.92; if 11V, scale is ~1.09)
        double voltageScale = voltajNominal / currentVoltage;

        // 3. Apply scale and clip to valid motor ranges [-1.0, 1.0]
        // Using a helper method or Math.max/min prevents unexpected behavior at high power
        double scaledX = clamp(x * voltageScale);
        double scaledY = clamp(y * voltageScale);
        double scaledR = clamp(r * voltageScale);

        drive.driveFieldCentric(scaledX, scaledY, scaledR ,theta);
    }

    private double clamp(double value) {
        return Math.max(-1.0, Math.min(1.0, value));
    }




}
