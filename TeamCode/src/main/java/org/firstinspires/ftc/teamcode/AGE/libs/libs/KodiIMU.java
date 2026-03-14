package org.firstinspires.ftc.teamcode.AGE.libs.libs;

import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.GyroEx;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class KodiIMU extends GyroEx {

    public IMU imu;
    private double offset = 0;
    private int multiplier = 1;

    public KodiIMU(HardwareMap hw) {
        imu = hw.get(IMU.class, "imu");
    }

    @Override
    public void init() {
        /*
         * FIX: Incorrect orientation causes the "crazy" behavior at 90 degrees.
         * Most robots have the Hub mounted flat.
         * If your USB is pointing forward, use UP and FORWARD.
         * To "invert X and Y" at the hardware level, you can flip UsbFacingDirection to BACKWARD.
         */
        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        );

        imu.initialize(new IMU.Parameters(orientation));
        reset();
    }

    public void invertGyro() {
        multiplier *= -1;
    }

    @Override
    public double getHeading() {
        // Returns normalized heading [-180, 180]
        return AngleUnit.normalizeDegrees(getAbsoluteHeading() - offset);
    }

    public double getAngle() {
        // Returns 0-360 heading for field-centric drive
        double angle = (getAbsoluteHeading() - offset) % 360;
        return (angle < 0) ? angle + 360 : angle;
    }

    @Override
    public double getAbsoluteHeading() {
        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        // Yaw is rotation around the vertical axis
        return angles.getYaw(AngleUnit.DEGREES) * multiplier;
    }

    @Override
    public void reset() {
        // Hardware reset: This makes the current heading "0"
        imu.resetYaw();
        offset = 0;
    }

    @Override
    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    @Override public double[] getAngles() { return new double[]{0, 0, 0}; }
    @Override public void disable() { imu.close(); }
    @Override public String getDeviceType() { return "REV Internal IMU"; }
}