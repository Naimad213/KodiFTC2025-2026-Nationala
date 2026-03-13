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
        // MAKE SURE THESE MATCH YOUR PHYSICAL HUB MOUNTING
        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.DOWN
        );

        imu.initialize(new IMU.Parameters(orientation));
        reset();
    }

    public void invertGyro() {
        multiplier *= -1;
    }

    @Override
    public double getHeading() {
        // Returns normalized heading [-180, 180] relative to offset
        double heading = getAbsoluteHeading() - offset;
        return AngleUnit.normalizeDegrees(heading);
    }

    public double getAngle() {
        // Returns 0-360 heading
        return AngleUnit.normalizeDegrees(getAbsoluteHeading() - offset) + 180;
    }

    @Override
    public double getAbsoluteHeading() {
        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        return angles.getYaw(AngleUnit.DEGREES) * multiplier;
    }

    @Override
    public void reset() {
        offset = getAbsoluteHeading();
    }

    @Override
    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    // Boilerplate for GyroEx
    @Override public double[] getAngles() { return new double[]{0, 0, 0}; }
    @Override public void disable() { imu.close(); }
    @Override public String getDeviceType() { return "REV Internal IMU"; }
}