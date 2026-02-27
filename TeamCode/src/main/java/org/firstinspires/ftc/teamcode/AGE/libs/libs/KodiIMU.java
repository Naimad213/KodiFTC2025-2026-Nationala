package org.firstinspires.ftc.teamcode.AGE.libs.libs;

import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.GyroEx;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class KodiIMU extends GyroEx {

    private IMU imu;

    /***
     * Heading relative to starting position
     */
    double globalHeading;

    /**
     * Heading relative to last offset
     */
    double relativeHeading;

    /**
     * Offset between global heading and relative heading
     */
    double offset;

    private int multiplier;

    private RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
    private RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;

    private RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

    /**
     * Create a new object for the built-in gyro/imu in the Rev Expansion Hub
     *
     * @param hw      Hardware map
     * @param imuName Name of sensor in configuration
     */

    double angle,lastAngle;

    public KodiIMU(HardwareMap hw, String imuName) {
        imu = hw.get(IMU.class, imuName);
        multiplier = 1;
    }

    /**
     * Create a new object for the built-in gyro/imu in the Rev Expansion Hub with the default configuration name of "imu"
     *
     * @param hw Hardware map
     */
    public KodiIMU(HardwareMap hw) {
        this(hw, "imu");
    }

    @Override
    public void init() {
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        reset();
        globalHeading = 0;
        relativeHeading = 0;
        offset = 0;
        lastAngle = 0;
    }

    public void invertGyro() {
        multiplier *= -1;
    }

//    @Override
//    public double getHeadingOLD() {
//        double currentAngle = getAbsoluteHeading() - offset;
//        currentAngle += 360 * (1 - Math.signum(Math.max(currentAngle,0)));
//        return currentAngle;
//    }

    @Override
    public double getHeading(){
        return getAbsoluteHeading() - offset;
    }

    public double getAngle() {
        // Obține Yaw-ul (rotația pe axa Z)
        double rawYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        // Dacă ai nevoie de 0-360 (pentru codul tău de path following):
        double theta360 = (rawYaw % 360.0 + 360.0) % 360.0;

        return theta360;
        // Sau returnează rawYaw direct dacă adaptezi matematica la -180/180
    }
    public double getHeadingOld() {
        double currentAngle = getAbsoluteHeading();
        double deltaAngle = currentAngle - lastAngle;
        // Detectăm tranzițiile de la 90 la -90 sau de la -90 la 90
        if (deltaAngle < -90) {
            // Tranziție de la 90 la -90
            deltaAngle += 180;
        } else if (deltaAngle > 90) {
            // Tranziție de la -90 la 90
            deltaAngle -= 180;
        }

        // Actualizăm unghiul global
        globalHeading += deltaAngle;

        // Actualizăm ultima citire
        lastAngle = currentAngle;

        // Convertim unghiul global în intervalul [0, 360]
        return (globalHeading % 360 + 360 - offset) % 360;
    }

    @Override
    public double getAbsoluteHeading() {
        return imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES).firstAngle * multiplier;
    }

    @Override
    public double[] getAngles() {
        return new double[0];
    }

    @Override
    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    @Override
    public void reset() {
        offset = getAbsoluteHeading();
    }

    public void zeroHeading() {
        imu.resetYaw();
    }

    @Override
    public void disable() {
        imu.close();
    }

    @Override
    public String getDeviceType() {
        return "";
    }
}
