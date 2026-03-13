package org.firstinspires.ftc.teamcode.AGE.libs.libs;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

public class KodiVision {

    HardwareMap hardwareMap;
    public Telemetry telemetry;
    public IMU imu;

    public double kP = 0.05;
    public double kD = 0.12001;

    public double goalX = 0;
    public double lastError = 0;
    public double angleTolerance = 0.5;

    public final double MAX_POWER = 1;

    KodiPinPoint pinPoint;
    public final ElapsedTime t = new ElapsedTime();

    public KodiLimelight limelight;
    public LLResult result; // We will store the result here once per loop
    public String teamColor;

    final double LIMELIGHT_HEIGHT = 28;
    final double APRILTAG_HEIGHT = 74;
    final double LIMELIGHT_ANGLE = 20;

    public KodiVision(HardwareMap hardwareMap, KodiLimelight limelight, String teamColor, IMU imu) {
        this.hardwareMap = hardwareMap;
        this.limelight = limelight;
        this.teamColor = teamColor;
        this.imu = imu;

        if (teamColor.equals("RED")) {
            limelight.initRed(hardwareMap);
        } else {
            limelight.initBlue(hardwareMap);
        }
    }

    public void setKp(double newKp) { kP = newKp; }
    public void setKd(double newKd) { kD = newKd; }
    public void updateLimelight() {
        result = limelight.getResult();
    }



    public boolean isSeeingAprilTag(int targetId) {
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
            for (LLResultTypes.FiducialResult tag : tags) {
                if (tag.getFiducialId() == targetId) {
                    return true;
                }
            }
        }
        return false;
    }

    public double getDistance() {
        result= limelight.getResult();
        if (result != null && result.isValid()) {
            double ty = result.getTy();
            double angleToGoalRadians = Math.toRadians(LIMELIGHT_ANGLE + ty);
            if (angleToGoalRadians == 0) return 1e-6;
            return (APRILTAG_HEIGHT - LIMELIGHT_HEIGHT) / Math.tan(angleToGoalRadians);
        }
        return -1;
    }

    public double getLimelightRotationCorrection(boolean hasTarget) {

        if (!hasTarget || result == null || !result.isValid()) {
            lastError = 0;
            t.reset();
            return 0;
        }
        double deltaTime = t.seconds();
        double error = goalX - result.getTx();
        double pTerm = kP * error;
        double dTerm = 0;

        if (deltaTime > 0) {
            dTerm = kD * (error - lastError) / deltaTime;
        }
        double turnPower;
        if (Math.abs(error) < angleTolerance) {
            turnPower = 0;
        } else {
            turnPower = Range.clip(pTerm + dTerm, -MAX_POWER, MAX_POWER);
        }
        lastError = error;
        t.reset();
        return turnPower;
    }
    public Pose3D updateMegaTag2Pose(){
        if (imu == null || limelight == null) return null;
        double currentYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES).zRotationRate;
        limelight.updateRobotOrientation(currentYaw);
        updateLimelight();

        if (Math.abs(angularVelocity) > 720) {
            return null;
        }

        if (result != null && result.isValid()) {
            Pose3D mt2Pose = result.getBotpose_MT2();

            if (mt2Pose != null) {
                double x = mt2Pose.getPosition().x;
                double y = mt2Pose.getPosition().y;
                if (telemetry != null) {
                    telemetry.addData("MT2 Location:", "(" + x + ", " + y + ")");
                }
                return mt2Pose;
            }
        }
        return null;
    }


    public void killSwitch() {
        if(limelight != null) {
            limelight.stop();
        }
    }
}

