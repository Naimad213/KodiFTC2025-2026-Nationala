package org.firstinspires.ftc.teamcode.AGE.libs.libs;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

public class KodiVision {

    HardwareMap hardwareMap;
    public Telemetry telemetry;

    public double kP = 0.05;
    public double kD = 0.12001;

    public double goalX = 0;
    public double lastError = 0;
    public double angleTolerance = 0.5;

    public final double MAX_POWER = 1;
    public final ElapsedTime t = new ElapsedTime();

    public KodiLimelight limelight;
    public LLResult result; // We will store the result here once per loop
    public String teamColor;

    final double LIMELIGHT_HEIGHT = 17;
    final double APRILTAG_HEIGHT = 29.5;
    final double LIMELIGHT_ANGLE = 21;

    public KodiVision(HardwareMap hardwareMap, KodiLimelight limelight, String teamColor) {
        this.hardwareMap = hardwareMap;
        this.limelight = limelight;
        this.teamColor = teamColor;

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

    public void killSwitch() {
        if(limelight != null) {
            limelight.stop();
        }
    }
}