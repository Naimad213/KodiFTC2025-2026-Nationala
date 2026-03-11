package org.firstinspires.ftc.teamcode.AGE.libs.libs;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class KodiLimelight {

    public Limelight3A limelight;
    HardwareMap hardwareMap;

    public void updateRobotOrientation(double yaw) {
        if (limelight != null) {
            limelight.updateRobotOrientation(yaw);
        }
    }

    public void initRed(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    public void initBlue(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1);
        limelight.start();
    }

    public LLResult getResult(){
        return limelight.getLatestResult();
    }

    public int getDetectedFiducialId() {
        LLResult result = getResult();
        if (result != null && result.isValid() && !result.getFiducialResults().isEmpty()) {
            return result.getFiducialResults().get(0).getFiducialId();
        }
        return -1;
    }

    public void stop(){
        if (limelight != null) {
            limelight.stop();
        }
    }
}