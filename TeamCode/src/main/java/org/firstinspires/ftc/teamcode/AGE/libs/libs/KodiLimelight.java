package org.firstinspires.ftc.teamcode.AGE.libs.libs;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class KodiLimelight {


    public Limelight3A limelight;
    HardwareMap hardwareMap;

    public void initRed(HardwareMap hardwareMap){
        this.hardwareMap=hardwareMap;
        limelight= hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
    }
    public void initBlue(HardwareMap hardwareMap){
        this.hardwareMap=hardwareMap;
        limelight= hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1);
        limelight.start();
    }
}
