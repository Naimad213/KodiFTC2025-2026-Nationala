package org.firstinspires.ftc.teamcode.AGE.libs.testing_systems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.AGE.libs.libs.KodiLimelight;
import org.firstinspires.ftc.teamcode.AGE.libs.libs.KodiVision;


@TeleOp
public class limelightTest extends LinearOpMode {


    KodiVision vision;
    KodiLimelight limelight;


    @Override
    public void runOpMode() throws InterruptedException {

        telemetry= new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        limelight= new KodiLimelight();
        vision = new KodiVision(hardwareMap, limelight  , "RED");

        waitForStart();
        if(isStopRequested()) return;
        while(opModeIsActive()){
            if(isStopRequested()) {
                vision.killSwitch();
            }
            vision.updateLimelight();
            telemetry.addData("distance" ,  vision.getDistance() );
            telemetry.update();
        }

    }
}
