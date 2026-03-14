package org.firstinspires.ftc.teamcode.AGE.libs.finals.auto;

import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AGE.libs.libs.KodiBotFinalV4;


import java.util.concurrent.TimeUnit;

@Autonomous(name="AutoFortat", preselectTeleOp = "BLUE")
public class autoFortat  extends LinearOpMode {

    KodiBotFinalV4 robot;
    Timing.Timer timer = new Timing.Timer(1300, TimeUnit.MILLISECONDS);
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new KodiBotFinalV4(hardwareMap, "BLUE");
        waitForStart();
        timer.start();
        while(opModeIsActive() && !timer.done()){
            robot.drive.driveRobotCentric(0,-.45,0);
        }
        robot.drive.driveRobotCentric(0,0,0);
        sleep(50);
    }
}