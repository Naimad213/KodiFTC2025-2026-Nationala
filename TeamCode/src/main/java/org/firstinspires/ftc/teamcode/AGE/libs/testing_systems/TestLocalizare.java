package org.firstinspires.ftc.teamcode.AGE.libs.testing_systems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.AGE.libs.libs.KodiLocalization;


@TeleOp
public class TestLocalizare extends LinearOpMode {

    //KodiBotFinalV3 robot;

    KodiLocalization loc;
    public void initHW(){
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        loc = new KodiLocalization(hardwareMap);
    }

    @Override
    public void runOpMode() throws InterruptedException {

            initHW();

            waitForStart();
        loc.startNew();


            while(opModeIsActive() && !isStopRequested()){


                telemetry.addData("x: ",loc.getLocAsPoint().x);
                telemetry.addData("y: ",loc.getLocAsPoint().y);
                telemetry.addData("theta: ",loc.getLocAsPoint().theta);

//                telemetry.addData("xPin: ",loc.pinpoint.getPosition().getX(DistanceUnit.CM));
//                telemetry.addData("yPin: ",loc.pinpoint.getPosition().getY(DistanceUnit.CM));
//                telemetry.addData("theta PIN: ",loc.pinpoint.getPosition().getHeading(AngleUnit.DEGREES));
                telemetry.update();
            }


    }
}
