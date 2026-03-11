package org.firstinspires.ftc.teamcode.AGE.libs.testing_systems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.AGE.libs.libs.KodiBotFinalV4;
import org.firstinspires.ftc.teamcode.AGE.libs.libs.KodiLocalization;


@TeleOp
public class TestLocalizare extends LinearOpMode {

    //KodiBotFinalV3 robot;

    KodiLocalization loc;
    KodiBotFinalV4 robot;
    GamepadEx gm1;
    public void initHW(){
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        loc = new KodiLocalization(hardwareMap);
        robot = new KodiBotFinalV4(hardwareMap, "RED");
         gm1 = new GamepadEx(gamepad1);
    }

    @Override
    public void runOpMode() throws InterruptedException {

            initHW();

            waitForStart();
        loc.startNew();


            while(opModeIsActive() && !isStopRequested()){
                gm1.readButtons();
                double x = -gm1.getLeftX();
                double y = -gm1.getLeftY();
                robot.pinPoint.update();
                double theta = robot.pinPoint.getPosition().getHeading(AngleUnit.DEGREES);
                theta += 360.0 * Math.abs(Math.min(0, Math.signum(theta)));
                double turn = -gm1.getRightX();
                robot.driveWithVoltageCompensation(x,y,turn ,theta);

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
