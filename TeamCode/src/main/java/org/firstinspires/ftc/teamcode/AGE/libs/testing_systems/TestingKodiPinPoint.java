package org.firstinspires.ftc.teamcode.AGE.libs.testing_systems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.AGE.libs.libs.KodiPinPoint;

@TeleOp(name = "PinPoint-Test")
public class TestingKodiPinPoint extends LinearOpMode {

    KodiPinPoint pinPoint;

    double x, y, r;

    @Override
    public void runOpMode() throws InterruptedException {
        pinPoint = new KodiPinPoint(hardwareMap);
        pinPoint.init();
        pinPoint.reset();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();
        while(opModeIsActive()){
            pinPoint.update();

            y = pinPoint.getPosition().getX(DistanceUnit.MM);
            x = pinPoint.getPosition().getY(DistanceUnit.MM);
            r = (int)pinPoint.getPosition().getHeading(AngleUnit.DEGREES);

            telemetry.addData("x",x);
            telemetry.addData("y",y);
            telemetry.addData("r",r);
            telemetry.update();
        }
    }
}
