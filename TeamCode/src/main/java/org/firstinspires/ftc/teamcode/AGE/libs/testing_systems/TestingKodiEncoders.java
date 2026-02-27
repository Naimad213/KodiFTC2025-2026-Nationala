package org.firstinspires.ftc.teamcode.AGE.libs.testing_systems;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.AGE.libs.libs.Config;
import org.firstinspires.ftc.teamcode.AGE.libs.libs.KodiIMU;


//FAIL MARE
@TeleOp(name="Test-Encodere-raw")
public class TestingKodiEncoders extends LinearOpMode {

    //KodiRobotel robotel;
//    KodiPinPoint pinPoint;
    Motor verticalEncoder, horizontalEncoder;
    KodiIMU imu;

    public double x = 0, y = 0, theta = 0;
    public double prevV = 0, prevH = 0;

    public void initHW(){
        verticalEncoder = new Motor(hardwareMap, "verticalEncoder");
        horizontalEncoder = new Motor(hardwareMap, "intake2");
        imu = new KodiIMU(hardwareMap);


        verticalEncoder.resetEncoder();
        horizontalEncoder.resetEncoder();

        imu.init();
    }
    @Override
    public void runOpMode() throws InterruptedException {
        initHW();


        waitForStart();


            while (opModeIsActive()) {
                double heading = imu.getHeading();


                double dV = verticalEncoder.getDistance() - prevV;
                 double dH = horizontalEncoder.getDistance() - prevH;

                prevV += dV;
                 prevH += dH;

                double hyp = -Math.hypot(dV, dH);
                 double moveAngle = Math.atan2(dV, dH);

               double robotAngle = Math.toRadians(heading);

                double deltaX = hyp * Math.sin(robotAngle + moveAngle);
               double deltaY = hyp * Math.cos(robotAngle + moveAngle);

               x += deltaX;
               y += deltaY;
                telemetry.addData("x", x);
                telemetry.addData("y", y);
                telemetry.addData("distanta raw x", verticalEncoder.getDistance());
                telemetry.addData("distanta raw y", horizontalEncoder.getDistance());
               telemetry.addData("HEADING", heading);
                telemetry.update();

            }

    }
}
