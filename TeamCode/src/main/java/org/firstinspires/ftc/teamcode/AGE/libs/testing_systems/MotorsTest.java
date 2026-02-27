package org.firstinspires.ftc.teamcode.AGE.libs.testing_systems;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name="Test motoare", group="TESTE-PIESE")
public class MotorsTest extends LinearOpMode {


    public Motor lFMotor, lRMotor, rFMotor, rRMotor;
    @Override
    public void runOpMode() throws InterruptedException {
        lFMotor = new Motor(hardwareMap, "leftFront");
        rFMotor = new Motor(hardwareMap, "rightFront");
        lRMotor = new Motor(hardwareMap, "leftRear");
        rRMotor = new Motor(hardwareMap, "rightRear");

        waitForStart();
        while (opModeIsActive()) {
            boolean a = gamepad1.a;
            boolean b = gamepad1.b;
            boolean x = gamepad1.x;
            boolean y = gamepad1.y;

            double powerLeftFront = y ? 1 : 0;
            double powerRightFront = b ? 1 : 0;
            double powerLeftRear = x ? 1 : 0;
            double powerRightRear = a ? 1 : 0;


            lFMotor.set(powerLeftFront);
            rFMotor.set(powerRightFront);
            lRMotor.set(powerLeftRear);
            rRMotor.set(powerRightRear);

            telemetry.addData("leftFront : ", powerLeftFront);
            telemetry.addData("rightFront : ", powerRightFront);
            telemetry.addData("leftRear : ", powerLeftRear);
            telemetry.addData("rightRear : ", powerRightRear);





        }
    }
}
