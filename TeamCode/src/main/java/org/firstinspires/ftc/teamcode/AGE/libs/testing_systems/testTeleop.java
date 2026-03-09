package org.firstinspires.ftc.teamcode.AGE.libs.testing_systems;

import com.arcrobotics.ftclib.command.MecanumControllerCommand;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.AGE.libs.libs.KodiLimelight;
import org.firstinspires.ftc.teamcode.AGE.libs.libs.KodiPinPoint;

@TeleOp(name = "testTeleop", group = "TESTE-PIESE")
public class testTeleop extends LinearOpMode {
    MecanumDrive drive;
     Motor lFMotor, lRMotor, rFMotor, rRMotor;
   //  KodiPinPoint pinPoint;
    GamepadEx gm1;

    public void initHW(){

        lFMotor = new Motor(hardwareMap, "leftFront");
        rFMotor = new Motor(hardwareMap, "rightFront");
        lRMotor = new Motor(hardwareMap, "leftRear");
        rRMotor = new Motor(hardwareMap, "rightRear");

         gm1 = new GamepadEx(gamepad1);
        lFMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rFMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        lRMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rRMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rRMotor.setInverted(false);

        drive = new MecanumDrive(lFMotor,rFMotor,lRMotor,rRMotor);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initHW();
        waitForStart();
        if(isStopRequested()) return;
        while (opModeIsActive()) {
            gm1.readButtons();
            double x = gm1.getLeftX();
            double y = -gm1.getLeftY();
           // pinPoint.update();
           // double theta = pinPoint.getPosition().getHeading(AngleUnit.DEGREES);
           // theta += 360.0 * Math.abs(Math.min(0, Math.signum(theta)));
            double turn = gm1.getRightX();

            drive.driveRobotCentric(x,y,turn);

        }
    }
}
