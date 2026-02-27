package org.firstinspires.ftc.teamcode.AGE.libs.testing_systems;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.LEFT_TRIGGER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.RIGHT_TRIGGER;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.AGE.libs.libs.KodiBotFinalV3;


@TeleOp(name="Tele_op_cu_Intake_11_24.11.2025")
public class IntakeTest extends LinearOpMode {

    Motor motorTurret1;


     CRServo servoIntake;

    KodiBotFinalV3 robot;
    GamepadEx gm1;



    public void initHW() {
        robot = new KodiBotFinalV3(hardwareMap);
        motorTurret1 = new Motor(hardwareMap, "intakeM");


        servoIntake = new CRServo(hardwareMap,"clapa");

        gm1 = new GamepadEx(gamepad1);

        motorTurret1.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motorTurret1.setInverted(false);//sunt deja

    }

    @Override
    public void runOpMode() throws InterruptedException {
        initHW();
        waitForStart();


        try {
            while (opModeIsActive()) {

                gm1.readButtons();

                double leftTrigger = gm1.getTrigger(LEFT_TRIGGER);   // 0.0 - 1.0
                double rightTrigger = gm1.getTrigger(RIGHT_TRIGGER) ;// 0.0 - 1.0
                boolean rightBumper= gm1.getButton(GamepadKeys.Button.RIGHT_BUMPER);
                boolean leftBumper= gm1.getButton(GamepadKeys.Button.LEFT_BUMPER);
                boolean a= gm1.getButton(GamepadKeys.Button.A);
                boolean b= gm1.getButton(GamepadKeys.Button.B);

                double powerBumper1 = rightBumper ? 0.7 : 0;
                double powerBumper2 = leftBumper ? 0.7 : 0;
                double powerSus = a ? 1 : 0;
                double powerSus2 = b ? 1 : 0;

                double intakeJosPower = (rightTrigger - leftTrigger);
                double intakeSusPower = powerBumper1-powerBumper2;



                motorTurret1.set(intakeJosPower);
                servoIntake.set(powerSus-powerSus2);



                double x = gm1.getLeftX();
                double y = gm1.getLeftY();
                double r = gm1.getRightX();

                robot.drive.driveRobotCentric(x, y, r);

            }

        } catch (Exception e) {
                robot.killSwitch();
        }
    }
}
