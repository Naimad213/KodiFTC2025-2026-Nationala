package org.firstinspires.ftc.teamcode.AGE.libs.testing_systems;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.LEFT_TRIGGER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.RIGHT_TRIGGER;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp(name="test-Intake")
public class testIntake extends LinearOpMode {


    GamepadEx gm1;

    DcMotor intake;

    public void initHW(){
        intake = hardwareMap.get(DcMotor.class, "intakeM");
        gm1= new GamepadEx(gamepad1);
    }
    @Override
    public void runOpMode() throws InterruptedException {
        try {
            initHW();
            waitForStart();
            while(opModeIsActive()){
                gm1.readButtons();

                double leftTrigger = gm1.getTrigger(LEFT_TRIGGER);   // 0.0 - 1.0
                double rightTrigger = gm1.getTrigger(RIGHT_TRIGGER) ;// 0.0 - 1.0

                double intakePower = (rightTrigger - leftTrigger);

                intake.setPower(intakePower);
            }
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
    }

}
