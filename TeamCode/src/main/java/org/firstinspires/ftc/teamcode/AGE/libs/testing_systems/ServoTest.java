package org.firstinspires.ftc.teamcode.AGE.libs.testing_systems;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.AGE.libs.libs.ServoSubSystem;


/**
 * A TeleOp mode designed for testing and calibrating a specific servo motor.
 * <p>
 * This OpMode initializes a {@link SimpleServo} named "trap" (configured for 0-300 degrees)
 * and allows the operator to manually adjust its position using the gamepad D-pad.
 * </p>
 *
 * <p><b>Controls:</b></p>
 * <ul>
 *   <li><b>Gamepad 1 D-pad Up:</b> Increment the servo position by 0.1.</li>
 *   <li><b>Gamepad 1 D-pad Down:</b> Decrement the servo position by 0.1.</li>
 * </ul>
 *
 * <p>
 * The current position of the servo is displayed via telemetry for feedback and debugging purposes.
 * Note: The position variable is not clamped, so values may exceed the 0.0 to 1.0 range logically,
 * though hardware behavior depends on the specific servo configuration.
 * </p>
 */
@TeleOp(name="ServoTest")
public class ServoTest extends LinearOpMode {

    ServoEx servoTest1;
    ServoEx servoTest2;

    ServoSubSystem servoSubSystem;
    double currentPosition=0;
    double currentPosition2=0;

    @Override
    public void runOpMode() throws InterruptedException {
        //0.23
        servoTest1 = new SimpleServo(hardwareMap, "lift", 0, 180,
                AngleUnit.DEGREES);
        servoSubSystem = new ServoSubSystem(hardwareMap);


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {
            if(gamepad1.dpad_up) {
                currentPosition += 0.25; // Decrease this step size
                sleep(200);              // Wait 0.2 seconds so one tap = one step
            }
            else if(gamepad1.dpad_down) {
                currentPosition -= 0.25;
                sleep(200);              // Wait here too
            }
            if(gamepad1.dpad_left) {
                currentPosition2 += 0.25; // Decrease this step size
                sleep(200);              // Wait 0.2 seconds so one tap = one step
            }
            else if(gamepad1.dpad_right) {
                currentPosition2 -= 0.25;
                sleep(200);              // Wait here too
            }
            servoTest1.setPosition(currentPosition);


            telemetry.addData("Servo Position", servoTest1.getPosition());
   //         telemetry.addData("Servo Target", servoTargetPos);
            telemetry.update();

        }
    }
}

