package org.firstinspires.ftc.teamcode.AGE.libs.testing_systems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="ServoTest")
public class ServoTest extends LinearOpMode {

    public ServoEx fireLeft, fireRight, fireMid, servoTest;
    double currentPosition = 0; // Start at middle

    /// 0.25-0.26 SHOOT POSITION SERVO LEFT
    GamepadEx gm1;

    @Override
    public void runOpMode() throws InterruptedException {
            // Hardware names must match the configuration on the Control Hub
            fireLeft = new SimpleServo(hardwareMap, "fireLeft", 0, 300, AngleUnit.DEGREES);
            fireMid = new SimpleServo(hardwareMap, "fireMid", 0, 300, AngleUnit.DEGREES);
            fireRight = new SimpleServo(hardwareMap, "fireRight", 0, 300, AngleUnit.DEGREES);
            gm1 = new GamepadEx(gamepad1);
            telemetry= new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

            telemetry.addData("Status", "Initialized");
            telemetry.update();

            waitForStart();

        try {
            while (opModeIsActive()) {
                gm1.readButtons();

                // Adjust position with DPAD UP/DOWN
                if (gm1.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                    currentPosition += 0.05;
                } else if (gm1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                    currentPosition -= 0.05;
                }

                // Clamp the position between 0.0 and 1.0
                currentPosition = Range.clip(currentPosition, 0, 1);

                // Apply to all servos for testing
                fireLeft.setPosition(currentPosition);
                fireMid.setPosition(currentPosition);
                fireRight.setPosition(currentPosition);

                // Display values to debug
                telemetry.addData("Target Position", "%.2f", currentPosition);
                telemetry.addData("FireLeft Pos", fireLeft.getPosition());
                telemetry.addData("FireMid Pos", fireMid.getPosition());
                telemetry.addData("FireRight Pos", fireRight.getPosition());telemetry.update();
            }
        } catch (Exception e) {
            e.getMessage();
            e.getCause();
        }
    }
}