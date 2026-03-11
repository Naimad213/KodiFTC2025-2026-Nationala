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

    public ServoEx fireLeft, fireRight, fireMid;
    
    // Default (Standby) positions
    double currentLeft = 0.0;
    double currentMid = 1.0; 
    double currentRight = 0.45;

    GamepadEx gm1;

    @Override
    public void runOpMode() throws InterruptedException {
        // Fix: Use SimpleServo or hardwareMap.get
        fireLeft = new SimpleServo(hardwareMap, "fireRight", 0, 300, AngleUnit.DEGREES);
        fireMid = new SimpleServo(hardwareMap, "fireMid", 0, 300, AngleUnit.DEGREES);
        fireRight = new SimpleServo(hardwareMap, "fireLeft", 0, 300, AngleUnit.DEGREES);
        fireMid.setInverted(true);
        // Set standby positions immediately
        fireLeft.setPosition(currentLeft);
        fireMid.setPosition(currentMid);
        fireRight.setPosition(currentRight);

        gm1 = new GamepadEx(gamepad1);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addData("Status", "Initialized - Servos at Standby");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            gm1.readButtons();

            // --- MANUAL TUNING ---
            // Left (DPAD UP/DOWN)
            if (gm1.wasJustPressed(GamepadKeys.Button.DPAD_UP)) currentLeft += 0.05;
            else if (gm1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) currentLeft -= 0.05;

            // Mid (DPAD LEFT/RIGHT)
            if (gm1.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) currentMid += 0.05;
            else if (gm1.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) currentMid -= 0.05;

            // Right (X / B)
            if (gm1.wasJustPressed(GamepadKeys.Button.X)) currentRight += 0.05;
            else if (gm1.wasJustPressed(GamepadKeys.Button.B)) currentRight -= 0.05;

            // --- QUICK ACTION TEST ---
            // Hold 'A' to see all shoot positions at once
            if (gm1.getButton(GamepadKeys.Button.A)) {
                fireLeft.setPosition(0.35);
                fireMid.setPosition(0.5);
                fireRight.setPosition(0.15);
            } else {
                // Return to tuned standby positions
                currentLeft = Range.clip(currentLeft, 0, 1);
                currentMid = Range.clip(currentMid, 0, 1);
                currentRight = Range.clip(currentRight, 0, 1);

                fireLeft.setPosition(currentLeft);
                fireMid.setPosition(currentMid);
                fireRight.setPosition(currentRight);
            }

            telemetry.addData("--- STANDBY (Tuning) ---", "");
            telemetry.addData("Left (DPAD U/D)", "%.2f", currentLeft);
            telemetry.addData("Mid (DPAD L/R)", "%.2f", currentMid);
            telemetry.addData("Right (X/B)", "%.2f", currentRight);
            telemetry.addLine("\nHOLD 'A' to test ACTION positions");
            telemetry.addData("FireLeft Actual", fireLeft.getPosition());
            telemetry.addData("FireMid Actual", fireMid.getPosition());
            telemetry.addData("FireRight Actual", fireRight.getPosition());
            telemetry.update();
        }
    }
}
