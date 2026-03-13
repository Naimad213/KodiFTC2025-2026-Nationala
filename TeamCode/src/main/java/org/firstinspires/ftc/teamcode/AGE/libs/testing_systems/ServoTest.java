package org.firstinspires.ftc.teamcode.AGE.libs.testing_systems;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.LEFT_TRIGGER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.RIGHT_TRIGGER;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;



@TeleOp(name="ServoTest")
public class ServoTest extends LinearOpMode {

    public ServoEx fireLeft, fireRight, fireMid , servoTest,servoTest2;
    public Motor motor;
    
    // Default (Standby) positions
    double currentLeft = 0.0;
    double currentMid = 1.0; 
    double currentRight = 0.45;

    GamepadEx gm1;

    @Override
    public void runOpMode() throws InterruptedException {

        servoTest = new SimpleServo(hardwareMap, "servoDreapta", 0, 300, AngleUnit.DEGREES);
        //motor = new Motor(hardwareMap  , "testMotor");


        gm1 = new GamepadEx(gamepad1);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addData("Status", "Initialized - Servos at Standby");
        telemetry.update();



        waitForStart();

        while (opModeIsActive()) {
            gm1.readButtons();

            boolean inverted2 , inverted1=false ;

            waitForStart();


                gm1.readButtons();

                /// INVERT SERVO PENTRU BLOCARE
                // Toggle Servo 2 (X)
             //  servoTest2.setInverted(true);

                // Reset both to Normal (DPAD UP)
//                if (gm1.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
//                    inverted1 = false;
//                    inverted2 = false;
                    //servoTest.setInverted(true);
                inverted2 = true;
                    //servoTest2.setInverted(inverted2);
//                }
            servoTest.setInverted(true);
                // --- QUICK ACTION TEST ---
                // By sending the EXACT SAME number to both, you will easily see
                // the physical mirroring when one is inverted.
                if (gm1.getButton(GamepadKeys.Button.A)) {
                    servoTest.setPosition(0);
                    //servoTest2.setPosition(0.8); // Changed from 0 to 0.8
                } else {
                    servoTest.setPosition(0.5);
                    //servoTest2.setPosition(0.0); // Changed from 0.8 to 0.0
                }
                double rightTrigger = gm1.getTrigger(RIGHT_TRIGGER);///in
                double leftTrigger = gm1.getTrigger(LEFT_TRIGGER);///OUT

                //motor.set(rightTrigger-leftTrigger);
                // --- TELEMETRY ---
                telemetry.addData("Hold 'A' to Move", "Both commanded to 0.8");
                telemetry.addLine();
                telemetry.addData("Servo 1 (Y to toggle)", "Inverted: " + inverted1 + " | Pos: " + servoTest.getPosition());
                telemetry.addData("Servo 2 (X to toggle)", "Inverted: " + inverted2 + " | Pos: " ); //servoTest2.getPosition());
                telemetry.update();

        }
    }
}
