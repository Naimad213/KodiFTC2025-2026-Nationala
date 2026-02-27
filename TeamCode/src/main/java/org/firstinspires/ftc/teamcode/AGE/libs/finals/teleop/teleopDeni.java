package org.firstinspires.ftc.teamcode.AGE.libs.finals.teleop;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.LEFT_TRIGGER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.RIGHT_TRIGGER;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.teamcode.AGE.libs.libs.KodiBotFinalV3;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name="TELEOP-67" ,group="MEET")
public class teleopDeni extends LinearOpMode {

    KodiBotFinalV3 robot;
    GamepadEx gm1;


    double theta=0;




    public void initHW() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new KodiBotFinalV3(hardwareMap);
        gm1 = new GamepadEx(gamepad1);
        gm1.gamepad.setLedColor(217, 65, 148, 999999);
        robot.pinPoint.init();


    }

    @Override
    public void runOpMode() throws InterruptedException {
        initHW();
        waitForStart();

        try {
            while (opModeIsActive()) {
                gm1.readButtons();


                double x = gm1.getLeftX();
                double y = gm1.getLeftY();
                robot.pinPoint.update();
                theta = robot.pinPoint.getPosition().getHeading(AngleUnit.DEGREES);
                theta += 360.0 * Math.abs(Math.min(0, Math.signum(theta)));
                double manualTurn = gm1.getRightX();
                double autoTurnCorrection;

                AprilTagDetection id24 = robot.vision.getDetection(24);
                if ((gm1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER))){
                    autoTurnCorrection = robot.vision.getRotationCorrection(id24);
                } else {
                    autoTurnCorrection = 0;
                }
                double finalTurnPower = manualTurn+autoTurnCorrection;



                /// CHASSIS DRIVE
                robot.driveWithVoltageCompensation(x, y, finalTurnPower, theta);



                /// INTAKE
                double rightTrigger = gm1.getTrigger(RIGHT_TRIGGER);
                double leftTrigger = gm1.getTrigger(LEFT_TRIGGER);
                robot.intake.update(rightTrigger, leftTrigger);


                ///OUTTAKE

                boolean b = gm1.getButton(GamepadKeys.Button.B);
                robot.outtake.update(b);


                /// SORTARE






                telemetry.addData("voltaj: ", robot.batteryVoltageSensor.getVoltage());





                telemetry.update();
            }
        } catch (Exception e) {
            robot.killSwitch();
        }
    }
}