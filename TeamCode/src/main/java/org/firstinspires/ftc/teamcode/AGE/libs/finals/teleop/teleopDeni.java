package org.firstinspires.ftc.teamcode.AGE.libs.finals.teleop;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.LEFT_TRIGGER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.RIGHT_TRIGGER;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.teamcode.AGE.libs.libs.KodiBotFinalV3;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name="RED" ,group="NATIONALA")
public class teleopDeni extends LinearOpMode {

    KodiBotFinalV3 robot;
    GamepadEx gm1;

    AprilTagDetection idTower,GPP,PGP,PPG;

    double x,y, turn,turnCorrection,finalTurnPower,theta=0;





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


                 x = gm1.getLeftX();
                 y = gm1.getLeftY();
                robot.pinPoint.update();
                theta = robot.pinPoint.getPosition().getHeading(AngleUnit.DEGREES);
                theta += 360.0 * Math.abs(Math.min(0, Math.signum(theta)));
                turn = gm1.getRightX();

                idTower = robot.vision.getDetection(24);

                if ((gm1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER))){
                    turnCorrection = robot.vision.getRotationCorrection(idTower);
                } else {
                    turnCorrection = 0;
                }
                 finalTurnPower = turn+turnCorrection;

                /// CHASSIS DRIVE
                robot.driveWithVoltageCompensation(x, y, finalTurnPower, theta);



                /// INTAKE
                double rightTrigger = gm1.getTrigger(RIGHT_TRIGGER);
                double leftTrigger = gm1.getTrigger(LEFT_TRIGGER);
                if(!robot.sortSubsystem.allMatched) {
                    robot.intake.update(rightTrigger, leftTrigger);
                }

                AprilTagDetection activePattern = null;
                String activeP="";
                if (GPP != null) {
                    activePattern = GPP;
                    activeP = "GPP";
                }
                else if (PGP != null) {
                    activePattern = PGP;
                    activeP="PGP";
                }
                else if (PPG != null) {
                    activePattern = PPG;
                    activeP="PPG";
                }
                /// SORTING+OUTTAKE
                NormalizedColorSensor[] launchQueue = robot.sortSubsystem.getLaunchSequence(activePattern);

                boolean b = gm1.getButton(GamepadKeys.Button.B);
                robot.outtake.update(b, robot.flyWheelSpline.getTargetRPM(robot.vision.getDistance()));
                if (b) {
                    for (NormalizedColorSensor sensorToFire : launchQueue) {
                        if (sensorToFire == robot.sortSubsystem.BLeft) {
                             robot.servoSubSystem.fireLeft();
                            sleep(100);
                        } else if (sensorToFire == robot.sortSubsystem.MidSensor) {
                             robot.servoSubSystem.fireMid();
                            sleep(100);
                        } else if (sensorToFire == robot.sortSubsystem.BRight) {
                             robot.servoSubSystem.fireRight();
                            sleep(100);
                        }
                    }
                }


                telemetry.addData("voltaj: ", robot.batteryVoltageSensor.getVoltage());
                telemetry.addData("current pattern : ", activeP);





                telemetry.update();
            }
        } catch (Exception e) {
            robot.killSwitch();
        }
    }
}