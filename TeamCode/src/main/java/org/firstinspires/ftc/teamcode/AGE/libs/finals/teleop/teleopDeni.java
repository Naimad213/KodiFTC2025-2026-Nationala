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

// ADDED: Imports for LynxModule and ElapsedTime
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.List;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


import org.firstinspires.ftc.teamcode.AGE.libs.libs.KodiBotFinalV4;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name="RED" ,group="NATIONALA")
public class teleopDeni extends LinearOpMode {

    KodiBotFinalV4 robot;
    GamepadEx gm1;

    boolean GPP,PGP,PPG;
    AprilTagDetection gpp,pgp,ppg;

    double x,y, turn,turnCorrection,finalTurnPower,theta=0;

    // ADDED: List to hold the hubs
    List<LynxModule> allHubs;

    public void initHW() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new KodiBotFinalV4(hardwareMap,"RED");
        gm1 = new GamepadEx(gamepad1);
        gm1.gamepad.setLedColor(217, 65, 148, 999999);
        robot.pinPoint.init();

        /// manual bullk cache pentru a optimiza loading time
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initHW();


        ElapsedTime loopTimer = new ElapsedTime();

        waitForStart();
        loopTimer.reset();

        try {
            while (opModeIsActive()) {
               //cache clearing
                for (LynxModule hub : allHubs) {
                    hub.clearBulkCache();
                }

                gm1.readButtons();

                x = gm1.getLeftX();
                y = gm1.getLeftY();
                robot.pinPoint.update();
                theta = robot.pinPoint.getPosition().getHeading(AngleUnit.DEGREES);
                theta += 360.0 * Math.abs(Math.min(0, Math.signum(theta)));
                turn = gm1.getRightX();

                robot.vision.updateLimelight();
                if ((gm1.getButton(GamepadKeys.Button.RIGHT_BUMPER))){
                    turnCorrection = robot.vision.getLimelightRotationCorrection(robot.vision.isSeeingAprilTag(21));
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

                /// DETECTIE PENTRU SORTARE
                GPP = robot.vision.isSeeingAprilTag(21);
                PGP = robot.vision.isSeeingAprilTag(22);
                PPG = robot.vision.isSeeingAprilTag(23);
                AprilTagDetection activePattern = null;
                String activeP="";
                if (GPP) {
                    activePattern = gpp;
                    activeP = "GPP";
                }
                else if (PPG) {
                    activePattern = ppg;
                    activeP="PGP";
                }
                else  {
                    activePattern = ppg;
                    activeP="PPG";
                }

                /// SORTING+OUTTAKE
                NormalizedColorSensor[] launchQueue = robot.sortSubsystem.getLaunchSequence(activePattern);
                boolean b = gm1.getButton(GamepadKeys.Button.B);
                robot.outtake.update(b, robot.flyWheelSpline.getTargetRPM(robot.vision.getDistance()));
                if (b && robot.outtake.readyToShoot()) {
                    for (NormalizedColorSensor sensorToFire : launchQueue) {
                        if (sensorToFire == robot.sortSubsystem.sensorLeft) {
                            robot.servoSubSystem.fireLeft();
                            sleep(50);
                        } else if (sensorToFire == robot.sortSubsystem.sensorMid) {
                            robot.servoSubSystem.fireMid();
                            sleep(50);
                        } else if (sensorToFire == robot.sortSubsystem.sensorRight) {
                            robot.servoSubSystem.fireRight();
                            sleep(50);
                        }
                    }
                }

                /// calculam looptime
                double loopTime = loopTimer.milliseconds();
                loopTimer.reset();

                telemetry.addData("Loop Time (ms)", loopTime);
                telemetry.addData("Loop Hz", 1000.0 / loopTime);
                telemetry.addData("voltaj: ", robot.batteryVoltageSensor.getVoltage());
                telemetry.addData("current pattern : ", activeP);
                telemetry.update();
            }
        } catch (Exception e) {
            robot.killSwitch();
        }
    }
}