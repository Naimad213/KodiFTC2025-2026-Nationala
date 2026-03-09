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

import org.firstinspires.ftc.teamcode.AGE.libs.libs.KodiBotFinalV3;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name="RED" ,group="NATIONALA")
public class teleopDeni extends LinearOpMode {

    KodiBotFinalV3 robot;
    GamepadEx gm1;

    AprilTagDetection idTower,GPP,PGP,PPG;

    double x,y, turn,turnCorrection,finalTurnPower,theta=0;

    // ADDED: List to hold the hubs
    List<LynxModule> allHubs;

    public void initHW() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
       // robot = new KodiBotFinalV3(hardwareMap);
        gm1 = new GamepadEx(gamepad1);
        gm1.gamepad.setLedColor(217, 65, 148, 999999);
        robot.pinPoint.init();

        // ADDED: Get all hubs and set them to MANUAL bulk caching
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initHW();

        // ADDED: Timer for loop times
        ElapsedTime loopTimer = new ElapsedTime();

        waitForStart();
        loopTimer.reset();

        try {
            while (opModeIsActive()) {
                // ADDED: Clear the bulk cache at the VERY START of every loop
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

                /// DETECTIE PENTRU SORTARE
                GPP = robot.vision.getDetection(21);
                PGP = robot.vision.getDetection(22);
                PPG = robot.vision.getDetection(23);
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
                if (b && robot.outtake.readyToShoot()) {
                    for (NormalizedColorSensor sensorToFire : launchQueue) {
                        if (sensorToFire == robot.sortSubsystem.BLeft) {
                            robot.servoSubSystem.fireLeft();
                            sleep(50);
                        } else if (sensorToFire == robot.sortSubsystem.MidSensor) {
                            robot.servoSubSystem.fireMid();
                            sleep(50);
                        } else if (sensorToFire == robot.sortSubsystem.BRight) {
                            robot.servoSubSystem.fireRight();
                            sleep(50);
                        }
                    }
                }

                // ADDED: Calculate and display loop time
                double loopTime = loopTimer.milliseconds();
                loopTimer.reset(); // Reset timer for the next loop

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