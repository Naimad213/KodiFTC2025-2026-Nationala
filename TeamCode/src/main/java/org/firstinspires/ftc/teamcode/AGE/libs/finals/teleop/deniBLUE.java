package org.firstinspires.ftc.teamcode.AGE.libs.finals.teleop;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.LEFT_TRIGGER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.RIGHT_TRIGGER;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

// ADDED: Imports for LynxModule and ElapsedTime
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.List;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


import org.firstinspires.ftc.teamcode.AGE.libs.libs.KodiBotFinalV4;

@TeleOp(name="BLUE" ,group="NATIONALA")
public class deniBLUE extends LinearOpMode {

    KodiBotFinalV4 robot;
    GamepadEx gm1;

    double x,y, turn,turnCorrection,finalTurnPower,theta=0;

    List<LynxModule> allHubs;

    public void initHW() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new KodiBotFinalV4(hardwareMap,"BLUE");
        gm1 = new GamepadEx(gamepad1);
        gm1.gamepad.setLedColor(217, 65, 148, 999999);

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
                robot.servoSubSystem.updateInversion();
                gm1.readButtons();

                x = -gm1.getLeftX();
                y = -gm1.getLeftY();
                theta = robot.imu.getAbsoluteHeading();
                theta = (theta % 360 + 360) % 360;
                turn = -gm1.getRightX();


//                if ((gm1.getButton(GamepadKeys.Button.RIGHT_BUMPER))){
//                    robot.vision.updateLimelight();
//                    turnCorrection = robot.vision.getLimelightRotationCorrection(robot.vision.isSeeingAprilTag(20));
//                } else {
//                    turnCorrection = 0;
//                }
//                finalTurnPower = turn+turnCorrection;

                /// CHASSIS DRIVE
                robot.driveWithVoltageCompensation(x, y, turn);

                handleSubsystems();
                /// calculam looptime si hz
                double loopTime = loopTimer.milliseconds();
                loopTimer.reset();
                robot.vision.updateLimelight();
                telemetry.addData("Loop Time (ms)", loopTime);
                telemetry.addData("Loop Hz", 1000.0 / loopTime);
                telemetry.addData("voltaj: ", robot.batteryVoltageSensor.getVoltage());
                telemetry.addData("color out: " , robot.servoSubSystem.getColor(robot.servoSubSystem.sensorOut));
                telemetry.addData("ready to shoot" , robot.outtake.readyToShoot());
                telemetry.addData("current rpm" , robot.outtake.getVelocity());
                telemetry.addData("target rpm" , robot.outtake.currentTargetVelocity);
                telemetry.addData("distance : " ,robot.vision.getDistance());
                telemetry.update();
            }
        } catch (Exception e) {
            robot.killSwitch();
        }

    }
    public void handleSubsystems() {
        double rightTrigger = gm1.getTrigger(RIGHT_TRIGGER);
        double leftTrigger = gm1.getTrigger(LEFT_TRIGGER);

        robot.intake.update(leftTrigger, rightTrigger);


        boolean b = gm1.getButton(GamepadKeys.Button.B);

        double dist = (robot.vision.getDistance() != -1) ? robot.vision.getDistance() :0;
        robot.servoSubSystem.servoDreapta.setInverted(true);
        if (robot.outtake.readyToShoot()&& robot.servoSubSystem.readyToLaunch()) {
            robot.servoSubSystem.servoDreapta.setPosition(0);
        } else {
            robot.servoSubSystem.servoDreapta.setPosition(0.5);
        }

        robot.servoSubSystem.servoDreapta.setInverted(true);
        if(b){
            robot.vision.updateLimelight();
            dist = (robot.vision.getDistance() != -1) ? robot.vision.getDistance() :0;
        }
        if (b && robot.outtake.readyToShoot() && robot.servoSubSystem.readyToLaunch()) {
            robot.vision.updateLimelight();
            robot.intake.update(0, 0);
        }
        if (!robot.servoSubSystem.readyToLaunch() && b && dist==0) {
            robot.intake.update(0, 1);
        }
        robot.outtake.update(b, dist);

    }
}