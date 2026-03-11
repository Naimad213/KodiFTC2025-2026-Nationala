package org.firstinspires.ftc.teamcode.AGE.libs.testing_systems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.AGE.libs.libs.KodiBotFinalV4;


@TeleOp
public class splineCalibration extends LinearOpMode {

    KodiBotFinalV4 robot;
    GamepadEx gm1;

    double[] step= { 10.0, 1.0, 0.1, 0.01, 0.001, 0.0001};

    double f = 0, p = 0;
    int stepIndex = 2; // Start with 0.1 step

    private double targetRPM = 0;


    public void initHW(){
        robot = new KodiBotFinalV4(hardwareMap, "RED");
        gm1 = new GamepadEx(gamepad1);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initHW();

        waitForStart();
        if(isStopRequested()) return;
        
        while(opModeIsActive()){
            gm1.readButtons();

            double x = -gm1.getLeftX();
            double y = -gm1.getLeftY();
            robot.pinPoint.update();
            double theta = robot.pinPoint.getPosition().getHeading(AngleUnit.DEGREES);
            theta += 360.0 * Math.abs(Math.min(0, Math.signum(theta)));
            double turn = -gm1.getRightX();

            robot.driveWithVoltageCompensation(x, y, turn, theta);

            // Corrected input logic using GamepadEx methods
            if(gm1.wasJustPressed(GamepadKeys.Button.BACK)){ // Mapping for Share/Options
                stepIndex = (stepIndex + 1) % step.length;
            }
            if(gm1.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)){
                f += step[stepIndex];
            }
            if(gm1.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)){
                f -= step[stepIndex];
            }
            if(gm1.wasJustPressed(GamepadKeys.Button.DPAD_UP)){
                p += step[stepIndex];
            }
            if(gm1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)){
                p -= step[stepIndex];
            }

            boolean b = gm1.getButton(GamepadKeys.Button.B);
            boolean shootMid = gm1.getButton(GamepadKeys.Button.X);
            boolean shootLeft = gm1.getButton(GamepadKeys.Button.LEFT_BUMPER);
            boolean shootRight = gm1.getButton(GamepadKeys.Button.RIGHT_BUMPER);
            
            if(shootMid){
                robot.servoSubSystem.fireMid();
            } else {
                robot.servoSubSystem.resetMid();
            }
            if(shootLeft){
                robot.servoSubSystem.fireLeft();
            } else {
                robot.servoSubSystem.resetLeft();
            }
            if(shootRight){
                robot.servoSubSystem.fireRight();
            }else{
                robot.servoSubSystem.resetRight();
            }

            if(gm1.wasJustPressed(GamepadKeys.Button.Y)){
                targetRPM += 100;
            }
            if(gm1.wasJustPressed(GamepadKeys.Button.A)){
                targetRPM -= 100;
            }

            robot.outtake.updateTrain(b, targetRPM, p, f);

            robot.vision.updateLimelight();
            
            telemetry.addData("f", "%.6f", f);
            telemetry.addData("p", "%.6f", p);
            telemetry.addData("targetRPM", targetRPM);
            telemetry.addData("distance", robot.vision.getDistance());
            telemetry.addData("Current Step", step[stepIndex]);
            telemetry.addData("velocity", robot.outtake.getVelocity());
            telemetry.addData("readyToShoot", robot.outtake.readyToShoot());
            telemetry.update();
        }
        
        robot.killSwitch();
    }
}
