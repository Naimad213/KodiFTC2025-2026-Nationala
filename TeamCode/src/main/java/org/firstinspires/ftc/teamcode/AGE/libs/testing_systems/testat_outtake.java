package org.firstinspires.ftc.teamcode.AGE.libs.testing_systems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.libs.AGE.ServoSubSystem;

@TeleOp
public class testat_outtake extends LinearOpMode {



    GamepadEx gm1;
    public DcMotorEx outtakeM1,outtakeM2;

    ServoSubSystem servo;

    //  TUNING CONSTANTS FOR REV HD HEX 5:1
    // Max TPS : approx 2800.
    // F = 1 / 2800 = ~0.00036
    public static double f = 0;
    public static double p = 0; // Start low, increase if recovery is slow
    public static double i = 0; // Start low, increase if recovery is slow

    public static double d=0;

    double[] step= { 10.0, 1.0, 0.1,0.001,0.0001};
    int stepIndex=1;
    // Targets (TPS)
    public static double TARGET_SMALL_ZONE = 1600; // ~60% power
    //F:19
    //P:18
    //D:2
    public static double TARGET_BIG_ZONE = 1000;// ~85% power
    //F: 18
    //P:16
    //D:2

    private double currentTargetVelocity = TARGET_BIG_ZONE;

    double seconds = System.currentTimeMillis()*0.001;

    public void initHW() {
        // Initialize Motor
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        outtakeM2 = hardwareMap.get(DcMotorEx.class, "outtakeM2");
        outtakeM2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        outtakeM2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        outtakeM1 = hardwareMap.get(DcMotorEx.class, "outtakeM1");
        outtakeM1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        outtakeM1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        servo= new ServoSubSystem(hardwareMap);
        gm1 =new GamepadEx(gamepad1);

        telemetry.addLine("init complete");
    }


    @Override
    public void runOpMode() throws InterruptedException {


        initHW();
        waitForStart();
        //f: 15  p: 14
//9.6 6.8
        while (opModeIsActive()) {
            if(gm1.gamepad.optionsWasPressed()){
                if(currentTargetVelocity==TARGET_BIG_ZONE) currentTargetVelocity=TARGET_SMALL_ZONE;
                else currentTargetVelocity=TARGET_BIG_ZONE;
            }
            if(gm1.gamepad.yWasPressed()) {
                servo.setLeverUp();
            }
            if(gm1.gamepad.aWasPressed()) {
                servo.setLeverDown();
            }
            if(gm1.gamepad.shareWasPressed()){
                stepIndex = (stepIndex+1) % step.length;
            }
            if(gm1.gamepad.dpadLeftWasPressed()){
                f+=step[stepIndex];
            }
            if(gm1.gamepad.dpadRightWasPressed()){
                f-=step[stepIndex];
            }
            if(gm1.gamepad.dpadUpWasPressed()){
                    p+=step[stepIndex];
            }
            if(gm1.gamepad.dpadDownWasPressed()){
                p-=step[stepIndex];
            }

            if(gm1.getButton(GamepadKeys.Button.X)){
                d+=step[stepIndex];
            }
            if(gm1.getButton(GamepadKeys.Button.B)){
                d-=step[stepIndex];
            }
            PIDFCoefficients pidfCoefficients= new PIDFCoefficients(p,0,d,f);
            outtakeM2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
            outtakeM2.setVelocity(-currentTargetVelocity);
            outtakeM1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
            outtakeM1.setVelocity(-currentTargetVelocity);

            double v1 = -outtakeM1.getVelocity();
            double v2 = -outtakeM2.getVelocity();
            double curVelo = (v1 + v2) / 2.0;
            double velocityDiff = Math.abs(v1 - v2);
            double error = Math.abs(currentTargetVelocity - curVelo);
            telemetry.addData("error : ",error);
            telemetry.addData("curVelo : ",curVelo);
            telemetry.addData("TARGET: ",currentTargetVelocity);
            telemetry.addData("Velocity diff: ", velocityDiff);
            telemetry.addData("step : ",step);

            telemetry.addData("P : ",p);
            telemetry.addData("I  : ",i);
            telemetry.addData("D: ", d);
            telemetry.addData("F : ", f);
            telemetry.addData("TIME : ",seconds);
            telemetry.update();
            telemetry.addLine();

        }
    }
}
