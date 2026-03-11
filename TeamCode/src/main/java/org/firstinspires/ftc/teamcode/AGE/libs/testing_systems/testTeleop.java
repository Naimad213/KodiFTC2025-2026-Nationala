package org.firstinspires.ftc.teamcode.AGE.libs.testing_systems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.MecanumControllerCommand;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import org.apache.commons.math3.distribution.TDistribution;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.AGE.libs.libs.KodiBotFinalV4;
import org.firstinspires.ftc.teamcode.AGE.libs.libs.KodiLimelight;
import org.firstinspires.ftc.teamcode.AGE.libs.libs.KodiPinPoint;

@TeleOp(name = "testTeleop", group = "TESTE-PIESE")
public class testTeleop extends LinearOpMode {
   GamepadEx gm1;

   KodiBotFinalV4 robot;



    public void initHW(){

      robot = new KodiBotFinalV4(hardwareMap, "RED");
        telemetry= new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());
       gm1 = new GamepadEx(gamepad1);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initHW();
        waitForStart();
        if(isStopRequested()) return;
        while (opModeIsActive()) {
            gm1.readButtons();
            double x = -gm1.getLeftX();
            double y = -gm1.getLeftY();
           robot.pinPoint.update();
            double theta = robot.pinPoint.getPosition().getHeading(AngleUnit.DEGREES);
            theta += 360.0 * Math.abs(Math.min(0, Math.signum(theta)));
            double turn = -gm1.getRightX();

            robot.driveWithVoltageCompensation(x,y,turn ,theta);
            robot.intake.update(gm1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) , gm1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER));
            boolean b = gm1.getButton(GamepadKeys.Button.B);
            robot.vision.updateLimelight();
            double distance=0;
            if(robot.vision.getDistance()!=-1){
                 distance =robot.vision.getDistance();
            }
            robot.outtake.update(b , distance);
            if(robot.outtake.readyToShoot()){
                robot.servoSubSystem.fireMid();
            }else{
                robot.servoSubSystem.resetMid();
            }
            telemetry.addData("ready to shoot" , robot.outtake.readyToShoot());
            telemetry.addData("cm " , robot.vision.getDistance());
            telemetry.update();
        }
    }

}
