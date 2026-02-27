package org.firstinspires.ftc.teamcode.AGE.libs.testing._auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.AGE.libs.libs.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.AGE.libs.libs.KodiBotFinalV3;
import org.firstinspires.ftc.teamcode.AGE.libs.libs.KodiLocalization;
import org.firstinspires.ftc.teamcode.AGE.libs.libs.KodiPursuit;
import org.firstinspires.ftc.teamcode.AGE.libs.libs.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.AGE.libs.libs.ServoSubSystem;

import java.util.ArrayList;

@Autonomous(name = "auto-recalibrare", preselectTeleOp = "TELEOP-67", group = "AGE-AUTO")
public class auto_recalibrare extends LinearOpMode {

    private NormalizedColorSensor colorSensor;
    NormalizedRGBA colors,colorsIntake;
    enum DetectedColor { GREEN, PURPLE, NOTHING }
    DetectedColor detectedColor = DetectedColor.NOTHING;

    KodiBotFinalV3 robot;
    MecanumDrive drive;
    KodiPursuit pp;
    KodiLocalization loc;

    IntakeSubsystem intakeSubsystem;
    OuttakeSubsystem outtakeSubsystem;

    ServoSubSystem servoSubSystem;
    ElapsedTime timer = new ElapsedTime();
    ArrayList<KodiPursuit> pursuitRegistry = new ArrayList<>();

    //IntakeSiOuttakeAuto autoHandlerOuttake, autoHandlerIntake;
    public void initHW(){

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new KodiBotFinalV3(hardwareMap);
        robot.init();
        drive = robot.getDriveSession();
        loc = new KodiLocalization(hardwareMap);

        loc.startNew();

    }
    @Override
    public void runOpMode() throws InterruptedException {

        initHW();
        waitForStart();

        if (isStopRequested()) return;

        try {

            pp= new KodiPursuit(drive, telemetry,loc)
                    .goTo(-40,10,45)
                    .goTo(0,80,0)
                    .goTo(40,10,-90)
                    .goTo(0,0,0)
                    .execute();
            pursuitRegistry.add(pp);
            while(opModeIsActive() && !pp.finished());




            throw new InterruptedException();
        } catch (Exception e) {
            telemetry.addData("Status", "Finished or Error");
            for (KodiPursuit p : pursuitRegistry) p.kill();
            if (loc != null) loc.stop();
            if (robot != null) robot.killSwitch();
        } finally {
            for (KodiPursuit p : pursuitRegistry) p.kill();
            if (loc != null) loc.stop();
            if (robot != null) robot.killSwitch();
        }
    }
}