package org.firstinspires.ftc.teamcode.testing.AGE.auto_final.pursuit;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.AGE.libs.libs.KodiBotFinalV4;
import org.firstinspires.ftc.teamcode.AGE.libs.libs.KodiLocalization;
import org.firstinspires.ftc.teamcode.AGE.libs.libs.KodiPursuit;

import java.util.ArrayList;

@Autonomous(name = "BLUE-FAR", preselectTeleOp = "BLUE", group = "AGE-AUTO")
public class blue_far extends LinearOpMode {

    NormalizedRGBA colors, colorsIntake;

    enum DetectedColor {GREEN, PURPLE, NOTHING}

    DetectedColor detectedColor = DetectedColor.NOTHING;

    KodiBotFinalV4 robot;
    MecanumDrive drive;
    KodiPursuit pp1;
    KodiLocalization loc;
    NormalizedColorSensor colorSensor;


    ArrayList<KodiPursuit> pursuitRegistry = new ArrayList<>();
    ElapsedTime timer = new ElapsedTime();

    public void initHW() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot = new KodiBotFinalV4(hardwareMap , "BLUE");
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

            /// START POINT TO OUTTAKE
            ElapsedTime safetyTimer = new ElapsedTime();
            pp1 = new KodiPursuit(drive, telemetry, loc)
                    .goTo(-12, 23, -16)
                    .execute();
            pursuitRegistry.add(pp1);
            robot.vision.updateLimelight();
            robot.outtake.update(true, robot.vision.getDistance());
            while (opModeIsActive() && !pp1.finished()) ; // Wait for first movement
            while (opModeIsActive() && !robot.outtake.readyToShoot() && safetyTimer.seconds() < 0.85) ;
            shootBurst(3, 120);
            while (opModeIsActive() && !pp1.finished()) ;//IDLE IN INTAKE
            robot.outtake.update(false, 0);
            /// SFARSIT START OUTTAKE


            /// INTAKE ZONA 1
            KodiPursuit pp2 = new KodiPursuit(drive, telemetry, loc)
                    .goTo(0, 100, -80)
                    .goTo(-100, 100, -80)
                    .execute();
            pursuitRegistry.add(pp2);
            robot.intake.update(1,0);
            while (opModeIsActive() && !pp2.finished()) ;
            robot.intake.stop();
            /// SFARSIT INTAKE 1


            ///OUTTAKE 1
            KodiPursuit pp3 = new KodiPursuit(drive, telemetry, loc)
                    .goTo(-12, 23, -15)
                    .execute();
            pursuitRegistry.add(pp3);
            robot.vision.updateLimelight();
            robot.outtake.update(true, robot.vision.getDistance());
            while (opModeIsActive() && !pp3.finished()) ;///RESETARE TIMER SI AJUNS LA TARGET
            safetyTimer.reset();
            while (opModeIsActive() && !robot.outtake.readyToShoot() && safetyTimer.seconds() < 0.85) ;
            shootBurst(3, 180);
            while (opModeIsActive() && !pp3.finished()) ;//IDLE IN INTAKE
            robot.outtake.update(false, 0);
            robot.servoSubSystem.servoDreapta.setPosition(0.3);


            /// INTAKE HUMAN PLAYER
            KodiPursuit pp4 = new KodiPursuit(drive, telemetry, loc)
                    .goTo(0, 10, -80)
                    .goTo(-130, 10, -80)
                    .goTo(0, 10, -80)
                    .goTo(-130, 10, -80)
                    .execute();
            pursuitRegistry.add(pp4);
            robot.intake.update(1,0);
            while (opModeIsActive() && !pp4.finished()) ;
            robot.intake.stop();
            /// SFARSIT INTAKE 2


            ///OUTTAKE 2
            KodiPursuit pp5 = new KodiPursuit(drive, telemetry, loc)
                    .goTo(-15, 20, -16)
                    .execute();
            pursuitRegistry.add(pp5);
            robot.vision.updateLimelight();
            robot.outtake.update(false, robot.vision.getDistance());
            while (opModeIsActive() && !pp5.finished()) ; // fixat la ultima sesiune de fail era pp3
            safetyTimer.reset();
            while (opModeIsActive() && !robot.outtake.readyToShoot() && safetyTimer.seconds() < 0.85) ;
            shootBurst(2.8, 150);
            while (opModeIsActive() && !pp5.finished()) ;//IDLE IN INTAKE
            robot.outtake.update(false, 0);
            /// SFARSIT OUTTAKE 2

            /// PARK
            KodiPursuit pp8 = new KodiPursuit(drive, telemetry, loc)
                    .goTo(-30, 20, -88)
                    .execute();
            pursuitRegistry.add(pp8);
            while (opModeIsActive() && !pp8.finished()) ;
            /// SFARSIT PARK
            telemetry.addData("Status: ", "Finished");
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

    private void shootBurst(double duration, int shootingDelay) {
        timer.reset();
        while (opModeIsActive() && timer.seconds() <= duration) {
            robot.servoSubSystem.getColor(robot.servoSubSystem.sensorOut);
            if (detectedColor != DetectedColor.NOTHING) {
                robot.intake.update(0, 0);
                robot.servoSubSystem.updateInversion();
                robot.servoSubSystem.servoDreapta.setPosition(0);
            } else {
                robot.servoSubSystem.servoDreapta.setPosition(0.3);
                sleep(shootingDelay);
                robot.intake.update(0, 1); // Keep intaking while waiting
            }
        }
    }



}