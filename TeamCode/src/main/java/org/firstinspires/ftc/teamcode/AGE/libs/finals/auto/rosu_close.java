package org.firstinspires.ftc.teamcode.AGE.libs.finals.auto;


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

@Autonomous(name = "ROSU-CLOSE", preselectTeleOp = "RED", group = "AGE-AUTO")
public class rosu_close extends LinearOpMode {

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

        robot = new KodiBotFinalV4(hardwareMap,"RED");
        drive = robot.getDriveSession();
        loc = new KodiLocalization(hardwareMap);


        loc.startNew();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initHW();
        waitForStart();

        if (isStopRequested()) return;

//        try {
//
//            /// START POINT TO OUTTAKE
//            robot.outtake.update(false, 0);
//            ElapsedTime safetyTimer = new ElapsedTime();
//            pp1 = new KodiPursuit(drive, telemetry, loc)
//                    .goTo(50, 78, -118)
//                    .execute();
//            pursuitRegistry.add(pp1);
//            while (opModeIsActive() && !pp1.finished()) ; // Wait for first movement
//            while (opModeIsActive() && !robot.outtake.readyToShoot() && safetyTimer.seconds() < 0.85) ;
//            shootBurst(3, 180);
//            while (opModeIsActive() && !pp1.finished()) ;//IDLE IN INTAKE
//            robot.outtake.update(false, false);
//            robot.servoSubSystem.setLeverDown();
//            /// SFARSIT START OUTTAKE
//
//
//            /// INTAKE ZONA 1
//
//            KodiPursuit pp2 = new KodiPursuit(drive, telemetry, loc)
//                    .goTo(-60, 150, -88)
//                    .execute();
//            pursuitRegistry.add(pp2);
//            robot.intake.update(1, 0);
//            while (opModeIsActive() && !pp2.finished() );
//            robot.intake.stop();
//            /// SFARSIT INTAKE 1
//
//
//            ///OUTTAKE 1
//            robot.outtake.update(false, true);
//            KodiPursuit pp3 = new KodiPursuit(drive, telemetry, loc)
//                    .goTo(-10, 85, -120)
//                    .execute();
//            pursuitRegistry.add(pp3);
//            while (opModeIsActive() && !pp3.finished()) ;///RESETARE TIMER SI AJUNS LA TARGET
//            safetyTimer.reset();
//            while (opModeIsActive() && !robot.outtake.readyToShoot() && safetyTimer.seconds() < 0.85) ;
//            shootBurst(2.7, 180);
//            while (opModeIsActive() && !pp3.finished()) ;//IDLE IN INTAKE
//            robot.outtake.update(false, false);
//            robot.servoSubSystem.setLeverDown();
//            /// SFARSIT OUTTAKE 1
//
//
//            /// INTAKE ZONA 2
//            robot.intake.update(1, 0);
//            KodiPursuit pp4 = new KodiPursuit(drive, telemetry, loc)
//                    .goTo(14, 205, -88)
//                    .goTo(-55, 205, -88)
//                    .execute();
//            pursuitRegistry.add(pp4);
//            while (opModeIsActive() && !pp4.finished() );
//            robot.intake.stop();
//            /// SFARSIT INTAKE 2
//
//
//            ///OUTTAKE 2
//            robot.outtake.update(false, true);
//            KodiPursuit pp5 = new KodiPursuit(drive, telemetry, loc)
//                    .goTo(-10, 85, -120)
//                    .execute();
//            pursuitRegistry.add(pp5);
//            safetyTimer.reset();
//            while (opModeIsActive() && !pp5.finished()) ; // Wait for first movement
//            while (opModeIsActive() && !robot.outtake.readyToShoot() && safetyTimer.seconds() < 0.85) ;
//            shootBurst(2.5, 100);
//            while (opModeIsActive() && !pp1.finished()) ;//IDLE IN INTAKE
//            robot.outtake.update(false, false);
//            robot.servoSubSystem.setLeverDown();
//            /// SFARSIT OUTTAKE 2
//
////            /// INTAKE ZONA 3
////            KodiPursuit pp6 = new KodiPursuit(drive, telemetry, loc)
////                    .goTo(14, 255, -90)
////                    .goTo(-66, 255, -90)
////                    .execute();
////            pursuitRegistry.add(pp6);
////            robot.intake.update(1, 0); // Intake ON
////            while (opModeIsActive() && !pp6.finished() );
////            robot.intake.stop();
////            /// SFARSIT INTAKE 3
//
//
//            /// PARK
//            KodiPursuit pp8 = new KodiPursuit(drive, telemetry, loc)
//                    .goTo(-45, 240, -90)
//                    .execute();
//            pursuitRegistry.add(pp8);
//            while (opModeIsActive() && !pp8.finished()) ;
//            /// SFARSIT PARK
//            telemetry.addData("Status: ", "Finished");
//            throw new InterruptedException();
//        } catch (Exception e) {
//            telemetry.addData("Status", "Finished or Error");
//            for (KodiPursuit p : pursuitRegistry) p.kill();
//            if (loc != null) loc.stop();
//            if (robot != null) robot.killSwitch();
//        } finally {
//            for (KodiPursuit p : pursuitRegistry) p.kill();
//            if (loc != null) loc.stop();
//            if (robot != null) robot.killSwitch();
//        }
    }

//    private void shootBurst(double duration, int shootingDelay) {
//        timer.reset();
//        while (opModeIsActive() && timer.seconds() <= duration) {
//            getColor();
//            if (detectedColor != DetectedColor.NOTHING) {
//                robot.intake.update(0, 0);
//                robot.servoSubSystem.setLeverUp();
//            } else {
//                robot.servoSubSystem.setLeverDown();
//                sleep(shootingDelay); // Give it time to actually go up
//                robot.intake.update(0, 1); // Keep intaking while waiting
//            }
//        }
//    }

    public void getColor(){
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        if (((OpticalDistanceSensor) colorSensor).getLightDetected() < 0.05) {
            detectedColor = DetectedColor.NOTHING;
        } else {
            double red = colors.red, green = colors.green, blue = colors.blue;
            if (green > (red + blue) * 0.75) detectedColor = DetectedColor.GREEN;
            else if ((red + blue) > green * 1.5) detectedColor = DetectedColor.PURPLE;
            else detectedColor = DetectedColor.NOTHING;
        }
    }

}