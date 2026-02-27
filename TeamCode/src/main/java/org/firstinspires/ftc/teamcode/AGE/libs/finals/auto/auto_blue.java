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


import org.firstinspires.ftc.teamcode.AGE.libs.libs.KodiBotFinalV3;
import org.firstinspires.ftc.teamcode.AGE.libs.libs.KodiLocalization;
import org.firstinspires.ftc.teamcode.AGE.libs.libs.KodiPursuit;

import java.util.ArrayList;

@Autonomous(name = "AUTO BLUE-NOU-SOLO", preselectTeleOp = "TELEOP-67", group = "AGE-AUTO")
public class auto_blue extends LinearOpMode {

    NormalizedRGBA colors, colorsIntake;

    enum DetectedColor {GREEN, PURPLE, NOTHING}

    DetectedColor detectedColor = DetectedColor.NOTHING;

    KodiBotFinalV3 robot;
    MecanumDrive drive;
    KodiPursuit pp1;
    KodiLocalization loc;
    NormalizedColorSensor colorSensor;


    ArrayList<KodiPursuit> pursuitRegistry = new ArrayList<>();
    ElapsedTime timer = new ElapsedTime();

    public void initHW() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new KodiBotFinalV3(hardwareMap);
        robot.init();
        drive = robot.getDriveSession();
        loc = new KodiLocalization(hardwareMap);
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
        colors = colorSensor.getNormalizedColors();
        colorSensor.setGain(15.0f);
        loc.startNew();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initHW();
        waitForStart();

        if (isStopRequested()) return;

        try {

            /// START POINT TO OUTTAKE
            robot.outtake.update(true);
            ElapsedTime safetyTimer = new ElapsedTime();
            pp1 = new KodiPursuit(drive, telemetry, loc)
                    .goTo(70, -70, 45)
                    .execute();
            pursuitRegistry.add(pp1);
            while (opModeIsActive() && !pp1.finished()) ; // Wait for first movement
            while (opModeIsActive() && !robot.outtake.readyToShoot() && safetyTimer.seconds() < 0.9) ;
            shootBurst(2.4, 130);
            while (opModeIsActive() && !pp1.finished()) ;//IDLE IN INTAKE
            robot.outtake.update(true);
            robot.servoSubSystem.setLeverDown();
            /// SFARSIT START OUTTAKE


            /// INTAKE ZONA 1

            KodiPursuit pp2 = new KodiPursuit(drive, telemetry, loc)
                    .goTo(60, -132, 90)
                    .goTo(-20,132,90)
                    .execute();
            pursuitRegistry.add(pp2);
            robot.intake.update(1, 0);
            while (opModeIsActive() && !pp2.finished() );
            robot.intake.update(1,0);
            sleep(100);
            robot.intake.stop();
            /// SFARSIT INTAKE 1


            ///OUTTAKE 1
            robot.outtake.update( true);
            KodiPursuit pp3 = new KodiPursuit(drive, telemetry, loc)
                    .goTo(-70, -70, 45)
                    .execute();

            pursuitRegistry.add(pp3);
            while (opModeIsActive() && !pp3.finished()) ;///RESETARE TIMER SI AJUNS LA TARGET
            safetyTimer.reset();
            while (opModeIsActive() && !robot.outtake.readyToShoot() && safetyTimer.seconds() < 1) ;
            shootBurst(2.4, 100);
            while (opModeIsActive() && !pp3.finished()) ;//IDLE IN INTAKE
            robot.outtake.update(true);
            robot.servoSubSystem.setLeverDown();
            /// SFARSIT OUTTAKE 1


            /// INTAKE ZONA 2
            KodiPursuit pp4 = new KodiPursuit(drive, telemetry, loc)
                    .goTo(-75, 195, 90)
                    .goTo(-5, 195, 90)
                    .execute();
            while(!pp4.finished() && loc.y>=190)  robot.intake.update(0.7, 0);
            pursuitRegistry.add(pp4);
            while (opModeIsActive() && !pp4.finished() );
            robot.intake.stop();
            /// SFARSIT INTAKE 2


            ///OUTTAKE 2
            robot.outtake.update( true);
            KodiPursuit pp5 = new KodiPursuit(drive, telemetry, loc)
                    .goTo(-70, 78, 123)
                    .execute();
            pursuitRegistry.add(pp5);
            safetyTimer.reset();
            while (opModeIsActive() && !pp5.finished()) ; // asteptam prima miscare sa fie gata
            while (opModeIsActive() && !robot.outtake.readyToShoot() && safetyTimer.seconds() < 1) ;
            shootBurst(2, 140);
            while (opModeIsActive() && !pp1.finished()) ;//iddle
            robot.outtake.update(true);
            robot.servoSubSystem.setLeverDown();
            /// SFARSIT OUTTAKE 2

            /// INTAKE ZONA 3
            KodiPursuit pp6 = new KodiPursuit(drive, telemetry, loc)
                    .goTo(-70, 255, 90)
                    .goTo(-5, 255, 90)
                    .execute();
            pursuitRegistry.add(pp6);
            robot.intake.update(1, 0); // Intake ON
            while (opModeIsActive() && !pp6.finished() );
            robot.intake.stop();
            /// SFARSIT INTAKE 3



            /// PARK
            KodiPursuit pp8 = new KodiPursuit(drive, telemetry, loc)
                    .goTo(-12, 110, 90)
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
            getColor();
            if (detectedColor != DetectedColor.NOTHING) {
                robot.intake.update(0, 0);
                robot.servoSubSystem.setLeverUp();
            } else {
                robot.servoSubSystem.setLeverDown();
                sleep(shootingDelay); //aplicam delay
                robot.intake.update(1, 0); // luam in intake
            }
        }
    }

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
