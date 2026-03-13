package org.firstinspires.ftc.teamcode.AGE.libs.testing_systems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.AGE.libs.libs.KodiIMU;
import org.firstinspires.ftc.teamcode.AGE.libs.libs.KodiLimelight;
import org.firstinspires.ftc.teamcode.AGE.libs.libs.KodiVision;


@Disabled
@TeleOp(name="MegaTag2 Test" , group = "TESTE-PIESE")
public class megaTagTest extends LinearOpMode {

    KodiVision vision;
    KodiLimelight limelight;
    KodiIMU imu;

    @Override
    public void runOpMode() throws InterruptedException {
        // Fix: The limelight variable was null when passed to KodiVision
        limelight = new KodiLimelight();
        imu = new KodiIMU(hardwareMap);
        imu.init();
        
        // Now limelight is initialized before being passed
        vision = new KodiVision(hardwareMap, limelight, "RED", imu.imu);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addData("Status", "Initialized - Point Limelight at AprilTags");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // 1. Update MegaTag2
            Pose3D robotPose = vision.updateMegaTag2Pose();

            if (robotPose != null) {
                // 2. Convert Meters to Inches
                double xInches = robotPose.getPosition().x * 39.37;
                double yInches = robotPose.getPosition().y * 39.37;
                double yaw = robotPose.getOrientation().getYaw(AngleUnit.DEGREES);

                telemetry.addData("Localization", "MegaTag2 ACTIVE");
                telemetry.addData("X (Inches)", "%.2f", xInches);
                telemetry.addData("Y (Inches)", "%.2f", yInches);
                telemetry.addData("Heading (Deg)", "%.2f", yaw);
            } else {
                telemetry.addData("Localization", "NO TAGS DETECTED");
            }

            telemetry.update();
        }
        
        vision.killSwitch();
    }
}
