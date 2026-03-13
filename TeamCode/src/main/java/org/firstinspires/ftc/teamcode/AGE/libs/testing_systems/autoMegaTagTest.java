package org.firstinspires.ftc.teamcode.AGE.libs.testing_systems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.AGE.libs.libs.KodiBotFinalV4;
import org.firstinspires.ftc.teamcode.AGE.libs.libs.KodiLocalization;
import org.firstinspires.ftc.teamcode.AGE.libs.libs.KodiPursuit;

public class autoMegaTagTest extends LinearOpMode {

    KodiPursuit pp;
    KodiBotFinalV4 robot;
    MecanumDrive drive;
    KodiLocalization loc;



    public void initHW(){
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());
        robot= new KodiBotFinalV4(hardwareMap  , "RED" );
        drive = robot.getDriveSession();
        loc= new KodiLocalization(hardwareMap);

    }

    @Override
    public void runOpMode() throws InterruptedException {
        initHW();
        waitForStart();
        try {
            while(opModeIsActive()){
                pp.goTo(60, 120, 90).execute();

                while (opModeIsActive() && !pp.finished()) {

                    Pose3D visionPose = robot.vision.updateMegaTag2Pose();

                    if (visionPose != null) {


                        double vX = visionPose.getPosition().x * 100.0;
                        double vY = visionPose.getPosition().y * 100.0;

                        double vTheta = robot.vision.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);


                        double distanceToTag = robot.vision.getDistance();


                        boolean isConfident = (distanceToTag > 0 && distanceToTag < 180.0);

                        loc.updateFromVision(vX, vY, vTheta, isConfident);
                    }

                    telemetry.addData("Loc X (cm)", loc.x);
                    telemetry.addData("Loc Y (cm)", loc.y);
                    telemetry.update();
                }
            }
            throw new InterruptedException();
        } catch (InterruptedException e) {
            robot.killSwitch();
        }
    }
}
