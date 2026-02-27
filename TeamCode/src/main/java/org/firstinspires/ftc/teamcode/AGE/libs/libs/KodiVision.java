package org.firstinspires.ftc.teamcode.AGE.libs.libs;

import static java.lang.Integer.parseInt;

import android.graphics.Color;
import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.opencv.Circle;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ColorSpace;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.Scalar;

import java.util.List;


public class KodiVision {

    HardwareMap hardwareMap;
    public Telemetry telemetry;

    public double kP = 0.05;
    public double kD = 0.12001;

    public double goalX = 0;
    public double lastError = 0;
    public double angleTolerance = 1.0;

    public final double MAX_POWER = 0.5;
    public final ElapsedTime t = new ElapsedTime();

    private final Position cameraPosition = new Position(DistanceUnit.CM, 15, -13, 37, 0);
    private final YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 0, 14.379, 0, 0);

    public AprilTagProcessor aprilTag;
    public VisionPortal visionPortal;

    public KodiVision(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        initVisionPortal();
    }

    public void setKp(double newKp) { kP = newKp; }
    public void setKd(double newKd) { kD = newKd; }

    // This method now ONLY reads from the existing pipeline
    public AprilTagDetection getDetection(int id) {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        for (AprilTagDetection detection : detections) {
            if (detection.metadata.id == id) {
                return detection;
            }
        }
        return null;
    }

    private void initVisionPortal() {
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(false)
                .setDrawCubeProjection(false)
                .setDrawTagOutline(true)
                .setCameraPose(cameraPosition, cameraOrientation)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .setLiveViewContainerId(0) // Note: ID 0 means no live view on robot controller, saves CPU
                .setCameraResolution(new Size(640, 480))
                .build();
    }

    public double getDistanceCorrection(AprilTagDetection currID) {
        if (currID == null) {
            lastError = 0;
            t.reset();
            return 0;
        }

        double deltaTime = t.seconds();

        double error = goalX - currID.ftcPose.bearing;

        double pTerm = kP * error;
        double dTerm = 0;

        if (deltaTime > 0) {
            dTerm = kD * (error - lastError) / deltaTime;
        }

        double turnPower;
        if (Math.abs(error) < angleTolerance) {
            turnPower = 0;
        } else {
            turnPower = Range.clip(pTerm + dTerm, -MAX_POWER, MAX_POWER);
        }

        lastError = error;
        t.reset();

        return turnPower;
    }

    public double getRotationCorrection(AprilTagDetection currID) {
        if (currID == null) {
            lastError = 0;
            t.reset();
            return 0;
        }

        double deltaTime = t.seconds();

        double error = goalX - currID.ftcPose.bearing;

        double pTerm = kP * error;
        double dTerm = 0;

        if (deltaTime > 0) {
            dTerm = kD * (error - lastError) / deltaTime;
        }

        double turnPower;
        if (Math.abs(error) < angleTolerance) {
            turnPower = 0;
        } else {
            turnPower = Range.clip(pTerm + dTerm, -MAX_POWER, MAX_POWER);
        }

        lastError = error;
        t.reset();

        return turnPower;
    }

    public void killSwitch() {
        if (visionPortal != null && visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            visionPortal.close();
        }
    }
}