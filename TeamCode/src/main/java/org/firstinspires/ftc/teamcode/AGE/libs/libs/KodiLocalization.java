package org.firstinspires.ftc.teamcode.AGE.libs.libs;


import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.AGE.libs.libs.KodiIMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@com.acmerobotics.dashboard.config.Config
public class KodiLocalization {

    HardwareMap hardwareMap;
    public Motor verticalEncoder, horizontalEncoder;

    public KodiPinPoint pinpoint;
    public KodiIMU imu;

    Thread updateThread;

    public double x = 0, y = 0, theta = 0;
    public double prevV = 0, prevH = 0;

    public boolean kill = false;

    public double visionAlpha = 0.15;
    
    public double maxVisionCorrection = 8.0;


    public KodiLocalization(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;

        verticalEncoder = new Motor(hardwareMap, "verticalEncoder");
        horizontalEncoder = new Motor(hardwareMap, "rightRear");

        imu = new KodiIMU(hardwareMap);
        imu.init();
        imu.reset();

        verticalEncoder.setDistancePerPulse(Config.TICKS_TO_CM_GOBILDA);
        verticalEncoder.setInverted(true);
        horizontalEncoder.setDistancePerPulse(Config.TICKS_TO_CM_GOBILDA);

        verticalEncoder.resetEncoder();
        horizontalEncoder.resetEncoder();
    }

    public void startNew(){
        updateThread = new Thread(() -> {
            x = y = 0; kill = false;
            pinpoint.reset();
            while (!updateThread.isInterrupted() && !kill) {
                pinpoint.update();

                x = pinpoint.getPosition().getX(DistanceUnit.CM);
                y = pinpoint.getPosition().getY(DistanceUnit.CM) ;
                theta=pinpoint.getPosition().getHeading(AngleUnit.DEGREES);

            }
        });
        updateThread.start();
    }
    public void startNew2(){
        updateThread = new Thread(() -> {
            x = y = 0; kill = false;
            pinpoint.reset();
            while (!updateThread.isInterrupted() && !kill) {
                pinpoint.update();
                x = pinpoint.getPosition().getY(DistanceUnit.CM);//practic x si inversat
                y = -pinpoint.getPosition().getX(DistanceUnit.CM);//y
                theta=-pinpoint.getPosition().getHeading(AngleUnit.DEGREES);
                theta += 360.0 * Math.abs(Math.min(0,Math.signum(theta)));

            }
        });
        updateThread.start();
    }
    public void start(){
        updateThread = new Thread(() -> {
            x = y = 0;
            imu.reset();
            imu.invertGyro();
            while (!updateThread.isInterrupted()){
                theta = imu.getAbsoluteHeading();

                theta = (theta % 360 + 360) % 360;

                double dV = verticalEncoder.getDistance() - prevV;//invert value for right X
                double dH = horizontalEncoder.getDistance() - prevH;

                prevV += dV;
                prevH += dH;

                double hyp = -Math.hypot(dV,dH);
                double moveAngle = Math.atan2(dV,dH);

                double robotAngle = Math.toRadians(360 - theta);

                double deltaX = hyp * Math.sin(robotAngle + moveAngle);
                double deltaY = hyp * Math.cos(robotAngle + moveAngle);

                x += deltaX;
                y += deltaY;
            }

        });
        updateThread.start();
    }
    public void startGemini(){
        updateThread = new Thread(() -> {
        // 1. Reset Position and Yaw
        x = y = 0;
        imu.reset(); // Correctly calls imu.resetYaw() in KodiIMU
        imu.invertGyro();
        // Initialize previous values to prevent a massive jump on the first loop
        prevV = verticalEncoder.getDistance();
        prevH = horizontalEncoder.getDistance();

        while (!updateThread.isInterrupted()){
            // 2. Get Heading and Normalize to 0-360
            theta = imu.getAbsoluteHeading();
            theta = (theta % 360 + 360) % 360;
            double headingRad = Math.toRadians(theta);

            // 3. Calculate local displacements

            double dV =  horizontalEncoder.getDistance()- prevV;
            double dH =  verticalEncoder.getDistance()- prevH;

            prevV += dV;
            prevH += dH;

            // 4. Proper Rotation Matrix (Fixes the "90 degree crazy" issue)
            double cos = Math.cos(headingRad);
            double sin = Math.sin(headingRad);

            // Transform local movement (dH, dV) to global coordinates
            // Standard: Global X = Local_X * cos - Local_Y * sin
            double deltaX = dH * cos - dV * sin;
            double deltaY = dH * sin + dV * cos;

            // 5. Apply "Invert X and Y" by negating the addition
            this.x -= deltaX;
            this.y -= deltaY;

            try {
                Thread.sleep(10); // Prevents CPU hogging
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
    });
        updateThread.start();
    }

    public void startCombined2(){
        updateThread = new Thread(() -> {
            x = y = 0;
            while (!updateThread.isInterrupted()){
                pinpoint.update();
                theta = pinpoint.getPosition().getHeading(AngleUnit.DEGREES);
                theta += 360.0 * Math.abs(Math.min(0,Math.signum(theta)));

                double dV = verticalEncoder.getDistance() - prevV;
                double dH = horizontalEncoder.getDistance() - prevH;

                prevV += dV;
                prevH += dH;

                double hyp = -Math.hypot(dV,dH);
                double moveAngle = Math.atan2(dV,dH);

                double robotAngle = Math.toRadians(360 - theta);

                double deltaX = hyp * Math.sin(robotAngle + moveAngle);
                double deltaY = hyp * Math.cos(robotAngle + moveAngle);

                x += deltaX;
                y += deltaY;

            }

        });
        updateThread.start();
    }

    public void updateFromVision(double visionX, double visionY, double visionTheta, boolean isConfident) {
        if (!isConfident) return;

        /// diferenta pentru a trece coordonatele prin filtru
        double diffX = visionX - this.x;
        double diffY = visionY - this.y;


        if (Math.abs(diffX) > maxVisionCorrection) diffX = Math.signum(diffX) * maxVisionCorrection;
        if (Math.abs(diffY) > maxVisionCorrection) diffY = Math.signum(diffY) * maxVisionCorrection;

        this.x += diffX * visionAlpha;
        this.y += diffY * visionAlpha;


        this.theta = visionTheta;
         pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, this.x, this.y, AngleUnit.DEGREES, this.theta));
    }



    public Point getLocAsPoint(){
        return new Point(x,y,theta);
    }



    public void stop(){
        updateThread.interrupt();
        kill = true;
    }
    //Restart mid auto pentru sistem cartezian
    public void restart(){
        stop();



        pinpoint.reset();
        this.x = 0;
        this.y = 0;

        // Start the fresh thread
        startNew();

    }

}