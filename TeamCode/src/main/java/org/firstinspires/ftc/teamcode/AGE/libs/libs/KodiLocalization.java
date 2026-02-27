package org.firstinspires.ftc.teamcode.AGE.libs.libs;


import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@com.acmerobotics.dashboard.config.Config
public class KodiLocalization {

    HardwareMap hardwareMap;
    public Motor verticalEncoder, horizontalEncoder;
    KodiIMU imu;

    public KodiPinPoint pinpoint;

    Thread updateThread;

    public double x = 0, y = 0, theta = 0;
    public double prevV = 0, prevH = 0;

    public boolean kill = false;


    public KodiLocalization(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
        pinpoint = new KodiPinPoint(hardwareMap);
    }

    public void startNew(){
        updateThread = new Thread(() -> {
            x = y = 0; kill = false;
            pinpoint.reset();
            while (!updateThread.isInterrupted() && !kill) {
                pinpoint.update();
                x = pinpoint.getPosition().getX(DistanceUnit.CM);//practic x si inversat
                y = pinpoint.getPosition().getY(DistanceUnit.CM);//y
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
                x = pinpoint.getPosition().getX(DistanceUnit.CM);//practic x si inversat
                y = pinpoint.getPosition().getY(DistanceUnit.CM);//y
                theta=pinpoint.getPosition().getHeading(AngleUnit.DEGREES);
                theta += 360.0 * Math.abs(Math.min(0,Math.signum(theta)));

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