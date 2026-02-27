package org.firstinspires.ftc.teamcode.AGE.libs.libs;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;

public class KodiPursuit {

    public KodiLocalization loc;

    Thread pursuitThread;

    public ArrayList<Point> waypoints = new ArrayList<>();

    public Point lastPoint, targetPoint;


    public MecanumDrive drive;

    Telemetry telemetry;

    boolean kill = false;
    //schimbare in loc de MecanumDrive drive cu Robot
    public KodiPursuit(MecanumDrive drive, Telemetry telemetry, KodiLocalization loc){
        this.drive = drive;
        this.telemetry = telemetry;
        this.loc = loc;
    }

    public KodiPursuit goTo(double x, double y){
        waypoints.add(new Point(x,y,Double.NaN));
        return this;
    }

    public KodiPursuit goTo(double x, double y, double theta){
        double angle = (int)(theta + 3600) % 360;
        waypoints.add(new Point(x,y, -angle));
        return this;
    }

    public static double minAbs(double x, double y){
        if(Math.min(Math.abs(x),Math.abs(y)) == Math.abs(x)) return x;
        return y;
    }

    public static double toRobotDegrees(double x){
        return (x % 360 + 360) % 360;
    }

    public Point getBestPoint(double m, double b, Point robot, Point target){
        double xR = robot.x;
        double yR = robot.y;

        if(Math.hypot(target.x - xR, target.y - yR) < Config.targetR) return target;

        double xP = (xR + m * yR - m * b) / (m * m + 1);
        double yP = m * xP + b;
        double d = Math.hypot(xP - xR, yP - yR);

        double x1 = xP + Math.cos(Math.atan(m))
                * (Config.targetT * d + Config.targetR);
        double y1 = m * x1 + b;

        double x2 = xP - Math.cos(Math.atan(m))
                * (Config.targetT * d + Config.targetR);
        double y2 = m * x2 + b;


        /// selectarea punctului castigator



        double d1 = Math.hypot(target.x - x1, target.y - y1);
        double d2 = Math.hypot(target.x - x2, target.y - y2);

        double dMin = Math.min(d1,d2);
        /// returneaza punctul minim
        if(dMin == d1) return new Point(x1,y1);
        return new Point(x2,y2);
    }

    public Point getBestPoint2(double m, double b, Point robot, Point target){
        double xR = robot.x;
        double yR = robot.y;

        if(Math.hypot(target.x - xR, target.y - yR) < Config.targetR) return target;

        double m2_1 = m * m + 1;
        double xP = (xR + m * yR - m * b) / m2_1;
        double yP = m * xP + b;
        double d = Math.hypot(xP - xR, yP - yR);

        double cosAngle = 1.0 / Math.sqrt(m2_1); // Math.cos(Math.atan(m))

//        Apelurile funcțiilor trigonometrice, cum ar fi Math.cos() și Math.atan(),
//        sunt în general mai costisitoare din punct de vedere computațional decât
//        operațiile algebrice simple.
//        Expresia Math.cos(Math.atan(m)) poate fi simplificată.
//
//        Dacă avem angle = atan(m), atunci tan(angle) = m.
//        Folosind identitatea trigonometrică
//        1 + tan²(angle) = sec²(angle) = 1 / cos²(angle),
//        putem deriva: cos(angle) = 1 / sqrt(1 + m²)

        double offset = cosAngle * (Config.targetT * d + Config.targetR)
                * Math.signum(target.x - xP);

        // Math.abs(target.x - xP) > Math.abs((xP - offset) - xP)
        //                                       \= xT
        if(Math.abs(target.x - xP) > Math.abs(offset)){
            double xT = xP + offset;
            double yT = m * xT + b;
            return new Point(xT, yT);
        }

        return target;
    }

    public KodiPursuit execute(){
        pursuitThread = new Thread(() -> {
            lastPoint = loc.getLocAsPoint();
            double targetTheta = lastPoint.theta;
            SpeedController scH = new SpeedController(
                    Config.hA,
                    Config.hV,
                    Config.hBr,
                    Config.hI, Config.toleranceXY, Config.cutOff
            );

            SpeedController scV = new SpeedController(
                    Config.vA,
                    Config.vV,
                    Config.vBr,
                    Config.vI, Config.toleranceXY, Config.cutOff
            );

            SpeedController scR = new SpeedController(
                    Config.rA,
                    Config.rV,
                    Config.rBr,
                    Config.rI, Config.toleranceR, Config.cutOff
            );
            for(int i=0;!kill && i < waypoints.size();i++){
                targetPoint = waypoints.get(i);

                double dX = targetPoint.x - lastPoint.x;
                double dY = targetPoint.y - lastPoint.y;

                if(dX == 0) dX = 1e-6;

                double m = dY/dX;
                double b = lastPoint.y - m * lastPoint.x;

                boolean check = false;

                if(!Double.isNaN(targetPoint.theta)) targetTheta = targetPoint.theta;


                while (!kill && !check){

                    scH.updateCoef(Config.hA, Config.hV, Config.hBr, Config.hI, Config.toleranceXY, Config.cutOff);
                    scV.updateCoef(Config.vA, Config.vV, Config.vBr, Config.vI, Config.toleranceXY, Config.cutOff);
                    scR.updateCoef(Config.rA, Config.rV, Config.rBr, Config.rI, Config.toleranceR, Config.cutOff);

                    Point target = getBestPoint(m,b,loc.getLocAsPoint(),targetPoint);

                    double errorX = target.x - loc.x;
                    double errorY = target.y - loc.y;
                    double errorTheta = targetTheta - loc.theta;

                    errorTheta = minAbs(errorTheta, errorTheta - Math.signum(errorTheta) * 360.0);

                    double distance = Math.hypot(errorX,errorY);

                    /*double movementAngle = Math.atan2(errorY,errorX);

                    double currentTheta = Math.toRadians(360 - loc.theta);

                    double adjustedX = distance * Math.cos(movementAngle - currentTheta);
                    double adjustedY = distance * Math.sin(movementAngle - currentTheta);

                    double x = scH.getSpeed(adjustedX);
                    double y = scV.getSpeed(adjustedY);
                    double r = scR.getSpeed(errorTheta);

                    drive.driveRobotCentric(x, y, -r);*/

                    double x = scH.getSpeed(errorX);
                    double y = scV.getSpeed(errorY);
                    double r = -scR.getSpeed(errorTheta);///+r merge ceva

                    drive.driveFieldCentric(x,y,r,loc.theta);///+theta merge ceva
                /// AM SCHIMBAT FIELDCENTRIC

                    double deltaX = targetPoint.x - loc.x;
                    double deltaY = targetPoint.y - loc.y;

                    double deltaDist = Math.hypot(deltaX,deltaY);

                    telemetry.addData("Kill ",Thread.currentThread().isInterrupted());
                    telemetry.addLine();
                    telemetry.addData("accV",scV.accumulatedError);
                    telemetry.addData("accH",scH.accumulatedError);
                    telemetry.addData("accR",scR.accumulatedError);
                    telemetry.addLine();
                    telemetry.addData("rateV",scV.errRate);
                    telemetry.addData("rateH",scH.errRate);
                    telemetry.addData("rateR",scR.errRate);
                    telemetry.addLine();
                    telemetry.addData("locX",loc.x);
                    telemetry.addData("locY",loc.y);
                    telemetry.addData("locTheta",loc.theta);
                    telemetry.addLine();
                    telemetry.addData("errX",errorX);
                    telemetry.addData("errY",errorY);
                    telemetry.addData("errT",errorTheta);
                    telemetry.addData("dist",deltaDist);
                    telemetry.addLine();
                    telemetry.addData("x",x);
                    telemetry.addData("y",y);
                    telemetry.addData("r",r);
                    telemetry.addLine();
                    telemetry.addData("hor",target.x);
                    telemetry.addData("ver",target.y);
                    telemetry.addData("rot",targetTheta);

                    telemetry.update();


                    if(i == waypoints.size() - 1 || !Double.isNaN(targetPoint.theta)){
                        if(distance <= Config.toleranceXY){
                            check = true;
                        }
                        if(!Double.isNaN(targetPoint.theta) && Math.abs(errorTheta) > Config.toleranceR){
                            check = false;
                        }
                        if(check){
                            scH.resetSpeed();
                            scV.resetSpeed();
                            scR.resetSpeed();
                        }
                    }
                    else if(deltaDist < Config.targetR){
                        check = true;
                    }


                }

                lastPoint = targetPoint;
            }
            drive.driveRobotCentric(0,0,0);
        });
        pursuitThread.start();
        return this;
    }


    public boolean finished(){
        return !pursuitThread.isAlive();
    }

    public void kill() throws InterruptedException{
        kill = true;
        if (loc != null) {
            loc.stop();
        }
        if (pursuitThread != null && pursuitThread.isAlive()) {
            pursuitThread.interrupt();
            pursuitThread.join(500);
        }

        if (drive != null) {
            drive.driveRobotCentric(0, 0, 0);
        }
    }
}