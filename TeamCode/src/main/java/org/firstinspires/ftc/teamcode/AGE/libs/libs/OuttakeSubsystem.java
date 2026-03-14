package org.firstinspires.ftc.teamcode.AGE.libs.libs;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class OuttakeSubsystem {


    HardwareMap hardwareMap;
    public DcMotorEx outtakeM1,outtakeM2;
    PIDFCoefficients pidfCoefficients;
    public double currentTargetVelocity = 150;

    public FlyWheelSpline flyWheelSpline;
    public double TARGET_RPM=0;

    public double lastP =0, lastF=0;


    public OuttakeSubsystem(HardwareMap hardwareMap) {
        this.hardwareMap=hardwareMap;
        init();
    }
    public void init(){
        outtakeM1 = hardwareMap.get(DcMotorEx.class, "outtakeM1");
        outtakeM1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        outtakeM1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        outtakeM2 = hardwareMap.get(DcMotorEx.class, "outtakeM2");
        outtakeM2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        outtakeM2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        pidfCoefficients = new PIDFCoefficients(0,0,0,0);
        flyWheelSpline= new FlyWheelSpline();
    }
    public void update(boolean b) {
        if(b) {
            currentTargetVelocity = 130;
            pidfCoefficients = new PIDFCoefficients(14, 0, 0, 14);
            outtakeM2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
            outtakeM1.setVelocity(-currentTargetVelocity);
            outtakeM1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
            outtakeM2.setVelocity(-currentTargetVelocity);
        }else{
            stop();
        }

    }
    public void update(boolean b,double distance) {
        if(b && distance!=0) {
            TARGET_RPM = flyWheelSpline.getTargetRPM(distance);
            currentTargetVelocity = TARGET_RPM;
            pidfCoefficients = new PIDFCoefficients(14, 0, 0, 14);
            outtakeM2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
            outtakeM1.setVelocity(-currentTargetVelocity);
            outtakeM1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
            outtakeM2.setVelocity(-currentTargetVelocity);
        }else{
            stop();
        }

    }
    public void updateTrain(boolean b, double targetRPM, double p, double f) {
        if (b) {
            currentTargetVelocity = targetRPM;

            // Only write to hardware if the values actually changed!
            if (p != lastP || f != lastF) {
                pidfCoefficients = new PIDFCoefficients(p, 0, 0, f);
                outtakeM1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
                outtakeM2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
                lastP = p;
                lastF = f;
            }

            // Set velocity AFTER coefficients
            outtakeM1.setVelocity(-currentTargetVelocity);
            outtakeM2.setVelocity(-currentTargetVelocity);
        } else {
            stop();
        }
    }



    public void stop() {
        pidfCoefficients.d = 0;
        pidfCoefficients.p = 0;
        pidfCoefficients.i = 0;
        outtakeM1.setVelocity(0);
        outtakeM2.setVelocity(0);
    }




    //telemetry
    public double getVelocity() {
        return Math.abs((outtakeM1.getVelocity()+ outtakeM2.getVelocity())/2) ;
    }
    public boolean readyToShoot() {
        boolean ready;
        double v1 = outtakeM1.getVelocity();
        double v2 = outtakeM2.getVelocity();
        double ActualVelocity = (Math.abs(v1) + Math.abs(v2))/2;
        ready= Math.abs(Math.abs(currentTargetVelocity) - ActualVelocity) < 149;
        return ready;
    }





}
