package org.firstinspires.ftc.teamcode.AGE.libs.libs;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class OuttakeSubsystem {


    HardwareMap hardwareMap;
    public DcMotorEx outtakeM1,outtakeM2;
    PIDFCoefficients pidfCoefficients;
    private double currentTargetVelocity = 0;

    public FlyWheelSpline flyWheelSpline;
    public double TARGET_RPM=0;


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

    public void update(boolean b,double distance) {

        TARGET_RPM=flyWheelSpline.getTargetRPM(distance);
        currentTargetVelocity = TARGET_RPM;
        pidfCoefficients = new PIDFCoefficients(0,0,1,0);
        outtakeM2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        outtakeM1.setVelocity(-currentTargetVelocity);
        outtakeM1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        outtakeM2.setVelocity(-currentTargetVelocity);

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
        return (outtakeM1.getVelocity()+ outtakeM2.getVelocity())/2 ;
    }
    public boolean readyToShoot() {
        double v1 = outtakeM1.getVelocity();
        double v2 = outtakeM2.getVelocity();
        double ActualVelocity = Math.abs(v1) + Math.abs(v2);
        return Math.abs(Math.abs(currentTargetVelocity) - ActualVelocity) < 300;
    }





}
