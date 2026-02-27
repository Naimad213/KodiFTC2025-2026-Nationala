package org.firstinspires.ftc.teamcode.AGE.libs.libs;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.AGE.libs.libs.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.AGE.libs.libs.KodiPinPoint;
import org.firstinspires.ftc.teamcode.AGE.libs.libs.ServoSubSystem;

public class KodiBotFinalV3 {
    HardwareMap hardwareMap;

    public MecanumDrive drive;
    public VoltageSensor batteryVoltageSensor;



    /// MOTOARE SASIU

    public Motor lFMotor, lRMotor, rFMotor, rRMotor;


    /// SUBSISTEME
    public KodiPinPoint pinPoint;

    public IntakeSubsystem intake;

    public OuttakeSubsystem outtake;

    public ServoSubSystem servoSubSystem;
    //public AriseSubSystem arise;

    /// OBIECTE PENTRU DETECTIE CULOARE / VISION
    public NormalizedRGBA colors,colorsIntake;
    public NormalizedColorSensor colorSensor,colorIntake;

    public KodiVision vision;





    public KodiBotFinalV3(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        init();
    }

    public void init() {
        lFMotor = new Motor(hardwareMap, "leftFront");
        rFMotor = new Motor(hardwareMap, "rightFront");
        lRMotor = new Motor(hardwareMap, "leftRear");
        rRMotor = new Motor(hardwareMap, "rightRear");


        lFMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rFMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        lRMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rRMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rRMotor.setInverted(false);

        drive = new MecanumDrive(lFMotor,rFMotor,lRMotor,rRMotor);

        colorIntake= hardwareMap.get(NormalizedColorSensor.class, "sensorIntake");
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
        colorSensor.setGain(10.0f);


        intake =new IntakeSubsystem(hardwareMap ,colors,colorsIntake);
        outtake = new OuttakeSubsystem(hardwareMap);
        servoSubSystem = new ServoSubSystem(hardwareMap);
        //arise= new AriseSubSystem(hardwareMap);

        // vision = new KodiVision(hardwareMap);


        pinPoint = new KodiPinPoint(hardwareMap);
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();


    }


    public void driveWithVoltageCompensation(double x, double y, double r) {
        double currentVoltage = batteryVoltageSensor.getVoltage();
        if (currentVoltage < 10.5) {
            drive.driveRobotCentric(x, y, r);
            return;
        }

        double voltageScale = 12.0 / currentVoltage;


        double scaledX = x * voltageScale;
        double scaledY = y * voltageScale;
        double scaledR = r * voltageScale;

        drive.driveRobotCentric(scaledX, scaledY, scaledR);
    }
    public void driveWithVoltageCompensation(double x, double y, double r,double heading) {
        double currentVoltage = batteryVoltageSensor.getVoltage();
        /// aici se opreste deja din cauza sdk ului
        if (currentVoltage < 10.5) {
            drive.driveFieldCentric(x, y, r, heading);
            return;
        }

        double voltageScale = 12.0 / currentVoltage;

        /// valori finale
        double scaledX = x * voltageScale;
        double scaledY = y * voltageScale;
        double scaledR = r * voltageScale;

        drive.driveFieldCentric(scaledX, scaledY, scaledR, heading);
    }


    public MecanumDrive getDriveSession() { return drive ;}
    public void killSwitch(){

        intake.stop();
        outtake.stop();
        drive.driveFieldCentric(0,0,0,0);
    }
}
