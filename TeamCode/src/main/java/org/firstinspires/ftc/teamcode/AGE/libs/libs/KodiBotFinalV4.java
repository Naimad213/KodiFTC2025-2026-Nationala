package org.firstinspires.ftc.teamcode.AGE.libs.libs;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class KodiBotFinalV4 {
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
    public FlyWheelSpline flyWheelSpline;
    //public AriseSubSystem arise;

    /// OBIECTE PENTRU DETECTIE CULOARE / VISION
    public KodiVision vision;
    public KodiLimelight limelight;
    public String teamColor;
    public IMU imu;

    public KodiBotFinalV4(HardwareMap hardwareMap, String teamColor) {
        this.hardwareMap = hardwareMap;
        this.teamColor = teamColor;

        this.limelight = new KodiLimelight();
        this.imu = hardwareMap.get(IMU.class, "imu");

        if (this.teamColor.equals("RED")) {
            initRed();
        } else if (this.teamColor.equals("BLUE")) {
            initBlue();
        }
    }

    public void initRed() {
        lFMotor = new Motor(hardwareMap, "leftFront");
        rFMotor = new Motor(hardwareMap, "rightFront");
        lRMotor = new Motor(hardwareMap, "leftRear");
        rRMotor = new Motor(hardwareMap, "rightRear");

        lFMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rFMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        lRMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rRMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rFMotor.setInverted(false);

        drive = new MecanumDrive(lFMotor, rFMotor, lRMotor, rRMotor);

        intake = new IntakeSubsystem(hardwareMap);
        outtake = new OuttakeSubsystem(hardwareMap);
        servoSubSystem = new ServoSubSystem(hardwareMap);
        flyWheelSpline = new FlyWheelSpline();

        vision = new KodiVision(hardwareMap, limelight, teamColor, imu);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
    }

    public void initBlue() {
        lFMotor = new Motor(hardwareMap, "leftFront");
        rFMotor = new Motor(hardwareMap, "rightFront");
        lRMotor = new Motor(hardwareMap, "leftRear");
        rRMotor = new Motor(hardwareMap, "rightRear");


        lFMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rFMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        lRMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rRMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rFMotor.setInverted(false);
        drive = new MecanumDrive(lFMotor, rFMotor, lRMotor, rRMotor);

        intake = new IntakeSubsystem(hardwareMap);
        outtake = new OuttakeSubsystem(hardwareMap);
        servoSubSystem = new ServoSubSystem(hardwareMap);
        flyWheelSpline = new FlyWheelSpline();

        vision = new KodiVision(hardwareMap, limelight, teamColor, imu);

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

    public void driveWithVoltageCompensation(double x, double y, double r, double heading) {
        double currentVoltage = batteryVoltageSensor.getVoltage();
        if (currentVoltage < 10.5) {
            drive.driveFieldCentric(x, y, r, heading);
            return;
        }

        double voltageScale = 12.0 / currentVoltage;
        double scaledX = x * voltageScale;
        double scaledY = y * voltageScale;
        double scaledR = r * voltageScale;

        drive.driveFieldCentric(scaledX, scaledY, scaledR, heading);
    }

    public MecanumDrive getDriveSession() { return drive; }

    public void killSwitch() {
        intake.stop();
        outtake.stop();
        drive.driveFieldCentric(0, 0, 0, 0);

        if (vision != null) {
            vision.killSwitch();
        }
    }
}