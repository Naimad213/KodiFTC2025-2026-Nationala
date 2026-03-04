package org.firstinspires.ftc.teamcode.AGE.libs.libs;

import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;

public class    IntakeSubsystem {



    private Motor intakeMotor;
    public CRServo servoRight;
    public CRServo servoLeft;

    public CRServo servoIntake;

    HardwareMap hardwareMap;


    Thread intakeOn;
    public IntakeSubsystem(HardwareMap hardwareMap ) {
        this.hardwareMap=hardwareMap;

        init();
    }
    public void init(){
        intakeMotor = new Motor(hardwareMap, "intakeM");
        servoRight = new CRServo(hardwareMap, "CRservoR");
        servoLeft = new CRServo(hardwareMap, "CRservoL");
        servoIntake = new CRServo(hardwareMap,"upLift");
        servoLeft.setInverted(true);
    }



    public void reverseIntake() {
        intakeMotor.set(-1.0);
        servoLeft.set(-1.0);
        servoRight.set(-1.0);
        servoIntake.set(1.0); // Keep lift direction pushing away
    }

    public void update(double leftTrigger, double rightTrigger) {
        double intakePower = rightTrigger - leftTrigger;

        intakeMotor.set(intakePower);
        servoLeft.set(-intakePower);
        servoRight.set(-intakePower);
        servoIntake.set(intakePower);
    }

    public void upLift(){
        servoIntake.set(-0.2);
        intakeMotor.set(0.5);
    }
    public void downLift(){
        servoIntake.set(1);
        intakeMotor.set(-0.6);
    }
    public void runPostShootSequence() {
        servoIntake.set(-1);
        intakeMotor.set(-0.6);
        servoLeft.set(1);
        servoRight.set(1);
    }
    public void stop() {
        intakeMotor.set(0);
        servoLeft.set(0);
        servoRight.set(0);
        servoIntake.set(0);
    }













}
