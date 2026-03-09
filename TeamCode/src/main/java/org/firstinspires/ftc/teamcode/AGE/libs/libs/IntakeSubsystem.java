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

    HardwareMap hardwareMap;

    public IntakeSubsystem(HardwareMap hardwareMap ) {
        this.hardwareMap=hardwareMap;

        init();
    }
    public void init(){
        intakeMotor = new Motor(hardwareMap, "intakeM");
    }

    public void update(double leftTrigger, double rightTrigger) {
        double intakePower = rightTrigger - leftTrigger;
        intakeMotor.set(intakePower);
    }

    public void stop() {
        intakeMotor.set(0);

    }













}
