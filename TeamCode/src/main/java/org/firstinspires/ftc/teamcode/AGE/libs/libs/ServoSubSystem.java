package org.firstinspires.ftc.teamcode.AGE.libs.libs;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class ServoSubSystem {

    HardwareMap hardwareMap;
    public ServoEx fireLeft, fireRight, fireMid;

    // fireLeft: 0 start, 0.3 shoot
    public final double LEFT_START = 0.0;
    public final double LEFT_SHOOT = 0.3;

    // fireMid: 0 start, 0.35 shoot
    public final double MID_START = 0.0;
    public final double MID_SHOOT = 0.35;

    // fireRight: 0.45 start, 0.15 shoot
    public final double RIGHT_START = 0.5;
    public final double RIGHT_SHOOT = 0;

    public ServoSubSystem(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        init();
    }

    public void init() {

        fireLeft = new SimpleServo(hardwareMap, "fireRight", 0, 300, AngleUnit.DEGREES);
        fireMid = new SimpleServo(hardwareMap, "fireMid", 0, 300, AngleUnit.DEGREES);
        fireMid.setInverted(true);
        fireRight = new SimpleServo(hardwareMap, "fireLeft", 0, 300, AngleUnit.DEGREES);
        setAllStart();
    }

    public void setAllStart() {
        fireLeft.setPosition(LEFT_START);
        fireMid.setPosition(MID_START);
        fireRight.setPosition(RIGHT_START);
    }


    public void fireLeft() {
        fireLeft.setPosition(LEFT_SHOOT);
    }

    public void fireMid() {
        fireMid.setPosition(MID_SHOOT);
    }

    public void fireRight() {
        fireRight.setPosition(RIGHT_SHOOT);
    }

    public void resetLeft()  { fireLeft.setPosition(LEFT_START); }
    public void resetMid()   { fireMid.setPosition(MID_START); }
    public void resetRight() { fireRight.setPosition(RIGHT_START); }
}