package org.firstinspires.ftc.teamcode.AGE.libs.libs;


import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class ServoSubSystem {

    HardwareMap hardwareMap;
     public ServoEx lever;

    public final double LEVER_UP_POSITION = Position.LEVER_UP.val; // Is 1.0 from your enum
    public final double LEVER_DOWN_POSITION = Position.LEVER_DOWN.val; // Is 0.0 from your en


    public enum Position {


        LEVER_DOWN(0),
        LEVER_UP(0.33);




        public final double val;
        Position(double val) {
            this.val = val;
        }
    }

    public ServoSubSystem(HardwareMap hardwareMap ) {
       this.hardwareMap=hardwareMap;
       init();
    }


    public void init(){
        lever = new SimpleServo(hardwareMap, "lift", 0, 300,
                AngleUnit.DEGREES);

    }

    public void setLeverDown() {
        lever.setPosition(LEVER_DOWN_POSITION);
    }
    public void setLeverUp() {
        lever.setPosition(LEVER_UP_POSITION);
    }




}
