package org.firstinspires.ftc.teamcode.AGE.libs.libs;


import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class ServoSubSystem {

    HardwareMap hardwareMap;
     public ServoEx fireLeft, fireRight, fireMid;

    public final double LEVER_UP_POSITION = Position.LEVER_UP.val;
    public final double LEVER_DOWN_POSITION = Position.LEVER_DOWN.val;

    ElapsedTime wait= new ElapsedTime();
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
        fireLeft = new SimpleServo(hardwareMap, "fireLeft", 0, 300,
                AngleUnit.DEGREES);
        fireMid = new SimpleServo(hardwareMap, "fireMid", 0, 300,
                AngleUnit.DEGREES);
        fireRight = new SimpleServo(hardwareMap, "fireRight", 0, 300,
                AngleUnit.DEGREES);

    }

   public void fireLeft(){
        wait.reset();
        fireLeft.setPosition(LEVER_UP_POSITION);
        if(wait.seconds()>0.5){
            fireLeft.setPosition(LEVER_DOWN_POSITION);
        }
   }
    public void fireMid(){
        wait.reset();
        fireMid.setPosition(LEVER_UP_POSITION);
        if(wait.seconds()>0.5){
            fireMid.setPosition(LEVER_DOWN_POSITION);
        }
    }
    public void fireRight(){
        wait.reset();
        fireRight.setPosition(LEVER_UP_POSITION);
        if(wait.seconds()>0.5){
            fireRight.setPosition(LEVER_DOWN_POSITION);
        }
    }




}
