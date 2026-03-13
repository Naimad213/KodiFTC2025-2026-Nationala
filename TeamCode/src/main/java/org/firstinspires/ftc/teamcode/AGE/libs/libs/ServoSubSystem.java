package org.firstinspires.ftc.teamcode.AGE.libs.libs;

import android.system.StructUtsname;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.sun.tools.javac.tree.DCTree;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class ServoSubSystem {

    HardwareMap hardwareMap;
    public ServoEx servoDreapta;
    
    public NormalizedColorSensor sensorOut ;

    public enum DetectedColor {GREEN, PURPLE, NOTHING}


    // fireLeft: 0 start, 0.3 shoot
    public final double SHOOT = 0.0;
    public final double STANDBY = 0.5;



    public ServoSubSystem(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        init();
    }

    public void init() {
        servoDreapta = new SimpleServo(hardwareMap , "servoDreapta" , 0 ,300 , AngleUnit.DEGREES);
        sensorOut = hardwareMap.get(NormalizedColorSensor.class, "sensorOut");
        sensorOut.setGain(5.0f);
    }

   public void fire(){
        updateInversion();
        servoDreapta.setPosition(SHOOT);
   }
    public void standby(){
        updateInversion();
        servoDreapta.setPosition(STANDBY);
    }
   public void updateInversion(){
        servoDreapta.setInverted(true);
   }
   
   public boolean readyToLaunch(){
       return getColor(sensorOut) == DetectedColor.GREEN || getColor(sensorOut) == DetectedColor.PURPLE;
   }
    public DetectedColor getColor(NormalizedColorSensor colorSensor) {
        NormalizedRGBA currentColor = colorSensor.getNormalizedColors();
        if (((OpticalDistanceSensor) colorSensor).getLightDetected() < 0.05) {
            return DetectedColor.NOTHING;
        }

        double red = currentColor.red, green = currentColor.green, blue = currentColor.blue;
        if (green > (red + blue) * 0.75) {
            return DetectedColor.GREEN;
        } else if ((red + blue) > green * 1.5) {
            return DetectedColor.PURPLE;
        }

        return DetectedColor.NOTHING;
    }
}