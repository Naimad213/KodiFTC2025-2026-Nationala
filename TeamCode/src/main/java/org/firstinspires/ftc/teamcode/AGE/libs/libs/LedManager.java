package org.firstinspires.ftc.teamcode.AGE.libs.libs;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LedManager {

    private DigitalChannel ledRed;    // Digital 2
    private DigitalChannel ledGreen;  // Digital 3

    public LedManager(HardwareMap hardwareMap) {
        //semn de intrebare la devicename
        ledRed = hardwareMap.get(DigitalChannel.class, "revLed1");
        ledGreen = hardwareMap.get(DigitalChannel.class, "revLed2");

        ledRed.setMode(DigitalChannel.Mode.OUTPUT);
        ledGreen.setMode(DigitalChannel.Mode.OUTPUT);


        ledRed.setState(false);
        ledGreen.setState(false);
    }


    public void redOn() {
        ledRed.setState(true);
        ledGreen.setState(false);
    }

    public void greenOn() {
        ledRed.setState(false);
        ledGreen.setState(true);
    }

    public void yellowOn() {
        ledRed.setState(true);
        ledGreen.setState(true);
    }

    public void allOff() {
        ledRed.setState(false);
        ledGreen.setState(false);
    }
}