package org.firstinspires.ftc.teamcode.AGE.libs.libs;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.ArrayList;

public class SortSubsystem {

    NormalizedColorSensor BLeft, BRight, MidSensor , mock;
    HardwareMap hardwareMap;

    public enum DetectedColor {GREEN, PURPLE, NOTHING}
    public enum Pattern {GPP, PGP, PPG, UNKNOWN}

    // Cleaned up constructor
    public SortSubsystem(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        init();
    }

    public void init() {
        BLeft = hardwareMap.get(NormalizedColorSensor.class, "BLeft");
        BRight = hardwareMap.get(NormalizedColorSensor.class, "BRight");
        MidSensor = hardwareMap.get(NormalizedColorSensor.class, "MidSensor");

        BLeft.setGain(15);
        BRight.setGain(15);
        MidSensor.setGain(15);
    }

    // Cleaned up getColor to prevent variables bleeding over
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

    public Pattern decodePattern(AprilTagDetection detection) {
        if (detection == null) return Pattern.UNKNOWN;
        int id = detection.id;
        if (id == 21) return Pattern.GPP;
        if (id == 22) return Pattern.PGP;
        if (id == 23) return Pattern.PPG;

        return Pattern.UNKNOWN;
    }

    public NormalizedColorSensor[] getLaunchSequence(AprilTagDetection detection) {
        Pattern pattern = decodePattern(detection);
        if(pattern == Pattern.UNKNOWN) return new NormalizedColorSensor[0];
        /// 1
        /// vedem fiecare sensor ce detecteaza
        DetectedColor leftColor = getColor(BLeft);
        DetectedColor midColor = getColor(MidSensor);
        DetectedColor rightColor = getColor(BRight);
        /// 2
        /// definim array-ul cu ordinea buna
        DetectedColor[] desiredOrder;
        switch (pattern) {
            case GPP:
                    desiredOrder = new DetectedColor[]{
                    DetectedColor.GREEN,
                    DetectedColor.PURPLE,
                    DetectedColor.PURPLE};
                    break;
            case PGP:
                    desiredOrder = new DetectedColor[]{
                    DetectedColor.PURPLE,
                    DetectedColor.GREEN,
                    DetectedColor.PURPLE};
                    break;
            case PPG:
                    desiredOrder = new DetectedColor[]{
                    DetectedColor.PURPLE,
                    DetectedColor.PURPLE,
                    DetectedColor.GREEN};
                    break;
            default:
                    desiredOrder = new DetectedColor[]{
                    DetectedColor.GREEN,
                    DetectedColor.PURPLE,
                    DetectedColor.PURPLE};
                    break;
        }



        NormalizedColorSensor[] secventaTragere = new NormalizedColorSensor[2];
        int sequenceIndex = 0;


        /// by default lever de pe pozitie nu l foloseste
        boolean leftUsed = false;
        boolean midUsed = false;
        boolean rightUsed = false;

        /// incercam sa vedem prin fiecare daca
        for (DetectedColor desiredColor : desiredOrder) {
            if (!leftUsed && leftColor == desiredColor) {
                secventaTragere[sequenceIndex++] = BLeft;
                leftUsed = true;
            } else if (!midUsed && midColor == desiredColor) {
                secventaTragere[sequenceIndex++] = MidSensor;
                midUsed = true;
            } else if (!rightUsed && rightColor == desiredColor) {
                secventaTragere[sequenceIndex++] = BRight;
                rightUsed = true;
            }
        }
        /// daca toate sunt ok atunci nu mai stam sa vedem care nu s pentru proccessing time
        boolean allMatched = leftUsed && midUsed && rightUsed;
        if (allMatched) return secventaTragere;
        else {
            /// le da dump
            if (!leftUsed) secventaTragere[sequenceIndex++] = BLeft;
            if (!midUsed) secventaTragere[sequenceIndex++] = MidSensor;
            if (!rightUsed) secventaTragere[sequenceIndex++] = BRight;
        }
        return secventaTragere;
    }
}