package org.firstinspires.ftc.teamcode.AGE.libs.libs;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;


/**
 * The SortSubsystem class manages the robot's color sensors and implements logic to identify
 * and sequence game elements based on detected colors and AprilTag patterns.
 *
 * <p>It provides functionality to:</p>
 * <ul>
 *   <li>Initialize and configure three color sensors (Left, Middle, Right).</li>
 *   <li>Identify colors (GREEN, PURPLE, or NOTHING) based on normalized RGBA values and proximity.</li>
 *   <li>Decode specific AprilTag IDs into predefined color patterns (GPP, PGP, PPG).</li>
 *   <li>Determine an optimal firing or sorting sequence by matching currently detected colors
 *       against a target pattern.</li>
 * </ul>
 *
 * @author Kodikas Robotics
 */
public class SortSubsystem {

    public NormalizedColorSensor BLeft, BRight, MidSensor ;
    HardwareMap hardwareMap;

    public enum DetectedColor {GREEN, PURPLE, NOTHING}
    public enum Pattern {GPP, PGP, PPG, UNKNOWN}

    public boolean allMatched=false;

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



        NormalizedColorSensor[] secventaTragere = new NormalizedColorSensor[3];
        int sequenceIndex = 0;


        /// boolean-uri pentru a verifica daca e valabil spot ul respectiv
        boolean leftUsed = false;
        boolean midUsed = false;
        boolean rightUsed = false;

        /// incercam sa vedem prin fiecare daca se potriveste cu patternu
        ///  apoi il punem in ordine
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
         allMatched = leftUsed && midUsed && rightUsed;
        if (allMatched) return secventaTragere;
        /// safety check daca nu s folosite toate sa i dea check out
        if (sequenceIndex < 3) {
            if (!leftUsed) secventaTragere[sequenceIndex++] = BLeft;
            if (sequenceIndex < 3 && !midUsed) secventaTragere[sequenceIndex++] = MidSensor;
            if (sequenceIndex < 3 && !rightUsed) secventaTragere[sequenceIndex++] = BRight;
        }
        return secventaTragere;

    }
}