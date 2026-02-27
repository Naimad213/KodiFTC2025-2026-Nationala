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

    private boolean isGreenBallUp = false;
    private List<String> ballStorage = new ArrayList<>();
    private String lastDetectedColor = "NOTHING";
    public enum SortingPattern { PPG, PGP, GPP }
    private SortingPattern currentPattern = SortingPattern.PPG;
    private boolean isBallUp = false;
    private boolean secondBallPassed = false;
    private boolean gppGreenDetected = false;
    private ElapsedTime pgpDropTimer = new ElapsedTime();
    private boolean isPgpDropping = false;

    private Motor intakeMotor;
    public CRServo servoRight;
    public CRServo servoLeft;

    public CRServo servoIntake;

    NormalizedRGBA colors,colorsIntake;
    NormalizedColorSensor colorSensor, colorIntake;

    HardwareMap hardwareMap;

    private ElapsedTime detectionCooldown = new ElapsedTime();
    private final double COOLDOWN_SECONDS = 0.5; // Adjust based on how fast balls move
    private boolean ballInSensorRange = false;

    Thread intakeOn;
    public IntakeSubsystem(HardwareMap hardwareMap , NormalizedRGBA colors ,NormalizedRGBA colorsIntake) {
        this.hardwareMap=hardwareMap;
        this.colors=colors;
        this.colorsIntake=colorsIntake;
        init();
    }
    public void init(){
        intakeMotor = new Motor(hardwareMap, "intakeM");
        servoRight = new CRServo(hardwareMap, "CRservoR");
        servoLeft = new CRServo(hardwareMap, "CRservoL");
        servoIntake = new CRServo(hardwareMap,"upLift");
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
        colorIntake = hardwareMap.get(NormalizedColorSensor.class, "sensorIntake");
        colorSensor.setGain(15.0f);
        colorIntake.setGain(15.0f);
        servoLeft.setInverted(true);
    }

    public void runIntake(double power) {
        intakeMotor.set(power);
        servoLeft.set(power);
        servoRight.set(power);
        servoIntake.set(power);
    }
    public void runCRServo (double power) {
        servoLeft.set(power);
        servoRight.set(power);
        servoIntake.set(power);
    }


    public void setPattern(SortingPattern pattern) {
        this.currentPattern = pattern;
    }

    public SortingPattern getPattern() {
        return currentPattern;
    }

    /**
     * Cycles through the sorting patterns: PPG -> PGP -> GPP -> PPG.
     * @param direction 1 for next pattern, -1 for previous pattern.
     */
    public void cyclePattern(int direction) {
        SortingPattern[] patterns = SortingPattern.values();
        int currentIndex = currentPattern.ordinal();

        // Calculate new index with wrap-around protection
        int newIndex = (currentIndex + direction + patterns.length) % patterns.length;
        currentPattern = patterns[newIndex];

        // Important: Reset sorting state when changing patterns to avoid jams
        resetSort();
    }



    public void updateWithSort(String detectedIntake, String detectedOnLift, double rightTrigger, double leftTrigger) {
        boolean nothing = !detectedOnLift.equals("NOTHING");
        boolean greenOnLift = detectedOnLift.equals("GREEN");
        boolean purpleOnLift = detectedOnLift.equals("PURPLE");

        // --- PPG LOGIC ---
        if (currentPattern == SortingPattern.PPG) {
            if (detectedIntake.equals("GREEN")) {
                ElapsedTime waitTime = new ElapsedTime();
                waitTime.reset();
                int tries = 2;
                while (tries--> 0) {
                    if (!isBallUp && waitTime.seconds() > 0.3) {
                        upLift();
                        isBallUp = true;
                    } else {
                        reverseIntake(); // Reject extra green
                    }
                 if (detectedIntake.equals("PURPLE")) {
                    downLift(); // Always take purple through
                } else {
                    update(rightTrigger, leftTrigger);
                }
            }

        }
            }

        // --- PGP LOGIC ---
        else if (currentPattern == SortingPattern.PGP) {
            // Handle the timed Green drop sequence
            if (purpleOnLift && greenOnLift && !isPgpDropping) {
                pgpDropTimer.reset();
                isPgpDropping = true;
            }

            if (isPgpDropping) {
                if (pgpDropTimer.seconds() < 0.5) {
                    downLift(); // Send it down
                } else {
                    downLift(); // Afterwards send it through (continues down)
                    isPgpDropping = false;
                    isBallUp = false; // Spot cleared
                }
                return;
            }

            if (!purpleOnLift) {
                // Initial state: Take both, Green Up, Purple Through
                if (detectedIntake.equals("GREEN")) {
                    if (!isBallUp) {
                        upLift();
                        isBallUp = true;
                    } else {
                        reverseIntake(); // Reject green if one is already up
                    }
                } else if (detectedIntake.equals("PURPLE")) {
                    downLift();
                } else {
                    update(rightTrigger, leftTrigger);
                }
            } else {
                // Purple is on lift
                if (detectedIntake.equals("PURPLE")) {
                    upLift(); // Purple Up
                    isBallUp = true;
                } else if (detectedIntake.equals("GREEN")) {
                    downLift(); // Green Through
                } else {
                    update(rightTrigger, leftTrigger);
                }
            }
        }

        // --- GPP LOGIC ---
        else if (currentPattern == SortingPattern.GPP) {
            if (!greenOnLift) {
                // Phase 1: Only Purple Up, Reject extra Purples if one is Up
                if (detectedIntake.equals("PURPLE")) {
                    if (!purpleOnLift) {
                        upLift();
                    } else {
                        reverseIntake(); // Only accept Green if Purple is already up
                    }
                } else if (detectedIntake.equals("GREEN")) {
                    upLift(); // Accept green (this transitions to phase 2)
                } else {
                    update(rightTrigger, leftTrigger);
                }
            } else {
                // Phase 2: Green on lift, only accept Purples through
                if (detectedIntake.equals("PURPLE")) {
                    downLift();
                } else if (detectedIntake.equals("GREEN")) {
                    reverseIntake(); // Reject extra greens
                } else {
                    update(rightTrigger, leftTrigger);
                }
            }
        }
    }

    public void reverseIntake() {
        // Spins all intake motors in the opposite direction to eject balls
        intakeMotor.set(-1.0);
        servoLeft.set(-1.0);
        servoRight.set(-1.0);
        servoIntake.set(1.0); // Keep lift direction pushing away
    }
    public void resetSort() {
        isBallUp = false;
        downLift();
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



    public void updateColor() {
        colorSensor.getNormalizedColors();
    }

    public String getColor ()
    {
        if (((OpticalDistanceSensor) colorSensor).getLightDetected() < 0.05) {
            return "NOTHING";
        }

        else {
            // purple : red + blue > green
            // green : red + blue < green

            double red = colors.red;
            double green = colors.green;
            double blue = colors.blue;

            // check green
            if (green > (red + blue) * 0.75) {
                return "GREEN";
            }
            // check purple
            else if ((red + blue) > green * 1.5) {
                return "PURPLE";
            }
            else {
                return "NOTHING";
            }
        }

    }

    public String getColorIntake() {
        colorsIntake = colorIntake.getNormalizedColors();        if (((OpticalDistanceSensor) colorIntake).getLightDetected() < 0.05) {
            return "NOTHING";
        } else {
            double red = colorsIntake.red;
            double green = colorsIntake.green;
            double blue = colorsIntake.blue;

            if (green > (red + blue) * 0.75) return "GREEN";
            else if ((red + blue) > green * 1.5) return "PURPLE";
            else return "NOTHING";
        }
    }


    public void updateBallStorage() {
        String currentColor = getColorIntake();
        boolean isSeeingBall = !currentColor.equals("NOTHING");

        // 1. Check if we are currently in a cooldown period
        if (detectionCooldown.seconds() < COOLDOWN_SECONDS) {
            lastDetectedColor = currentColor;
            return;
        }

        // 2. Detect the "Rising Edge" (Transition from NOTHING to a COLOR)
        if (isSeeingBall && lastDetectedColor.equals("NOTHING")) {
            if (ballStorage.size() < 3) {
                ballStorage.add(currentColor);
                // 3. Start the timer to ignore subsequent flickers
                detectionCooldown.reset();
            }
        }

        lastDetectedColor = currentColor;
    }

    public void removeBall() {
        if (!ballStorage.isEmpty()) {
            ballStorage.remove(0);
        }
    }
    public void clearStorage() {
        ballStorage.clear();
    }

    public List<String> getBallStorage() {
        return ballStorage;
    }



}
