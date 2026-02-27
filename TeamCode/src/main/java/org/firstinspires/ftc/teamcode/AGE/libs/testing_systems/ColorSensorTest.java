package org.firstinspires.ftc.teamcode.AGE.libs.testing_systems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import java.util.ArrayList;
import java.util.List;

@TeleOp
public class ColorSensorTest extends LinearOpMode {
     NormalizedColorSensor colorSensor, colorSensorIntake;

     NormalizedRGBA colorsIntake;
    private List<String> ballStorage = new ArrayList<>();

    public String lastDetectedColor = "NOTHING";


    @Override
    public void runOpMode() {

        colorSensorIntake = hardwareMap.get(NormalizedColorSensor.class, "sensorIntake");
        colorSensorIntake.setGain(15.0f); // Start with 2.0, try higher (e.g., 5.0 or 10.0) if still dark

        waitForStart();

        while (opModeIsActive()) {

            updateBallStorage();
            telemetry.addData("first :", ballStorage.get(0));
            telemetry.addData("second :", ballStorage.get(1));
            telemetry.addData("third :", ballStorage.get(2));


            telemetry.update();
        }
    }public String getColorIntake() {
        colorsIntake = colorSensorIntake.getNormalizedColors();   if (((OpticalDistanceSensor) colorSensorIntake).getLightDetected() < 0.05) {
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

        if (!currentColor.equals("NOTHING") && lastDetectedColor.equals("NOTHING")) {
            if (ballStorage.size() < 3) {
                ballStorage.add(currentColor);
            }
        }
        lastDetectedColor = currentColor;
    }

}


