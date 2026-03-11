package org.firstinspires.ftc.teamcode.AGE.libs.testing_systems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import org.firstinspires.ftc.teamcode.AGE.libs.libs.SortSubsystem;

import java.util.ArrayList;
import java.util.List;

@TeleOp
public class ColorSensorTest extends LinearOpMode {
     NormalizedColorSensor fireLeft, fireMid,fireRight;

    public enum DetectedColor {GREEN, PURPLE, NOTHING}


    @Override
    public void runOpMode() {
        telemetry= new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        fireLeft = hardwareMap.get(NormalizedColorSensor.class, "sensorRight");
        fireLeft.setGain(2.0f); // Start with 2.0, try higher (e.g., 5.0 or 10.0) if still dark
        fireMid = hardwareMap.get(NormalizedColorSensor.class, "sensorMid");
        fireMid.setGain(5.0f); // Start with 2.0, try higher (e.g., 5.0 or 10.0) if still dark
        fireRight = hardwareMap.get(NormalizedColorSensor.class, "sensorLeft");
        fireRight.setGain(5.0f); // Start with 2.0, try higher (e.g., 5.0 or 10.0) if still dark

        waitForStart();

        while (opModeIsActive()) {


            telemetry.addData("left :",getColor(fireLeft).toString());
            telemetry.addData("right :", getColor(fireRight).toString());
            telemetry.addData("mid :", getColor(fireMid).toString());


            telemetry.update();
        }
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


