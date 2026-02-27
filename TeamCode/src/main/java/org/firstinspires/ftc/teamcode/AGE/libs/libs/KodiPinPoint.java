package org.firstinspires.ftc.teamcode.AGE.libs.libs;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class KodiPinPoint {


    HardwareMap hardwareMap;
    public GoBildaPinpointDriver pinpoint;
    private double headingOffset = 0; // offset for field-centric reference


    public KodiPinPoint(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        init();

    }

    public void init() {
        configurePinPoint();
    }

    public void configurePinPoint() {
        headingOffset = 0;
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pinpoint.setOffsets(0, 0 , DistanceUnit.MM);
        pinpoint.resetPosAndIMU();
    }

    public void reset() {
        pinpoint.resetPosAndIMU();
        headingOffset = 0;
    }

    public void zeroHeading() {
        // call this when you want the current facing direction to be considered "0°" field heading
        headingOffset = getRawHeading();
    }

    private double getRawHeading() {
        // raw heading from IMU (may be negative or >360)
        return pinpoint.getHeading(AngleUnit.DEGREES);
    }

    public double getHeading() {
        // field-centric heading, corectat pentru offset
        double heading = getRawHeading() - headingOffset;

        // normalizare interval (0,360]
        heading = (heading % 360 + 360) % 360;
        return heading;
    }

    public void setPosition (Pose2D pose){
        pinpoint.setPosition(pose);
    }
    public Pose2D getPosition () {
        return pinpoint.getPosition();
    }

    public void update () {
        pinpoint.update();
    }


        // Configure the sensor

    }




