package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

public class SensorSubsystem extends SmartSubsystem{

    private DistanceSensor  frontDistance, backDistance, coneDistance;

    @Override
    public void initSubsystem(OpMode opMode) {
        super.initSubsystem(opMode);

//        frontDistance = hardwareMap.get(DistanceSensor.class, "frontColor");
//        backDistance = hardwareMap.get(DistanceSensor.class, "backColor");
//        coneDistance = hardwareMap.get(DistanceSensor.class, "coneColor");

    }
    public boolean coneIsLoaded()
    {
        return false;//coneDistance.getDistance(DistanceUnit.MM) < 60;
    }
    public boolean toFlip() {
        return false;
//        return frontDistance.getDistance(DistanceUnit.MM) < 100 && backDistance.getDistance(DistanceUnit.MM) < 100;
    }
    public String formatRGB(int red, int green, int blue) {
        return String.format("#%02X%02X%02X", red, green, blue);
    }
    @Override
    public void run(SubsystemData data) throws InterruptedException {

    }
}
