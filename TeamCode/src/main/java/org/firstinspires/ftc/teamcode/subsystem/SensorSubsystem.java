package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class SensorSubsystem extends SmartSubsystem{

    private DistanceSensor rightDistance, frontDistance, backDistance, coneDistance;
    private ColorSensor frontColor, backColor, coneColor;

    @Override
    public void initSubsystem(OpMode opMode) {
        super.initSubsystem(opMode);
        //rightDistance=hardwareMap.get(DistanceSensor.class, "rightDistance");
        frontDistance=hardwareMap.get(DistanceSensor.class, "frontColor");
        backDistance=hardwareMap.get(DistanceSensor.class, "backColor");
        coneDistance=hardwareMap.get(DistanceSensor.class, "coneColor");
        frontColor=hardwareMap.get(ColorSensor.class, "frontColor");
        backColor=hardwareMap.get(ColorSensor.class, "backColor");
        coneColor=hardwareMap.get(ColorSensor.class, "coneColor");

    }
    public boolean coneIsLoaded()
    {
        return coneDistance.getDistance(DistanceUnit.MM) < 60;
    }
    public boolean toFlip()
    {
        return frontDistance.getDistance(DistanceUnit.MM) <100 && backDistance.getDistance(DistanceUnit.MM) <100;
    }
    public String formatRGB(int red, int green, int blue)
    {
        return String.format("#%02X%02X%02X", red, green,blue);
    }
    @Override
    public void run(SubsystemData data) throws InterruptedException {

    }
}
