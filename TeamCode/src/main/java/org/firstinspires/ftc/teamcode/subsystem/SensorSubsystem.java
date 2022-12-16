package org.firstinspires.ftc.teamcode.subsystem;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
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
        rightDistance=hardwareMap.get(DistanceSensor.class, "rightDistance");
        frontDistance=hardwareMap.get(DistanceSensor.class, "frontColor");
        backDistance=hardwareMap.get(DistanceSensor.class, "backColor");
        coneDistance=hardwareMap.get(DistanceSensor.class, "coneColor");
        frontColor=hardwareMap.get(ColorSensor.class, "frontColor");
        backColor=hardwareMap.get(ColorSensor.class, "backColor");
        coneColor=hardwareMap.get(ColorSensor.class, "coneColor");

    }

    @Override
    public void run(SubsystemData data) throws InterruptedException {
        TelemetryPacket packet = new TelemetryPacket();

        opMode.telemetry.addData("rightDistance", rightDistance.getDistance(DistanceUnit.MM));
        opMode.telemetry.addData("frontDistance", frontDistance.getDistance(DistanceUnit.MM));
        opMode.telemetry.addData("backDistance", backDistance.getDistance(DistanceUnit.MM));
        opMode.telemetry.addData("coneDistance", coneDistance.getDistance(DistanceUnit.MM));
        float hsv[] = new float[3];

        Color.RGBToHSV(frontColor.red(), frontColor.green(), frontColor.blue(),hsv);
        packet.fieldOverlay()
                .setFill("hsl("+hsv[0]+", "+hsv[1]+", "+hsv[2]+")")
                .fillRect(-20, -20, 20, 20);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);

        Color.RGBToHSV(backColor.red(), backColor.green(), backColor.blue(),hsv);
        packet.fieldOverlay()
                .setFill("hsl("+hsv[0]+", "+hsv[1]+", "+hsv[2]+")")
                .fillRect(0, -20, 20, 20);;
        Color.RGBToHSV(coneColor.red(), coneColor.green(), coneColor.blue(),hsv);
        packet.fieldOverlay()
                .setFill("hsl("+hsv[0]+", "+hsv[1]+", "+hsv[2]+")")
                .fillRect(20, -20, 20, 20);;
    }
}
