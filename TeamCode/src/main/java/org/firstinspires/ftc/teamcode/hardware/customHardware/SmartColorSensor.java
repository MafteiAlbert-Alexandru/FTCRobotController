package org.firstinspires.ftc.teamcode.hardware.customHardware;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.arcrobotics.ftclib.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

public class SmartColorSensor implements HardwareDevice {

    int relativeLayoutId;
    View relativeLayout;

    ColorSensor sensorColor;
    DistanceSensor sensorDistance;
    HardwareMap hardwareMap;
    Telemetry telemetry;

    public SmartColorSensor(HardwareMap hardwareMap, String name, Telemetry telemetry){
        sensorColor = hardwareMap.get(ColorSensor.class, "sensor_color_distance");

        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_color_distance");
        relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);
    }

    public SmartColorSensor(HardwareMap hardwareMap, String name){
        new SmartColorSensor(hardwareMap, name, null);
    }

        float[] hsvValues = {0F, 0F, 0F};

        final float[] values = hsvValues;

        final double SCALE_FACTOR = 255;

        public void update() {

            Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                    (int) (sensorColor.green() * SCALE_FACTOR),
                    (int) (sensorColor.blue() * SCALE_FACTOR),
                    hsvValues);

            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
                }
            });

            TelemetryManger();
        }

        public void stop(){
            // Set the panel back to the default color
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.WHITE);
                }
            });
        }

    @Override
    public void disable() {

    }

    @Override
    public String getDeviceType() {
        return null;
    };

    public void TelemetryManger(){
        if(telemetry == null) return;

        telemetry.addData("Distance (cm)",
                String.format(Locale.US, "%.02f", sensorDistance.getDistance(DistanceUnit.CM)));
        telemetry.addData("Alpha", sensorColor.alpha());
        telemetry.addData("Red  ", sensorColor.red());
        telemetry.addData("Green", sensorColor.green());
        telemetry.addData("Blue ", sensorColor.blue());
        telemetry.addData("Hue", hsvValues[0]);

    }

}