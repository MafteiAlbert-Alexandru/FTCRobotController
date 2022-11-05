package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

public class Hardware_Map {

    HardwareMap hardwareMap;

    Hardware_Map(HardwareMap hardwareMap){this.hardwareMap = hardwareMap;}

    int getCameraMonitorViewId(){return hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());}
    WebcamName getWebcamName(){return hardwareMap.get(WebcamName .class, "webcam");}
}
