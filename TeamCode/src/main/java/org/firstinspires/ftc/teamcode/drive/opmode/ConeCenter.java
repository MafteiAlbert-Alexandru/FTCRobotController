package org.firstinspires.ftc.teamcode.drive.opmode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.SensorDistance;
import com.arcrobotics.ftclib.hardware.SensorDistanceEx;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.lang.reflect.Type;

@Config
@TeleOp
@Disabled
public class ConeCenter extends LinearOpMode {

    public static double leftDis = 10; //left distance
    public static double midDis = 10; //mid distance
    public static double rightDis = 10; //high distance
    public static double error = 0; //error
    public boolean centered = false;
    public String caseCone;

    double LeftDis, MidDis, RightDis;

    private DistanceSensor left, mid, right;

    @Override
    public void runOpMode() throws InterruptedException {

//        SensorDistance leftSensor, midSensor, rightSensor;
//        leftSensor = hardwareMap.get(SensorDistance.class, "sensor_left");
//        midSensor = hardwareMap.get(SensorDistance.class, "sensor_mid");
//        rightSensor = hardwareMap.get(SensorDistance.class, "sensor_right");
//
//        Rev2mDistanceSensor left = (Rev2mDistanceSensor)leftSensor;
//        Rev2mDistanceSensor mid = (Rev2mDistanceSensor)midSensor;
//        Rev2mDistanceSensor right = (Rev2mDistanceSensor)rightSensor;


        left = hardwareMap.get(DistanceSensor.class, "sensor_left");
        mid = hardwareMap.get(DistanceSensor.class, "sensor_mid");
        right = hardwareMap.get(DistanceSensor.class, "sensor_right");


        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        while (opModeIsActive()){

            LeftDis = left.getDistance(DistanceUnit.CM);
            MidDis = mid.getDistance(DistanceUnit.CM);
            RightDis = right.getDistance(DistanceUnit.CM);

            TelemetryManager();
            CaseCone();
        }
    }

    public void TelemetryManager(){
        telemetry.addData("leftDis", LeftDis);
        telemetry.addData("midDis", MidDis);
        telemetry.addData("rightDis", RightDis);
        telemetry.addData("Case of the Cone", caseCone);
        telemetry.update();
    }

    public void CaseCone(){
        if(isInRange(LeftDis, leftDis) && isInRange(MidDis, midDis) && isInRange(RightDis, rightDis)) caseCone = "Good";
        else caseCone = "Bad";
    }

    public boolean isInRange(double distance, double favDis){
        if(distance-error<favDis && favDis<distance+error) return true;
        else return false;
    }
}