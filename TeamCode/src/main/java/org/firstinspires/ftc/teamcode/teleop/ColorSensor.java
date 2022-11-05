package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp
@Config
public class ColorSensor extends LinearOpMode {

    public static boolean LedStatus = false; //Led activity

    public static boolean isCone = false;

    public static int redVal = 0;

    public static int blueVal = 0;

    public static int greenVal = 0;

    @Override
    public void runOpMode() throws InterruptedException {


        LynxI2cColorRangeSensor colorSensor = hardwareMap.get(LynxI2cColorRangeSensor.class, "color");

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        waitForStart();

        while (opModeIsActive()){

            colorSensor.enableLed(LedStatus);


            isCone = (colorSensor.red() > redVal || colorSensor.blue() > blueVal) && colorSensor.green()<greenVal;

            telemetry.addData("2_Red", colorSensor.red());
            telemetry.addData("1_Blue", colorSensor.blue());
            telemetry.addData("3_Green", colorSensor.green());
            telemetry.addData("Cone found", isCone);
            telemetry.update();

        }

    }
}
