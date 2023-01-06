package org.firstinspires.ftc.teamcode.teleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Config
public class ControlHubSpielen extends LinearOpMode {

    public static double powerDc;
    public static double servoPos;
    public static boolean usePhoton = false;

    DcMotor Motor0, Motor1, Motor2, Motor3;

    Servo Servo0,Servo1,Servo2,Servo3,Servo4,Servo5;

    @Override
    public void runOpMode() throws InterruptedException {

        if(usePhoton) PhotonCore.enable();
        else PhotonCore.disable();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Motor0 = hardwareMap.get(DcMotor.class, "motor0");
//        Motor1 = hardwareMap.get(DcMotor.class, "motor1");
//        Motor2 = hardwareMap.get(DcMotor.class, "motor2");
//        Motor3 = hardwareMap.get(DcMotor.class, "motor3");

        Servo0 = hardwareMap.get(Servo.class, "servo0");
//        Servo1 = hardwareMap.get(Servo.class, "servo1");
//        Servo2 = hardwareMap.get(Servo.class, "servo2");
//        Servo3 = hardwareMap.get(Servo.class, "servo3");
//        Servo4 = hardwareMap.get(Servo.class, "servo4");
//        Servo5 = hardwareMap.get(Servo.class, "servo5");

        Motor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        Motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        Motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        Motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Motor0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        Motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        Motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        Motor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        while (opModeIsActive()) {

            Motor0.setPower(powerDc);
//            Motor1.setPower(powerDc);
//            Motor2.setPower(powerDc);
//            Motor3.setPower(powerDc);

            setServoPosition(servoPos);

            TelemetryManager();

        }

    }

    void TelemetryManager(){
        telemetry.addData("Motor0 pos", Motor0.getCurrentPosition());
//        telemetry.addData("Motor1 pos", Motor1.getCurrentPosition());
//        telemetry.addData("Motor2 pos", Motor2.getCurrentPosition());
//        telemetry.addData("Motor3 pos", Motor3.getCurrentPosition());
        telemetry.update();
    }

    void setServoPosition(double pos){
        Servo0.setPosition(pos);
//        Servo1.setPosition(pos);
//        Servo2.setPosition(pos);
//        Servo3.setPosition(pos);
//        Servo4.setPosition(pos);
//        Servo5.setPosition(pos);
    }
}
