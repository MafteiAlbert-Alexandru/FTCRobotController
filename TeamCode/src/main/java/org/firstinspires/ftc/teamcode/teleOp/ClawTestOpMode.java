package org.firstinspires.ftc.teamcode.teleOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Config
public class ClawTestOpMode extends LinearOpMode {


    public static double claw = 0.5;
    // 0.43 - 0.54
    public static double pivot = 0.46;
    //    0.573 fata
    // 0.46 spate

    @Override
    public void runOpMode() throws InterruptedException {

        Servo clawServo, pivotServo;
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        pivotServo = hardwareMap.get(Servo.class, "pivotServo");

        clawServo.setDirection(Servo.Direction.REVERSE);

        waitForStart();
        while(opModeIsActive()){
        clawServo.setPosition(claw);
        pivotServo.setPosition(pivot);
        }
    }
}
