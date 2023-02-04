package org.firstinspires.ftc.teamcode.teleOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Config
public class ClawTestOpMode extends LinearOpMode {

    public static double clawOpen = 0.135;
    public static double clawWaiting = 0.25;
    public static double clawClose = 0;
    public static double pivotFront = 1;
    public static double pivotBack = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        Servo claw, pivot;

        boolean isFront = false;

        claw = hardwareMap.get(Servo.class, "claw");
        pivot = hardwareMap.get(Servo.class, "pivot");

        claw.setDirection(Servo.Direction.REVERSE);

        waitForStart();
//        claw.setPosition(0);
        pivot.setPosition(0);
        while(opModeIsActive()){
        claw.setPosition(clawClose);
        pivot.setPosition(pivotBack);
        }
    }
}
