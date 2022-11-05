package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.hardware.customHardware.SmartServo;

@TeleOp
public class CustomServo extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        SmartServo mySmartServo = new SmartServo(hardwareMap, "servo", 0, 300, AngleUnit.DEGREES);

        waitForStart();


        while (opModeIsActive()){

            if(gamepad1.left_bumper) mySmartServo.rotateByAngle(0.5);
            else if (gamepad1.right_bumper) mySmartServo.rotateByAngle(-0.5);

            if(gamepad1.dpad_up) mySmartServo.turnToAngle(0);
            else if(gamepad1.dpad_right) mySmartServo.turnToAngle(75);
            else if(gamepad1.dpad_down) mySmartServo.turnToAngle(150);
            else if(gamepad1.dpad_left) mySmartServo.turnToAngle(225);
            else if(gamepad1.touchpad) mySmartServo.turnToAngle(300);

        }
    }
}
