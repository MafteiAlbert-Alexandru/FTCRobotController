package org.firstinspires.ftc.teamcode.experiments;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.ClampSubsystem;


@TeleOp
public class ClampTester extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        ClampSubsystem clampSubsystem = new ClampSubsystem();
        clampSubsystem.initSubsystem(this, hardwareMap);
        waitForStart();

        while (opModeIsActive()){
            clampSubsystem.update();

            if(gamepad1.dpad_left) clampSubsystem.goToBackward();
            else if(gamepad1.dpad_right) clampSubsystem.goToForward();
            if(gamepad1.dpad_up) clampSubsystem.clamp();
            else if(gamepad1.dpad_down) clampSubsystem.release();

        }
    }
}

