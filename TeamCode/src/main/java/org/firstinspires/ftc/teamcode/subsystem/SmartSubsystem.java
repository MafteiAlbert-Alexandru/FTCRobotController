package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;


public abstract class SmartSubsystem {


    public LinearOpMode opMode;
    public HardwareMap hardwareMap;
    public boolean initialized;
    public abstract void run(SubsystemData data) throws InterruptedException;

    public void initSubsystem(LinearOpMode linearOpMode, HardwareMap hardwareMap) {
        opMode=linearOpMode;
        this.hardwareMap=hardwareMap;
        this.initialized=true;
    }
}

