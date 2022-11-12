package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.apache.commons.math3.geometry.euclidean.twod.Line;
import org.firstinspires.ftc.teamcode.hardware.customHardware.ToggleButton;


@Config
public class IntakeSubsystem extends SmartSubsystem {
    public Motor leftIntake, rightIntake;

    public static double power = 0.65;
    public void run(ToggleButtonReader button)
    {
        button.readValue();
        if(button.getState())
        {
            leftIntake.set(power);
            rightIntake.set(power);
        }else {
            leftIntake.set(0);
            rightIntake.set(0);
        }
    }

    @Override
    public void initSubsystem(LinearOpMode opMode, HardwareMap hardwareMap) {
        super.initSubsystem(opMode, hardwareMap);
        leftIntake=new Motor(hardwareMap, "leftIntake", Motor.GoBILDA.RPM_1620);
        leftIntake.setInverted(true);
        rightIntake=new Motor(hardwareMap, "rightIntake", Motor.GoBILDA.RPM_1620);
    }
}