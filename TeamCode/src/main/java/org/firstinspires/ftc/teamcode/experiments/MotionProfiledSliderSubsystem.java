package org.firstinspires.ftc.teamcode.experiments;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.hardware.SmartMotor;
import org.firstinspires.ftc.teamcode.hardware.SmartMotorEx;
import org.firstinspires.ftc.teamcode.subsystem.SmartSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.SubsystemData;
import org.jetbrains.annotations.Nullable;

public class MotionProfiledSliderSubsystem extends SmartSubsystem {
    private SmartMotorEx slider;
    private PIDExController controller;
    public static PIDExCoefficients veloCoefficients = new PIDExCoefficients(1,1,1,1,1);


    @Override
    public void run(@Nullable SubsystemData data) throws InterruptedException {

    }



    @Override
    public void initSubsystem(OpMode opMode) {
        super.initSubsystem(opMode);
        slider= new SmartMotorEx(hardwareMap, "slider", SmartMotor.NeveRest.RPM_1780);
        slider.resetEncoder();
        slider.setRunMode(SmartMotor.RunMode.RawPower);
        controller=new PIDExController(veloCoefficients);
    }
}
