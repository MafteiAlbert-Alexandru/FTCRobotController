package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.customHardware.ToggleButton;

@Config
public class TransferSubsystem extends SmartSubsystem {

    private Servo arm;
    private ToggleButton armButton;
    private Servo leg;
    public static double upperArmPos = 0.2;
    public static double sliderSubsystemedArmPos =0.69;
    public static double lowerArmPos = 0.69;

    public static double closedLegPos = 0.5;
    public static double openedLegPos =0;


    private boolean firstDown = false;
    public void sliderSubsystem()
    {
        arm.setPosition(sliderSubsystemedArmPos);
    }
    public void lower()
    {

    }


    @Override
    public void run(SubsystemData data) throws InterruptedException {
        armButton.update(data.operatorGamepad);
        if(armButton.getToggle())
        {
            leg.setPosition(openedLegPos);
            if(!firstDown)
                Thread.sleep(300);
            arm.setPosition(upperArmPos);
            firstDown=true;
        }else {
            arm.setPosition(lowerArmPos);
            if(firstDown) {
                Thread.sleep(300);
                firstDown=false;
            }
            if(data.operatorGamepad.getButton(GamepadKeys.Button.B))
            {
                leg.setPosition(closedLegPos);
            }else leg.setPosition(openedLegPos);
        }
    }

    @Override
    public void initSubsystem(LinearOpMode opMode, HardwareMap hardwareMap) {
        super.initSubsystem(opMode, hardwareMap);

        arm=hardwareMap.get(Servo.class, "arm");
        leg=hardwareMap.get(Servo.class, "leg");
        armButton=new ToggleButton(GamepadKeys.Button.A);

    }
}