package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class TransferSubsystem extends SmartSubsystem {

    private Servo arm;
    private Servo leg;
    public static double upperArmPos = 0.19;
    public static double lowerArmPos = 0.67;
    public static double idleArmPos = 0.62;
    public static double closedLegPos = 0.5;
    public static double openedLegPos =0;
    private boolean lifting=false;
    public void lift() throws InterruptedException {
        lifting=true;
        Thread.sleep(100);
    }
    public boolean isUp()
    {
        return lifting;
    }
    public void goDown() throws InterruptedException {
        lifting=false;
        Thread.sleep(100);
    }
    private boolean firstDown = false;


    private double realLowerArmPos=idleArmPos;
    @Override
    public void run(SubsystemData data) throws InterruptedException {
        if(data.operatorGamepad.isDown(GamepadKeys.Button.LEFT_BUMPER))
        {
            realLowerArmPos=lowerArmPos;
        }else realLowerArmPos=idleArmPos;
        if(data.operatorGamepad.isDown(GamepadKeys.Button.A)||lifting)
        {
            leg.setPosition(openedLegPos);
            if(!firstDown)
                Thread.sleep(300);
            arm.setPosition(upperArmPos);
            firstDown=true;
        }
        else {
            arm.setPosition(realLowerArmPos);
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
    public void initSubsystem(OpMode opMode) {
        super.initSubsystem(opMode);

        arm = hardwareMap.get(Servo.class, "arm");
        leg = hardwareMap.get(Servo.class, "leg");
    }
}