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
    public static double lowerArmPos = 0.65;
    public static double idleArmPos = 0.62;
    public static double closedLegPos = 0.27;
    public static double openedLegPos =0;
    private boolean lifting=false;
    public void lift() throws InterruptedException {
        lifting=true;

    }
    public boolean isUp()
    {
        return lifting;
    }
    public void goDown() throws InterruptedException {
        lifting=false;
    }
    private boolean firstDown = false;
    public boolean override=true;
    @Override
    public void run(SubsystemData data) throws InterruptedException {
        double realLowerArmPos = idleArmPos;
        if(data.operatorGamepad.isDown(GamepadKeys.Button.LEFT_BUMPER))
        {
            realLowerArmPos =lowerArmPos;
        }else {

            realLowerArmPos =idleArmPos;
        }
        if(data.operatorGamepad.isDown(GamepadKeys.Button.LEFT_BUMPER) || data.operatorGamepad.isDown(GamepadKeys.Button.B)) {
            leg.setPosition(closedLegPos);
        }
        else leg.setPosition(openedLegPos);

        if(override&&(data.operatorGamepad.isDown(GamepadKeys.Button.A)||lifting))
        {
            arm.setPosition(upperArmPos);
            firstDown=true;
        }
        else {
            arm.setPosition(realLowerArmPos);
        }
    }

    @Override
    public void initSubsystem(OpMode opMode) {
        super.initSubsystem(opMode);

        arm = hardwareMap.get(Servo.class, "arm");
        leg = hardwareMap.get(Servo.class, "leg");
    }
}