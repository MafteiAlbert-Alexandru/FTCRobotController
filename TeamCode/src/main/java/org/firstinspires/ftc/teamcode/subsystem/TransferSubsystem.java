package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class TransferSubsystem extends SmartSubsystem {

    private Servo arm;
    private Servo leg;
    public static double upperArmPos = 0.18 ;
    public static double lowerArmPos = 0.675;
    public static double idleArmPos = 0.62;
    public static double closedLegPos = 0.27;
    public static double hittingLegPos =0.45;
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
    public void goTo(double position)
    {
        arm.setPosition(position);
    }
    public void blockWithLeg()
    {
        //leg.setPosition(closedLegPos);
    }
    public void retreatLeg()
    {

        //leg.setPosition(openedLegPos);
    }

    public boolean bControl =true;
    @Override
    public void run(SubsystemData data) throws InterruptedException {
        if(bControl)
        {
//            if(data.operatorGamepad.isDown(GamepadKeys.Button.B))
//                leg.setPosition(hittingLegPos);
//            else
//                leg.setPosition(openedLegPos);
        }
    }

    @Override
    public void initSubsystem(OpMode opMode) {
        super.initSubsystem(opMode);

        arm = hardwareMap.get(Servo.class, "flipServo");
//        leg = hardwareMap.get(Servo.class, "legServo");
        arm.setPosition(idleArmPos);
    }
}