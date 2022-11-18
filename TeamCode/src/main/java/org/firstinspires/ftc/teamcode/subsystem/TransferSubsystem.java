package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.customHardware.ToggleButton;

@Config
public class TransferSubsystem extends SmartSubsystem {

    private Servo arm;
    public static double upperArmPos = 0.2;
    public static double lowerArmPos = 0.69;
    private Servo leg;
    public static double closedLegPos = 0.5;
    public static double centerLegPos = 0.25;
    public static double openedLegPos =0;

    public static double downLiftPos = 0.67;
    public static double upLiftPos = 0.2;
    public static double closeJoint1Pos = 0.48;
    public static double openJoint1Pos = 0.25;
    public static double closeJoint2Pos = 1;
    public static double openJoint2Pos = 0;
    public static double closeLegPos = 0;
    public static double openLegPos = 0.5;
    public boolean togAct = false;
    public static int timeout = 300;
    public boolean mainAct = false;



    public void runArm(boolean bool){
        if(mainAct) return;
        if(bool) arm.setPosition(upLiftPos);
        else arm.setPosition(downLiftPos);
    }

//    public void runJoint(boolean bool){
//        if(bool) {
//            joint1.setPosition(openJoint1Pos);
//            joint2.setPosition(openJoint2Pos);
//        }
//        else {
//            joint1.setPosition(closeJoint1Pos);
//            joint2.setPosition(closeJoint2Pos);
//        }
//    }

    public void runLegAuto(boolean bool){
        if(bool && togAct) {
            mainAct = true;
            togAct = false;
            leg.setPosition(openLegPos);
            opMode.sleep(timeout);
            leg.setPosition(closeLegPos);
            opMode.sleep(timeout);
            arm.setPosition(upLiftPos);
            opMode.sleep(timeout + 200);
            arm.setPosition(downLiftPos);
        }
        else if (!bool){
            mainAct = false;
            togAct = true;
        }
    }

    public void runLeg(boolean bool){
        if(bool) leg.setPosition(openLegPos);
        else leg.setPosition(closeLegPos);
    }
    private boolean firstDown = false;
    public void run(ToggleButton armButton, ButtonReader legButton)
    {
        armButton.update();


        if(armButton.getToggle())
        {
            leg.setPosition(openedLegPos);
            if(!firstDown)
                opMode.sleep(300);
            arm.setPosition(upperArmPos);
            firstDown=true;
        }else {
            arm.setPosition(lowerArmPos);
            if(firstDown) {
                opMode.sleep(300);
                firstDown=false;
            }
            if(legButton.isDown())
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

    }
}