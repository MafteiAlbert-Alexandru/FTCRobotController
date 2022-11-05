package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class TransferSubsystem {

    private final Servo arm;
    private final Servo joint1;
    private final Servo joint2;
    private final Servo leg;
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
    LinearOpMode opMode;

    public TransferSubsystem(Servo arm, Servo joint1, Servo joint2, Servo leg, LinearOpMode opMode){
        this.arm = arm;
        this.joint1 = joint1;
        this.joint2 = joint2;
        this.leg = leg;
        this.opMode = opMode;
    }

    public void runArm(boolean bool){
        if(mainAct) return;
        if(bool) arm.setPosition(upLiftPos);
        else arm.setPosition(downLiftPos);
    }

    public void runJoint(boolean bool){
        if(bool) {
            joint1.setPosition(openJoint1Pos);
            joint2.setPosition(openJoint2Pos);
        }
        else {
            joint1.setPosition(closeJoint1Pos);
            joint2.setPosition(closeJoint2Pos);
        }
    }

    public void runLegAuto(boolean bool){
        if(bool && togAct) {
            mainAct = true;
            togAct = false;
            leg.setPosition(openLegPos);
            opMode.sleep(timeout);
            leg.setPosition(closeLegPos);
            opMode.sleep(timeout);
            arm.setPosition(upLiftPos);
            opMode.sleep(timeout + 200
            );
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
}