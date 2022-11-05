package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;


@Config
public class IntakeSubsystem {
    public DcMotor intakeMotorLeft, intakeMotorRight;

    public static double power = 0.65;

    public IntakeSubsystem(DcMotor intakeMotorLeft, DcMotor intakeMotorRight){
        this.intakeMotorLeft = intakeMotorLeft;
        this.intakeMotorRight  = intakeMotorRight;
    }

    public IntakeSubsystem(HardwareMap hardwareMap, String leftIntake, String rightIntake){
        intakeMotorLeft = hardwareMap.get(DcMotor.class, leftIntake);
        intakeMotorRight = hardwareMap.get(DcMotor.class, rightIntake);
    }

    public void startIntake() {
        intakeMotorLeft.setPower(power);
        intakeMotorRight.setPower(power);
    }


    public void stopIntake() {
        intakeMotorLeft.setPower(0);
        intakeMotorRight.setPower(0);
    }

    public void reverseIntake() {
        intakeMotorLeft.setPower(-power);
        intakeMotorRight.setPower(-power);
    }

    public void runIntake(boolean isActive){
        if(isActive) startIntake();
        else stopIntake();
    }

}