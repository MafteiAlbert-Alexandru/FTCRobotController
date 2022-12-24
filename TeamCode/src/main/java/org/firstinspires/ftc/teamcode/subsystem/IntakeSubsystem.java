package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.hardware.SmartMotor;
import org.firstinspires.ftc.teamcode.hardware.SmartMotorEx;


@Config
public class IntakeSubsystem extends SmartSubsystem {
    public SmartMotorEx leftIntake;
    public SmartMotorEx rightIntake;


    public static double power = 1;


    public void intake(double pow)
    {
        leftIntake.set(pow);
        rightIntake.set(pow);
    }
    public void expulse()
    {
        leftIntake.set(-power);
        rightIntake.set(-power);
    }
    public void stop()
    {
        leftIntake.set(0);
        rightIntake.set(0);
    }

    @Override
    public void run(SubsystemData data) throws InterruptedException {
    }

    @Override
    public void initSubsystem(OpMode opMode) {
        super.initSubsystem(opMode);
        leftIntake=new SmartMotorEx(hardwareMap, "leftIntake", SmartMotorEx.GoBILDA.RPM_1620, SmartMotor.MotorDirection.REVERSE);
        rightIntake=new SmartMotorEx(hardwareMap, "rightIntake", SmartMotorEx.GoBILDA.RPM_1620, SmartMotor.MotorDirection.FORWARD);
        leftIntake.setInverted(true);
    }

}