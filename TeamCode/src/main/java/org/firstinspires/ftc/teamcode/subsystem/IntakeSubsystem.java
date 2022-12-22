package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.hardware.SmartMotor;
import org.firstinspires.ftc.teamcode.hardware.SmartMotorEx;


@Config
public class IntakeSubsystem extends SmartSubsystem {
    public SmartMotorEx leftIntake;
    public SmartMotorEx rightIntake;


    public static double power = 1;

    public void intake()
    {
        leftIntake.set(power);
        rightIntake.set(power);
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
        if(data.operatorGamepad.getButton(GamepadKeys.Button.LEFT_BUMPER))
        {
            leftIntake.set(power);
            rightIntake.set(power);
        }else if(data.operatorGamepad.getButton(GamepadKeys.Button.RIGHT_BUMPER)){
            leftIntake.set(-power);
            rightIntake.set(-power);
        }else {
            leftIntake.set(0);
            rightIntake.set(0);
        }

    }

    @Override
    public void initSubsystem(OpMode opMode) {
        super.initSubsystem(opMode);
        leftIntake=new SmartMotorEx(hardwareMap, "leftIntake", SmartMotorEx.GoBILDA.RPM_1620, SmartMotor.MotorDirection.REVERSE);
        rightIntake=new SmartMotorEx(hardwareMap, "rightIntake", SmartMotorEx.GoBILDA.RPM_1620, SmartMotor.MotorDirection.FORWARD);
        leftIntake.setInverted(true);
    }

}