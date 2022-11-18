package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.hardware.customHardware.SmartMotor;
import org.firstinspires.ftc.teamcode.hardware.customHardware.SmartMotorEx;


@Config
public class IntakeSubsystem extends SmartSubsystem {
    public SmartMotorEx leftIntake;
    public SmartMotorEx rightIntake;

    public static double power = 0.65;
    public void run(ToggleButtonReader button)
    {
        button.readValue();
        if(button.getState())
        {
            leftIntake.set(power);
            rightIntake.set(power);
        }else
        {
            leftIntake.set(0);
            rightIntake.set(0);
        }
    }

    @Override
    public void initSubsystem(LinearOpMode opMode, HardwareMap hardwareMap) {
        super.initSubsystem(opMode, hardwareMap);
        leftIntake=new SmartMotorEx(hardwareMap, "leftIntake", SmartMotorEx.GoBILDA.RPM_1620, SmartMotor.MotorDirection.REVERSE);
        rightIntake=new SmartMotorEx(hardwareMap, "rightIntake", SmartMotorEx.GoBILDA.RPM_1620, SmartMotor.MotorDirection.FORWARD);
    }
}