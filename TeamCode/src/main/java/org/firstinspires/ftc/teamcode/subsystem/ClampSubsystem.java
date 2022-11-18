package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.customHardware.ToggleButton;

@Config
public class ClampSubsystem extends SmartSubsystem{
    private Servo baseServo;
    private ToggleButton clampButton;
    private Servo armPullServo;
    public static double position=0;
    public static double clampOffset=0.28;
    public void run(ToggleButtonReader clampButton) {
        clampButton.readValue();
        baseServo.setPosition(position);

        if (clampButton.getState())
        {
            armPullServo.setPosition(baseServo.getPosition()+clampOffset);
        }else armPullServo.setPosition(baseServo.getPosition());
    }

    @Override
    public void run(SubsystemData data) throws InterruptedException {

    }

    @Override
    public void initSubsystem(LinearOpMode linearOpMode, HardwareMap hardwareMap) {
        super.initSubsystem(linearOpMode, hardwareMap);
        baseServo=hardwareMap.get(Servo.class, "swingBase");
        armPullServo=hardwareMap.get(Servo.class, "swingPullArm");
        armPullServo.setDirection(Servo.Direction.REVERSE);
        clampButton=new ToggleButton(GamepadKeys.Button.X);
    }
}
