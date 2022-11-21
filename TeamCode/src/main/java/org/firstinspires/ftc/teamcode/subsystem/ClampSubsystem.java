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
    public static double clampOffset=-0.02;
    public static double initialPosition=0.4;
    private boolean clamping = false;
    public void clamp()
    {
        clamping=true;

    }

    public void release()
    {
        clamping=false;
    }
    public void goToForward()
    {
        baseServo.setPosition(0);

    }
    public void goToBackward()
    {
        baseServo.setPosition(0.6);
    }
    @Override
    public void run(SubsystemData data) throws InterruptedException {
        if(data.operatorGamepad.wasJustPressed(GamepadKeys.Button.X)) clamping=!clamping;
        if(clamping) armPullServo.setPosition(baseServo.getPosition()-clampOffset);
        else armPullServo.setPosition(baseServo.getPosition()+0.28-clampOffset);
    }

    @Override
    public void initSubsystem(LinearOpMode linearOpMode, HardwareMap hardwareMap) {
        super.initSubsystem(linearOpMode, hardwareMap);
        baseServo=hardwareMap.get(Servo.class, "swingBase");
        armPullServo=hardwareMap.get(Servo.class, "swingPullArm");
        armPullServo.setDirection(Servo.Direction.REVERSE);
        baseServo.setPosition(initialPosition);
        clampButton=new ToggleButton(GamepadKeys.Button.X);
    }
}
