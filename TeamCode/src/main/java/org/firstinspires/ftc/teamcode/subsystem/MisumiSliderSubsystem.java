package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
@Config
public class MisumiSliderSubsystem extends SmartSubsystem{
    private MotorEx sliderMotor;
    private Servo baseServo;
    private Servo armPullServo;
    public static double position=0.5;
    public static double clampOffset=0.0;
    public void run(ToggleButtonReader clampButton) {
        clampButton.readValue();
        baseServo.setPosition(position);

        if (clampButton.getState())
        {
            armPullServo.setPosition(baseServo.getPosition()+clampOffset);
        }else armPullServo.setPosition(baseServo.getPosition());
    }
    @Override
    public void initSubsystem(LinearOpMode linearOpMode, HardwareMap hardwareMap) {
        super.initSubsystem(linearOpMode, hardwareMap);
        baseServo=hardwareMap.get(Servo.class, "swingBase");
        armPullServo=hardwareMap.get(Servo.class, "swingPullArm");
        armPullServo.setDirection(Servo.Direction.REVERSE);
    }
}
