package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.customHardware.SmartMotorEx;

@Config
public class SliderV2Subsystem {

    SmartMotorEx Left_Slider, Right_Slider;

    public static double PosCoefficient = 0.05;
    public static double PosTolerance = 8;
    public static double sliderPower = 0.1;
    public static int HighPos = 3500;
    public static int MidPos = 2250;
    public static int LowPos = 1000;
    public static int GroundPos = 10;
    public LinearOpMode opMode;

    public static double power = 0.05; //Thw Power of the slides
    public static double DPP = 1; //Distance per Pulse

    GamepadEx gamepad;

    public SliderV2Subsystem(GamepadEx gamepad, SmartMotorEx Left_Slider, SmartMotorEx Right_Slider, LinearOpMode opMode){
        this.gamepad = gamepad;
        this.Left_Slider = Left_Slider;
        this.Right_Slider = Right_Slider;
        this.opMode = opMode;
    }

    void SliderManager(){

        Left_Slider.setRunMode(SmartMotorEx.RunMode.PositionControl);
        Left_Slider.setPositionCoefficient(PosCoefficient);
        Left_Slider.setZeroPowerBehavior(SmartMotorEx.ZeroPowerBehavior.BRAKE);
        Left_Slider.setPositionTolerance(PosTolerance);
        Left_Slider.resetEncoder();

        Right_Slider.setRunMode(SmartMotorEx.RunMode.RawPower);
        Right_Slider.setZeroPowerBehavior(SmartMotorEx.ZeroPowerBehavior.BRAKE);
        Right_Slider.resetEncoder();

        while(opMode.opModeIsActive()){
            if(gamepad.isDown(GamepadKeys.Button.LEFT_BUMPER)){
                setTargetPosition(Left_Slider.getCurrentPosition()+1);
            }
            else if(gamepad.isDown(GamepadKeys.Button.RIGHT_BUMPER)){
                setTargetPosition(Left_Slider.getCurrentPosition()-1);
            }

            if(gamepad.isDown(GamepadKeys.Button.DPAD_DOWN))
                setTargetPosition(LowPos);

            else if(gamepad.isDown(GamepadKeys.Button.DPAD_LEFT))
                setTargetPosition(MidPos);

            else if (gamepad.isDown(GamepadKeys.Button.DPAD_UP))
                setTargetPosition(HighPos);

            else if (gamepad.isDown(GamepadKeys.Button.DPAD_RIGHT))
                setTargetPosition(GroundPos);

            Left_Slider.set(power);
        }
    }

    void setTargetPosition(int pos){
        Left_Slider.setTargetPosition(Math.max(GroundPos, Math.min(HighPos, pos)));
    }
}