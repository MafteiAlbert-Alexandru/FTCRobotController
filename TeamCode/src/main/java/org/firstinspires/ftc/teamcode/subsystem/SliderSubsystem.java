package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.hardware.customHardware.SmartMotorEx;

@Config
public class SliderSubsystem {

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

    Thread SliderSystem;

    GamepadEx gamepad;

    public SliderSubsystem(GamepadEx gamepad, SmartMotorEx Left_Slider, SmartMotorEx Right_Slider, LinearOpMode opMode){
        this.gamepad = gamepad;
        this.Left_Slider = Left_Slider;
        this.Right_Slider = Right_Slider;
        this.opMode = opMode;
    }

    public void SetupSliders()  {

        SliderSystem = new Thread(this::SliderManager);

        SliderSystem.start();

        //if(opMode.isStopRequested()) StopSliders();
    }

    public void StopSliders(){
        SliderSystem.interrupt();
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
                setTargetPosition(HighPos);
                if(Left_Slider.getCurrentPosition()<HighPos)
                    setPower(1);
                else stopMotor();
            }
            else if(gamepad.isDown(GamepadKeys.Button.RIGHT_BUMPER)){
                setTargetPosition(GroundPos);
                if(Left_Slider.getCurrentPosition()>GroundPos)
                    setPower(1);
                else stopMotor();
            }
            else stopMotor();

            if(gamepad.isDown(GamepadKeys.Button.DPAD_DOWN))
                LowPosSlider(LowPos);

            else if(gamepad.isDown(GamepadKeys.Button.DPAD_LEFT))
                MidPosSlider(MidPos);

            else if (gamepad.isDown(GamepadKeys.Button.DPAD_UP))
                HighPosSlider(HighPos);

            else if (gamepad.isDown(GamepadKeys.Button.DPAD_RIGHT))
                GroundPosSlider(GroundPos);
        }

        StopSliders();
    }

    void HighPosSlider (int pos){
        SetPosSlider(pos);

        String direction;

        if (Left_Slider.getCurrentPosition() > pos) direction = "down";
        else if (Left_Slider.getCurrentPosition() < pos) direction = "up";
        else direction = "equal";

        if (direction.equals("up")) {
            while (Left_Slider.getCurrentPosition() <= pos && opMode.opModeIsActive()) {
                if (Left_Slider.getCurrentPosition() <= pos) setPower(power);
                else stopMotor();

                if (gamepad.gamepad.touchpad || gamepad.gamepad.start || gamepad.isDown(GamepadKeys.Button.DPAD_DOWN) || gamepad.isDown(GamepadKeys.Button.DPAD_LEFT) || gamepad.isDown(GamepadKeys.Button.DPAD_RIGHT) || gamepad.isDown(GamepadKeys.Button.LEFT_BUMPER) || gamepad.isDown(GamepadKeys.Button.RIGHT_BUMPER)) break;

            }
        } else if (direction.equals("down")) {
            while (Left_Slider.getCurrentPosition() >= pos && opMode.opModeIsActive()) {
                if (Left_Slider.getCurrentPosition() >= pos) setPower(power);
                else stopMotor();

                if (gamepad.gamepad.touchpad || gamepad.gamepad.start || gamepad.isDown(GamepadKeys.Button.DPAD_DOWN) || gamepad.isDown(GamepadKeys.Button.DPAD_LEFT) || gamepad.isDown(GamepadKeys.Button.DPAD_RIGHT) || gamepad.isDown(GamepadKeys.Button.LEFT_BUMPER) || gamepad.isDown(GamepadKeys.Button.RIGHT_BUMPER)) break;
            }
        } else stopMotor();
    }

    void MidPosSlider (int pos){
        SetPosSlider(pos);

        String direction;

        if (Left_Slider.getCurrentPosition() > pos) direction = "down";
        else if (Left_Slider.getCurrentPosition() < pos) direction = "up";
        else direction = "equal";

        if (direction.equals("up")) {
            while (opMode.opModeIsActive() && Left_Slider.getCurrentPosition() <= pos) {
                if (Left_Slider.getCurrentPosition() <= pos) setPower(sliderPower);
                else stopMotor();

                if (gamepad.gamepad.touchpad || gamepad.gamepad.start || gamepad.isDown(GamepadKeys.Button.DPAD_DOWN) || gamepad.isDown(GamepadKeys.Button.DPAD_UP) || gamepad.isDown(GamepadKeys.Button.DPAD_RIGHT) || gamepad.isDown(GamepadKeys.Button.LEFT_BUMPER) || gamepad.isDown(GamepadKeys.Button.RIGHT_BUMPER)) break;

            }
        } else if (direction.equals("down")) {
            while (opMode.opModeIsActive() && Left_Slider.getCurrentPosition() >= pos) {
                if (Left_Slider.getCurrentPosition() >= pos) setPower(sliderPower);
                else stopMotor();

                if (gamepad.gamepad.touchpad || gamepad.gamepad.start || gamepad.isDown(GamepadKeys.Button.DPAD_DOWN) || gamepad.isDown(GamepadKeys.Button.DPAD_UP) || gamepad.isDown(GamepadKeys.Button.DPAD_RIGHT) || gamepad.isDown(GamepadKeys.Button.LEFT_BUMPER) || gamepad.isDown(GamepadKeys.Button.RIGHT_BUMPER)) break;

            }
        } else stopMotor();
    }

    void LowPosSlider (int pos){
        SetPosSlider(pos);

        String direction;

        if (Left_Slider.getCurrentPosition() > pos) direction = "down";
        else if (Left_Slider.getCurrentPosition() < pos) direction = "up";
        else direction = "equal";

        if (direction.equals("up")) {
            while (opMode.opModeIsActive() && Left_Slider.getCurrentPosition() <= pos) {
                if (Left_Slider.getCurrentPosition() <= pos) setPower(sliderPower);
                else stopMotor();

                if (gamepad.gamepad.touchpad || gamepad.gamepad.start || gamepad.isDown(GamepadKeys.Button.DPAD_LEFT) || gamepad.isDown(GamepadKeys.Button.DPAD_UP) || gamepad.isDown(GamepadKeys.Button.DPAD_RIGHT) || gamepad.isDown(GamepadKeys.Button.LEFT_BUMPER) || gamepad.isDown(GamepadKeys.Button.RIGHT_BUMPER)) break;


            }
        } else if (direction.equals("down")) {
            while (opMode.opModeIsActive() && Left_Slider.getCurrentPosition() >= pos) {
                if (Left_Slider.getCurrentPosition() >= pos) setPower(sliderPower);
                else stopMotor();

                if (gamepad.gamepad.touchpad || gamepad.gamepad.start || gamepad.isDown(GamepadKeys.Button.DPAD_LEFT) || gamepad.isDown(GamepadKeys.Button.DPAD_UP) || gamepad.isDown(GamepadKeys.Button.DPAD_RIGHT) || gamepad.isDown(GamepadKeys.Button.LEFT_BUMPER) || gamepad.isDown(GamepadKeys.Button.RIGHT_BUMPER)) break;


            }
        } else stopMotor();
    }

    void GroundPosSlider(int pos){
        SetPosSlider(pos);

        String direction;

        if (Left_Slider.getCurrentPosition() > pos) direction = "down";
        else if (Left_Slider.getCurrentPosition() < pos) direction = "up";
        else direction = "equal";

        if (direction.equals("up")) {
            while (opMode.opModeIsActive()) {
                if (Left_Slider.getCurrentPosition() >= pos) setPower(power);
                else stopMotor();

                if (gamepad.gamepad.touchpad || gamepad.gamepad.start || gamepad.isDown(GamepadKeys.Button.DPAD_DOWN) || gamepad.isDown(GamepadKeys.Button.DPAD_LEFT) || gamepad.isDown(GamepadKeys.Button.DPAD_UP) || gamepad.isDown(GamepadKeys.Button.LEFT_BUMPER) || gamepad.isDown(GamepadKeys.Button.RIGHT_BUMPER)) break;

            }
        } else if (direction.equals("down")) {
            while (opMode.opModeIsActive() && Left_Slider.getCurrentPosition() >= pos) {
                if (Left_Slider.getCurrentPosition() >= pos) setPower(power);
                else stopMotor();

                if (gamepad.gamepad.touchpad || gamepad.gamepad.start || gamepad.isDown(GamepadKeys.Button.DPAD_DOWN) || gamepad.isDown(GamepadKeys.Button.DPAD_LEFT) || gamepad.isDown(GamepadKeys.Button.DPAD_UP) || gamepad.isDown(GamepadKeys.Button.LEFT_BUMPER) || gamepad.isDown(GamepadKeys.Button.RIGHT_BUMPER)) break;
            }
        } else stopMotor();
    }

    void SetPosSlider(int pos) {
        Left_Slider.setTargetPosition(pos);
    }

    void setPower(double pow){
        Left_Slider.set(pow);
        Right_Slider.setPower(Left_Slider.motorEx.getPower());
    }

    void stopMotor(){
        Left_Slider.stopMotor();
        Right_Slider.stopMotor();
    }

    void setTargetPosition(int pos){
        Left_Slider.setTargetPosition(pos);
    }
}