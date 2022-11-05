package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.hardware.customHardware.SmartMotorEx;

public class SliderManager{

    SmartMotorEx slider;
    private static LinearOpMode opModeObj;
    private boolean _opModeIsActive;

    public SliderManager(LinearOpMode opMode, SmartMotorEx hm_slider, Gamepad gamepad){
        opModeObj = opMode;
        slider = hm_slider;
        gamepad1 = gamepad;
        SliderSetter();
        SliderController();
    }

    public void OpIsActive(boolean bool){
        _opModeIsActive = bool;
    }

    public static double PosCoefficient = 0.05;
    public static double PosTolerance = 8;
    public static double sliderPower = 0.1;
    public static int HighPos = 3500;
    public static int MidPos = 2500;
    public static int LowPos = 1520;
    public static int GroundPos = 200;
    public int sliderPos1;

    void SliderSetter(){
        slider.setRunMode(SmartMotorEx.RunMode.PositionControl);
        //slider.setPositionCoefficient(PosCoefficient);
        slider.setZeroPowerBehavior(SmartMotorEx.ZeroPowerBehavior.BRAKE);
        //slider.setPositionTolerance(PosTolerance);
        slider.resetEncoder();
    }

    void SliderController (){
            while (opModeObj.opModeIsActive()) {
                if (gamepad1.left_bumper) {
                    slider.setTargetPosition(HighPos);
                    if (sliderPos1 < HighPos) {
//                    SetPosSlider(HighPos, slider);
//                    slider.set(sliderPower);
                        slider.set(1);
                    } else slider.stopMotor();
                }
                else if (gamepad1.right_bumper) {
                    slider.setTargetPosition(GroundPos);
                    if (sliderPos1 > GroundPos) {
//                    SetPosSlider(HighPos, slider);
//                    slider.set(sliderPower);
                        slider.set(1);
                    } else
                        slider.stopMotor();
                }

                else slider.stopMotor();

                if (gamepad1.dpad_down) {
                    LowPosSlider(LowPos, slider);
                }
                else if (gamepad1.dpad_left) {
                    MidPosSlider(MidPos, slider);
                }
                else if (gamepad1.dpad_up) {
                    HighPosSlider(HighPos, slider);
                }
                else if (gamepad1.dpad_right) {
                    GroundPosSlider(GroundPos, slider);
                }
            }
        }

    void HighPosSlider ( int pos, SmartMotorEx slider){

            SetPosSlider(pos, slider);

            String direction;

            if (sliderPos1 > pos) direction = "down";
            else if (sliderPos1 < pos) direction = "up";
            else direction = "equal";

            if (direction.equals("up")) {
                while (sliderPos1 <= pos && opModeObj.opModeIsActive()) {
                    if (sliderPos1 >= 1000) slider.set(sliderPower);
                    else SetPowerSlider(0, slider);
                    if (gamepad1.touchpad) break;
                }
            } else if (direction.equals("down")) {
                while (sliderPos1 >= pos && opModeObj.opModeIsActive()) {
                    if (sliderPos1 >= 1000) slider.set(sliderPower);
                    else SetPowerSlider(0, slider);
                    if (gamepad1.touchpad) break;
                }
            } else SetPowerSlider(0, slider);
        }

    void MidPosSlider ( int pos, SmartMotorEx slider){
            SetPosSlider(pos, slider);

            String direction;

            if (sliderPos1 > pos) direction = "down";
            else if (sliderPos1 < pos) direction = "up";
            else direction = "equal";

            if (direction.equals("up")) {
                while (opModeObj.opModeIsActive() && sliderPos1 <= pos) {
                    if (sliderPos1 >= 1000) slider.set(sliderPower);
                    else SetPowerSlider(0.05, slider);
                    if (gamepad1.touchpad) break;
                }
            } else if (direction.equals("down")) {
                while (opModeObj.opModeIsActive() && sliderPos1 >= pos) {
                    if (sliderPos1 >= 1000) slider.set(sliderPower);
                    else SetPowerSlider(0.05, slider);
                    if (gamepad1.touchpad) break;
                }
            } else SetPowerSlider(0, slider);
        }

    void LowPosSlider ( int pos, SmartMotorEx slider){
            SetPosSlider(pos, slider);

            String direction;

            if (sliderPos1 > pos) direction = "down";
            else if (sliderPos1 < pos) direction = "up";
            else direction = "equal";

            if (direction.equals("up")) {
                while (opModeObj.opModeIsActive() && sliderPos1 <= pos) {
                    if (sliderPos1 >= 1000) slider.set(sliderPower);
                    else SetPowerSlider(0.05, slider);
                    if (gamepad1.touchpad) break;
                }
            } else if (direction.equals("down")) {
                while (opModeObj.opModeIsActive() && sliderPos1 >= pos) {
                    if (sliderPos1 >= 1000) slider.set(sliderPower);
                    else SetPowerSlider(0.05, slider);
                    if (gamepad1.touchpad) break;
                }
            } else SetPowerSlider(0, slider);
        }

    void GroundPosSlider(int pos, SmartMotorEx slider){
            SetPosSlider(pos, slider);

            String direction;

            if (sliderPos1 > pos) direction = "down";
            else if (sliderPos1 < pos) direction = "up";
            else direction = "equal";

            if (direction.equals("up")) {
                while (opModeObj.opModeIsActive()) {
                    if (sliderPos1 >= 1000) slider.set(sliderPower);
                    else SetPowerSlider(0.05, slider);
                    if (gamepad1.touchpad) break;
                }
            } else if (direction.equals("down")) {
                while (opModeObj.opModeIsActive() && sliderPos1 >= pos) {
                    if (sliderPos1 >= 1000) slider.set(sliderPower);
                    else SetPowerSlider(0.05, slider);
                    if (gamepad1.touchpad) break;
                }
            } else SetPowerSlider(0, slider);
        }

    void SetPosSlider ( int pos, @NonNull SmartMotorEx motor){
            motor.setTargetPosition(pos);
        }

    void SetPowerSlider ( double power, @NonNull SmartMotorEx motor){
            motor.set(power);
        }
    }
