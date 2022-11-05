package org.firstinspires.ftc.teamcode.teleop;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.customHardware.SmartMotorEx;

@Config
@TeleOp
public class SliderControl extends LinearOpMode {

    SmartMotorEx Left_Slider, Right_Slider;

    public static double PosCoefficient = 0.05;
    public static double PosTolerance = 8;
    public static double sliderPower = 0.1;
    public static int HighPos = 3500;
    public static int MidPos = 2250;
    public static int LowPos = 1000;
    public static int GroundPos = 10;
    public int leftPos, rightPos;

    public static double power = 0.05; //Thw Power of the slides
    public static double DPP = 1; //Distance per Pulse

    @Override
    public void runOpMode() throws InterruptedException {

        Left_Slider = new SmartMotorEx(hardwareMap, "Left_Slider", SmartMotorEx.GoBILDA.RPM_223);
        Right_Slider = new SmartMotorEx(hardwareMap, "Right_Slider", SmartMotorEx.GoBILDA.RPM_223);

        Thread left_sliderThread = new Thread( () -> SliderManager(Left_Slider, false));
        Thread right_sliderThread = new Thread( () -> SliderManager(Right_Slider, true));

        waitForStart();

        left_sliderThread.start();
        right_sliderThread.start();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        while (opModeIsActive()) {

            leftPos = Left_Slider.getCurrentPosition();
            rightPos = Right_Slider.getCurrentPosition();

            Left_Slider.setDistancePerPulse(DPP);
            Right_Slider.setDistancePerPulse(DPP);

            TelemetryManager();
        }

        left_sliderThread.interrupt();
        right_sliderThread.interrupt();
    }

    void TelemetryManager(){
        telemetry.addData("Position Left", Left_Slider.getCurrentPosition());
        telemetry.addData("Position Right", Right_Slider.getCurrentPosition());
        telemetry.update();
    }

    void SliderManager(SmartMotorEx slider, boolean isInverted){

        slider.setInverted(isInverted);

        slider.setRunMode(SmartMotorEx.RunMode.PositionControl);
        slider.setPositionCoefficient(PosCoefficient);
        slider.setZeroPowerBehavior(SmartMotorEx.ZeroPowerBehavior.BRAKE);
        slider.setPositionTolerance(PosTolerance);
        slider.resetEncoder();

        while(opModeIsActive()){
            if(gamepad1.left_bumper){
                slider.setTargetPosition(HighPos);
                if(slider.getCurrentPosition()<HighPos){
                    slider.set(1);
                }
                else slider.stopMotor();
            }
            else if(gamepad1.right_bumper){
                slider.setTargetPosition(GroundPos);
                if(slider.getCurrentPosition()>GroundPos){
                    slider.set(1);
                }
                else
                    slider.stopMotor();
            }
            else slider.stopMotor();

            if(gamepad1.dpad_down) {
                LowPosSlider(LowPos, slider);
            }
            else if(gamepad1.dpad_left) {
                MidPosSlider(MidPos, slider);
            }
            else if (gamepad1.dpad_up) {
                HighPosSlider(HighPos, slider);
            }
            else if (gamepad1.dpad_right){
                GroundPosSlider(GroundPos, slider);
            }
        }
    }

    void HighPosSlider ( int pos, SmartMotorEx slider){

        SetPosSlider(pos, slider);

        String direction;

        if (slider.getCurrentPosition() > pos) direction = "down";
        else if (slider.getCurrentPosition() < pos) direction = "up";
        else direction = "equal";

        if (direction.equals("up")) {
            while (slider.getCurrentPosition() <= pos && opModeIsActive()) {
                if (slider.getCurrentPosition() >= 1000) slider.set(sliderPower);
                else slider.stopMotor();
                if (gamepad1.touchpad) break;
            }
        } else if (direction.equals("down")) {
            while (slider.getCurrentPosition() >= pos && opModeIsActive()) {
                if (slider.getCurrentPosition() >= 1000) slider.set(sliderPower);
                else slider.stopMotor();
                if (gamepad1.touchpad) break;
            }
        } else slider.stopMotor();
    }

    void MidPosSlider ( int pos, SmartMotorEx slider){
        SetPosSlider(pos, slider);

        String direction;

        if (slider.getCurrentPosition() > pos) direction = "down";
        else if (slider.getCurrentPosition() < pos) direction = "up";
        else direction = "equal";

        if (direction.equals("up")) {
            while (opModeIsActive() && slider.getCurrentPosition() <= pos) {
                if (slider.getCurrentPosition() >= 1000) slider.set(sliderPower);
                else slider.set(power);
                if (gamepad1.touchpad) break;
            }
        } else if (direction.equals("down")) {
            while (opModeIsActive() && slider.getCurrentPosition() >= pos) {
                if (slider.getCurrentPosition() >= 1000) slider.set(sliderPower);
                else slider.set(power);
                if (gamepad1.touchpad) break;
            }
        } else slider.stopMotor();
    }

    void LowPosSlider ( int pos, SmartMotorEx slider){
        SetPosSlider(pos, slider);

        String direction;

        if (slider.getCurrentPosition() > pos) direction = "down";
        else if (slider.getCurrentPosition() < pos) direction = "up";
        else direction = "equal";

        if (direction.equals("up")) {
            while (opModeIsActive() && slider.getCurrentPosition() <= pos) {
                if (slider.getCurrentPosition() >= 1000) slider.set(sliderPower);
                else slider.set(power);
                if (gamepad1.touchpad) break;
            }
        } else if (direction.equals("down")) {
            while (opModeIsActive() && slider.getCurrentPosition() >= pos) {
                if (slider.getCurrentPosition() >= 1000) slider.set(sliderPower);
                else slider.set(power);
                if (gamepad1.touchpad) break;
            }
        } else slider.stopMotor();
    }

    void GroundPosSlider(int pos, SmartMotorEx slider){
        SetPosSlider(pos, slider);

        String direction;

        if (slider.getCurrentPosition() > pos) direction = "down";
        else if (slider.getCurrentPosition() < pos) direction = "up";
        else direction = "equal";

        if (direction.equals("up")) {
            while (opModeIsActive()) {
                if (slider.getCurrentPosition() >= 1000) slider.set(sliderPower);
                else slider.set(power);
                if (gamepad1.touchpad) break;
            }
        } else if (direction.equals("down")) {
            while (opModeIsActive() && slider.getCurrentPosition() >= pos) {
                if (slider.getCurrentPosition() >= 1000) slider.set(sliderPower);
                else slider.set(power);
                if (gamepad1.touchpad) break;
            }
        } else slider.stopMotor();
    }

    void SetPosSlider(int pos, @NonNull SmartMotorEx motor) {
        motor.setTargetPosition(pos);
    }
}
