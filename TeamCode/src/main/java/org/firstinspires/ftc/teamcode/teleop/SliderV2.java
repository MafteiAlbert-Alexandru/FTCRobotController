package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.customHardware.SmartMotor;
import org.firstinspires.ftc.teamcode.hardware.customHardware.SmartMotorEx;

@TeleOp
@Config
public class SliderV2 extends LinearOpMode {

    public ElapsedTime elapsedTime;
    public static double valPerTicks = 0;
    public static int groundPos = 0;
    public static int lowPos = 0;
    public static int midPos = 0;
    public static int highPos = 0;
    public static int minPos = 0;
    public static int maxPos = 0;

    SmartMotorEx leftSlider, rightSlider;

    public int desirePos = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        leftSlider = new SmartMotorEx(hardwareMap, "left_Slider", SmartMotor.GoBILDA.RPM_312, SmartMotorEx.MotorDirection.FORWARD);
        rightSlider = new SmartMotorEx(hardwareMap, "right_Slider", SmartMotor.GoBILDA.RPM_312, SmartMotorEx.MotorDirection.REVERSE);

        leftSlider.setRunMode(SmartMotor.RunMode.PositionControl);
        rightSlider.setRunMode(SmartMotor.RunMode.PositionControl);



        waitForStart();
        while (opModeIsActive()){


            InputManager();
            TelemetryManger();
        }
    }

    public void InputManager(){
        if(gamepad1.left_bumper) desirePos+=valPerTicks;
        else if(gamepad1.right_bumper) desirePos+=valPerTicks;
        else if(gamepad1.dpad_down) desirePos = lowPos;
        else if(gamepad1.dpad_left) desirePos = midPos;
        else if(gamepad1.dpad_up) desirePos = highPos;
        else if(gamepad1.dpad_right) desirePos = groundPos;
    }

    public void TelemetryManger(){
        telemetry.addLine("Left Slider Pos " + leftSlider.getCurrentPosition());
        telemetry.addLine("Right Slider Pos " + rightSlider.getCurrentPosition());
        telemetry.update();
    }
}
