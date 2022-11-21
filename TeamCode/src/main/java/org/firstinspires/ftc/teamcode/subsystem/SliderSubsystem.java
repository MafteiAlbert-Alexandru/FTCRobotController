package org.firstinspires.ftc.teamcode.subsystem;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.customHardware.SmartMotor;
import org.firstinspires.ftc.teamcode.hardware.customHardware.SmartMotorEx;



@Config
public class SliderSubsystem extends SmartSubsystem {

    public SmartMotorEx slider;


    public  int target = 0;
    public static double tolerance = 30;
    public static double upwardCoefficient = 0.04;
    public static double downwardCoefficient =0.04;
    public static double pow = 0.1;

    public static int groundPos =200;
    public static int lowPos =400;
    public static int midPos = 570;
    public static int highPos = 1550;

    public static int safePos = 200;
    public static int loadPos= 50;
    public static int clearPos = 300;


    public boolean isClear()
    {
        return slider.getCurrentPosition() >= safePos;
    }
    public void goToClear()
    {
        target=clearPos;
        slider.setTargetPosition(target);
    }

    public static boolean telemetryOn = false;
    public void goToTake()
    {
        target=loadPos;
        slider.setTargetPosition(target);
    }
    public void goToPosition(int position)
    {
        target=position;
        slider.setTargetPosition(position);
    }
    public Long lastTime=null;
    public static float clamp(float val, float min, float max) {
        return Math.max(min, Math.min(max, val));
    }
    public static float step = 10;
    public void move(float offset)
    {
        Long time = System.currentTimeMillis();
        if(lastTime==null) lastTime=time;


        target= (int) clamp(target+step*offset*(time-lastTime)/1000.0f, 0, highPos);
        slider.setTargetPosition(target);
    }
    @Override
    public void run(SubsystemData data) throws InterruptedException {
        if(data.operatorGamepad.wasJustPressed(GamepadKeys.Button.DPAD_UP)) target = highPos;
        else if(data.operatorGamepad.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) target = midPos;
        else if(data.operatorGamepad.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) target = groundPos;
        else if(data.operatorGamepad.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) target = lowPos;

        if(target>slider.getCurrentPosition()) slider.setPositionCoefficient(upwardCoefficient);
        else slider.setPositionCoefficient(downwardCoefficient);
        slider.setPositionTolerance(tolerance);

        slider.set(pow);
        slider.setTargetPosition((int)target);
        slider.setTargetPosition(target);

        if(telemetryOn){
            opMode.telemetry.addData("pos", slider.getCurrentPosition());
            opMode.telemetry.addData("target", target);
            opMode.telemetry.addData("pow", slider.get());
        }
    }

    @Override
    public void initSubsystem(LinearOpMode opMode, HardwareMap hardwareMap) {
        super.initSubsystem(opMode, hardwareMap);
        slider=new SmartMotorEx(hardwareMap, "slider", SmartMotor.NeveRest.RPM_1780, SmartMotor.MotorDirection.REVERSE);
        slider.resetEncoder();
        slider.setRunMode(SmartMotor.RunMode.PositionControl);
        slider.setZeroPowerBehavior(SmartMotor.ZeroPowerBehavior.FLOAT);
        slider.setInverted(true);
        target=0;
    }
}
