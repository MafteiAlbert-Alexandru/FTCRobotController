package org.firstinspires.ftc.teamcode.subsystem;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.customHardware.SmartMotor;
import org.firstinspires.ftc.teamcode.hardware.customHardware.SmartMotorEx;

@Config
public class SliderSubsystemAuto extends SmartSubsystem {

    public SmartMotorEx slider;

    public  int target = 0;
    public static double tolerance = 30;
    public static double upwardCoefficient = 0.1;
    public static double downwardCoefficient =0.1;
    public static double pow = 0.1;

    public static int groundPos =200;
    public static int lowPos =400;
    public static int midPos = 570;
    public static int highPos = 1475;

    public static int safePos = 200;
    public static int loadPos= 45;
    public static int clearPos = 300;

    public static int aimPos = 380;
    public static int cone5Pos = 230;
    public static int cone4Pos = 125;
    public static int cone3Pos = 65;
    public static int cone2Pos = 25;
    public static int cone1Pos = 0;

    public static boolean telemetryOn = true;

    public boolean isClear()
    {
        return slider.getCurrentPosition() >= safePos;
    }
    public void goToClear()
    {
        target=clearPos;
        slider.setTargetPosition(target);
    }

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

    public void update() throws InterruptedException {

        if(target>slider.getCurrentPosition()) slider.setPositionCoefficient(upwardCoefficient);
        else slider.setPositionCoefficient(downwardCoefficient);
        slider.setPositionTolerance(tolerance);

        slider.set(pow);
        slider.setTargetPosition((int)target);
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

    public void setTarget(int pos){
        target = pos;
    }

    public int getPosition(){
        return slider.getCurrentPosition();
    }
}
