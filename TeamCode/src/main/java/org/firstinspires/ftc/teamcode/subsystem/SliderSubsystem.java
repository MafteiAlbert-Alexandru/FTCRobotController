package org.firstinspires.ftc.teamcode.subsystem;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.customHardware.SmartMotor;
import org.firstinspires.ftc.teamcode.hardware.customHardware.SmartMotorEx;

@Config
 public class SliderSubsystem extends SmartSubsystem {

    public SmartMotorEx slider;

    public static int target = 0;
    public static double tolerance = 30;
    public static double upwardCoefficient = 0.1;
    public static double downwardCoefficient =0.1;
    public static double pow = 0.1;

    public static int GroundPos =150;
    public static int lowPos =750;
    public static int midPos = 1150;
    public static int highPos = 1475;

    public static int safePos = 600;
    public static int loadPos= 45;
    public static int ClearPos = 300;

    public static int aimPos = 380;
    public static int cone5Pos = 230;
    public static int cone4Pos = 125;
    public static int cone3Pos = 65;
    public static int cone2Pos = 25;
    public static int cone1Pos = 0;

    public static boolean telemetryOn = true;
    public static PIDCoefficients sliderPID= new PIDCoefficients(0.02,0.0003,0.0008);
    public static Double kV=0.0;
    public static double sliderSpeed = 1500;

    private PIDFController sliderController;
    public boolean isClear()
    {
        return slider.getCurrentPosition() >= safePos;
    }
    public void goToClear()
    {
        target=ClearPos;
    }

    public void goToTake()
    {
        target=loadPos;
    }
    public void goToPosition(int position)
    {
        target=position;
    }

    public void goTo(int position)
    {
        target=position;
        while(Math.abs(slider.getCurrentPosition()-target)>=tolerance);
    }

    @Override
    public void run(SubsystemData data) throws InterruptedException {

        sliderController.setTargetVelocity(sliderSpeed);
        slider.set(sliderController.update(slider.getCurrentPosition(), slider.getCorrectedVelocity()));

        if(telemetryOn){
            opMode.telemetry.addData("pos", slider.getCurrentPosition());
            opMode.telemetry.addData("target", target);
            opMode.telemetry.addData("pow", slider.get());
            opMode.telemetry.addData("vel", slider.getCorrectedVelocity());
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
        sliderController= new PIDFController(sliderPID,kV);
        slider=new SmartMotorEx(hardwareMap, "slider", SmartMotor.NeveRest.RPM_1780, SmartMotor.MotorDirection.REVERSE);
        sliderController.setOutputBounds(-1,1);



            slider.resetEncoder();
        slider.setRunMode(SmartMotor.RunMode.RawPower);
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
