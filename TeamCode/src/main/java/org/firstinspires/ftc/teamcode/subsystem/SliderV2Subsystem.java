package org.firstinspires.ftc.teamcode.subsystem;


import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.hardware.SmartMotor;
import org.firstinspires.ftc.teamcode.hardware.SmartMotorEx;
import org.firstinspires.ftc.teamcode.util.SmartMath;

@Config
public class SliderV2Subsystem extends SmartSubsystem {

    public SmartMotorEx slider;
    PIDController controller;

    // TODO CALIBRATE POSITIONS FURTHER
    public  int target = 0; //Smash target
    public static double tolerance = 10; //Cata eroare tolereaza (noi suntem mai smec si o avem la 0) retard nu o avem la 10 ca nu merge asa
    //Asta mi face glisiera sa faca pau pau (adica e kP-ul din PID)

    //Aici niste pozitii ca suntem noi cam smec

    public static int GroundPos =150; //Ajutam putin PID-ul
    public static int LowPos =725;
    public static int MediumPos = 1125;
    public static int HighPos = 1525;
    public static int PreLoadPos = 250;

    public static int SafePos = 725;
    public static int LoadPos = 45;

    public static int AimPos = 500;
    public static int cone5Pos = 500;
    public static int cone4Pos = 400;
    public static int cone3Pos = 300;
    public static int cone2Pos = 200;
    public static int cone1Pos = 100;

    public static double p = 0.01, i = 0.01, d = 0.0002;

    public static double up_f = 0.05;

    public static double down_f = 0.035;

    int minPos = 0, maxPos = 1800;

    public static boolean telemetryOn = false; //Daca vreau telemetrie

    @Override
    public void run(SubsystemData data) throws InterruptedException {


        controller.setPID(p, i, d);
        int armPos = slider.getCurrentPosition();
        double pid = controller.calculate(armPos, target);
        double power = pid + kf();
        target = SmartMath.Clamp(target, minPos, maxPos);

        slider.set(0);

        if(telemetryOn){
            opMode.telemetry.addData("pos", slider.getCurrentPosition());
            opMode.telemetry.addData("target", target);
            opMode.telemetry.addData("power", power);
        }
    }

    @Override
    public void initSubsystem(OpMode opMode) {
        super.initSubsystem(opMode);
        slider = new SmartMotorEx(hardwareMap, "sliderMotor", SmartMotor.NeveRest.RPM_1780, SmartMotor.MotorDirection.REVERSE);
        slider.setRunMode(SmartMotor.RunMode.RawPower);
        slider.resetEncoder();
        target=0;

        controller = new PIDController(p, i, d);
    }

    public void goTo(int target) {
        goTo(target, Long.MAX_VALUE, tolerance);
    }

    public void goTo(int target, long timeout, double tolerance) {
        setTarget(target);
        long startTime=System.currentTimeMillis();
        while(!SmartMath.ToleranceError(slider.getCurrentPosition(), target, tolerance) && System.currentTimeMillis()-startTime<timeout);
    }

    public void setTarget(int target){
        this.target = target;
        slider.setTargetPosition(target);
    }

    public int getPosition(){
        return slider.getCurrentPosition();
    }

    public double kf(){
        if(target>slider.getCurrentPosition()) return up_f;
        return down_f;
    }

    //Functie secsi ca sa se verifice daca glisiera este intr-o pozitie sigura
    public boolean isSafe()
    {
        return slider.getCurrentPosition() >= SafePos;
    }
}