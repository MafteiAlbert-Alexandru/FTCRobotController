package org.firstinspires.ftc.teamcode.subsystem;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.hardware.SmartMotor;
import org.firstinspires.ftc.teamcode.hardware.SmartMotorEx;
import org.firstinspires.ftc.teamcode.util.SmartMath;

@Config
public class SliderSubsystem extends SmartSubsystem {

    public SmartMotorEx slider;

    // TODO CALIBRATE POSITIONS FURTHER
    public  int target = 0; //Smash target
    public static double speed=5;
    public static double tolerance = 20; //Cata eroare tolereaza (noi suntem mai smec si o avem la 0) retard nu o avem la 10 ca nu merge asa
    public static double upwardCoefficient = 0.15; //Cand ridic glisiera mi se opune gravitatia deci putere go brrrr
    public static double downwardCoefficient =0.06; //Dar cand cobor sunt mai chill
    public static double pow = 0.1; //POWer pentru cine nu si-a dat seama
    //Asta mi face glisiera sa faca pau pau (adica e kP-ul din PID)

    //Aici niste pozitii ca suntem noi cam smec

    public static int GroundPos =150; //Ajutam putin PID-ul
    public static int LowPos =680;
    public static int MediumPos = 1170;
    public static int HighPos = 1600;
    public static int PreLoadPos = 330;

    public static int SafePos = 600;
    public static int LoadPos = 40;

    public static int AimPos = 500;
    public static int StackLoad = 150;

    public static int cone5Pos = 220;
    public static int cone4Pos = 170;
    public static int cone3Pos = 120;
    public static int cone2Pos = 50;
    public static int cone1Pos = 5;

    public static boolean telemetryOn = false; //Daca vreau telemetrie

    //Functie secsi ca sa se verifice daca glisiera este intr-o pozitie sigura
    public boolean isSafe()
    {
        return slider.getCurrentPosition() >= LowPos;
    }

    public SliderSubsystem(){}

    public SliderSubsystem(LinearOpMode opMode){
        initSubsystem(opMode);
    }


    @Override
    public void run(SubsystemData data) throws InterruptedException {

            if(target>slider.getCurrentPosition()) slider.setPositionCoefficient(upwardCoefficient);
            else slider.setPositionCoefficient(downwardCoefficient);
            slider.setPositionTolerance(tolerance);
            slider.setTargetPosition(target);
            slider.set(pow);

            if(telemetryOn){
                opMode.telemetry.addData("pos", slider.getCurrentPosition());
                opMode.telemetry.addData("target", target);
                opMode.telemetry.addData("power", slider.get());
            }
    }

    public void update() throws InterruptedException {

        if(target>slider.getCurrentPosition()) slider.setPositionCoefficient(upwardCoefficient);
        else slider.setPositionCoefficient(downwardCoefficient);
        slider.setPositionTolerance(tolerance);
        slider.setTargetPosition(target);
        slider.set(pow);

        if(telemetryOn){
            opMode.telemetry.addData("pos", slider.getCurrentPosition());
            opMode.telemetry.addData("target", target);
            opMode.telemetry.addData("power", slider.get());
        }
    }

    @Override
    public void initSubsystem(OpMode opMode) {
        super.initSubsystem(opMode);
        slider=new SmartMotorEx(hardwareMap, "slider", SmartMotor.NeveRest.RPM_1780, SmartMotor.MotorDirection.REVERSE);
        slider.resetEncoder();
        slider.setRunMode(SmartMotor.RunMode.PositionControl);
        slider.setZeroPowerBehavior(SmartMotor.ZeroPowerBehavior.FLOAT);
        slider.setInverted(true);
        target=0;
    }
    public void goTo(int target) {
        goTo(target, Long.MAX_VALUE,tolerance);
    }
    public void goTo(int target, long timeout) {
        goTo(target, timeout,tolerance);
    }

    public void goTo(int target, long timeout, double tolerance) {
        setTarget(target);
        long startTime=System.currentTimeMillis();
        while(!(SmartMath.ToleranceError(getPosition(), getTarget(), tolerance))& System.currentTimeMillis()-startTime<timeout);
    }
    public void setTarget(int target){
        this.target = target;
        slider.setTargetPosition(target);
    }

    public double getTarget(){
        return target;
    }

    public int getPosition(){
        return slider.getCurrentPosition();
    }
}