package org.firstinspires.ftc.teamcode.subsystem;


import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.PIDEx;
import com.ThermalEquilibrium.homeostasis.Parameters.FeedforwardCoefficients;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficientsEx;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.hardware.SmartMotor;
import org.firstinspires.ftc.teamcode.hardware.SmartMotorEx;
@Config
public class HomeostatisSliderSubsystem extends  SmartSubsystem {


    public SmartMotorEx slider;
    public static double tolerance = 10;
    public static int GroundPos =150; //Ajutam putin PID-ul
    public static int LowPos =660;
    public static int MediumPos = 1050;
    public static int HighPos = 1475;
    public static int PreLoadPos = 250;

    public static int SafePos = 600;
    public static int LoadPos = 80;

    public static int aimPos = 380;
    public static int cone5Pos = 230;
    public static int cone4Pos = 125;
    public static int cone3Pos = 65;
    public static int cone2Pos = 25;
    public static int cone1Pos = 0;
    public static double target=0;
    public static double power=0.8;
    public static PIDCoefficientsEx posCoefficients = new PIDCoefficientsEx(0.02,0.0005,0,100,25,0.5   );
    public static FeedforwardCoefficients coefficientsFF = new FeedforwardCoefficients(1,1,1);
    public boolean isSafe()
    {
        return slider.getCurrentPosition() >= LowPos;
    }
    private PIDEx posControl = new PIDEx(posCoefficients);

    @Override
    public void run(SubsystemData data) throws InterruptedException {



        opMode.telemetry.addData("Target Position", target);
        opMode.telemetry.addData("Slider Position", slider.getCurrentPosition());
        double commandPower=posControl.calculate(target, slider.getCurrentPosition());

        opMode.telemetry.addData("Command Power", commandPower);
        double targetPower = Math.signum(commandPower)*Math.min(power, Math.abs(commandPower)*coefficientsFF.Kv);
        opMode.telemetry.addData("Target Power", commandPower);
        //
//        opMode.telemetry.addData("Target Speed", targetSpeed);
//        opMode.telemetry.addData("Speed", slider.getCorrectedVelocity());
//        double commandPower =veloControl.calculate(targetSpeed, slider.getCorrectedVelocity());
//
//        opMode.telemetry.addData("Command Power", commandPower);
//        double targetPower = Math.signum(commandPower)*Math.min(power, Math.abs(commandPower));
//
//        opMode.telemetry.addData("Target Power", targetPower);
        slider.set(targetPower);

        opMode.telemetry.update();
    }

    @Override
    public void initSubsystem(OpMode opMode) {
        super.initSubsystem(opMode);
        slider=new SmartMotorEx(hardwareMap, "slider", SmartMotor.NeveRest.RPM_1780, SmartMotor.MotorDirection.REVERSE);
        slider.resetEncoder();
        slider.setRunMode(SmartMotor.RunMode.RawPower);
        slider.setZeroPowerBehavior(SmartMotor.ZeroPowerBehavior.FLOAT);
        slider.setInverted(true);
        target=0;
    }
    public void goTo(int target) {
        this.target=target;
        while (Math.abs(slider.getCurrentPosition()-target)>tolerance);
    }
    public void setTarget(int target){
        this.target = target;
        slider.setTargetPosition(target);
    }

    public int getPosition(){
        return slider.getCurrentPosition();
    }
}
