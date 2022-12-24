package org.firstinspires.ftc.teamcode.subsystem;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedforward.BasicFeedforward;
import com.ThermalEquilibrium.homeostasis.Filters.Estimators.RawValue;
import com.ThermalEquilibrium.homeostasis.Parameters.FeedforwardCoefficients;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficientsEx;
import com.ThermalEquilibrium.homeostasis.Systems.PositionVelocitySystem;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.experiments.PIDExFixed;
import org.firstinspires.ftc.teamcode.hardware.SmartMotor;
import org.firstinspires.ftc.teamcode.hardware.SmartMotorEx;

import java.util.function.DoubleSupplier;
@Config
public class HomeostatisSliderSubsystem extends  SmartSubsystem {


    public SmartMotorEx slider;

    public static double target=0;
    public static double speed=0;
    public static double power=0;
    public static PIDCoefficientsEx posCoefficients = new PIDCoefficientsEx(0.1,0.0,0.0,0,0,0);
    public static PIDCoefficientsEx veloCoefficients = new PIDCoefficientsEx(0.1,0,0, 100, 20, 0);
    public static FeedforwardCoefficients coefficientsFF = new FeedforwardCoefficients(1,1,1);

    private PIDExFixed posControl = new PIDExFixed(posCoefficients);
    private PIDExFixed veloControl = new PIDExFixed(veloCoefficients);
    private PositionVelocitySystem system;
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

        DoubleSupplier motorPosition = () -> slider.getCurrentPosition();
        DoubleSupplier motorVelocity = () -> slider.getVelocity();

        RawValue positionFilter = new RawValue(motorPosition);
        RawValue velocityFilter = new RawValue(motorVelocity);
        BasicFeedforward feedforward = new BasicFeedforward(coefficientsFF);
        system =new PositionVelocitySystem(positionFilter,
                        velocityFilter,feedforward,posControl,veloControl);
        target=0;
    }
}
