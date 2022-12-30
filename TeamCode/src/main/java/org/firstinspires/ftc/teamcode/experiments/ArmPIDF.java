package org.firstinspires.ftc.teamcode.experiments;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.SmartMotor;
import org.firstinspires.ftc.teamcode.hardware.SmartMotorEx;

@Config
@TeleOp
public class ArmPIDF extends LinearOpMode {

    SmartMotorEx slider;

    PIDController controller;

    public static double p = 0.01, i = 0.01, d = 0.0002;

    public static double up_f = 0.05;

    public static double down_f = 0.02;


    public static double target = 0;

    float minPos = 0, maxPos = 1800;

//    final double ticks_in_degree = 537 / 180.0;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        slider = new SmartMotorEx(hardwareMap, "slider", SmartMotor.NeveRest.RPM_1780, SmartMotor.MotorDirection.REVERSE);
        slider.setRunMode(SmartMotor.RunMode.RawPower);

        controller = new PIDController(p, i, d);

        waitForStart();

        while(opModeIsActive())
        {
            PIDF_Arm();
        }
    }

    public void PIDF_Arm()
    {
        controller.setPID(p, i, d);
        int armPos = slider.getCurrentPosition();
        double pid = controller.calculate(armPos, target);

        double power = pid + kf();

        slider.setPower(power);

        telemetry.addData("pos", armPos);
        telemetry.addData("target", target);
        telemetry.addData("power", power);
        telemetry.update();
    }

    public void clamp() {
        target = Math.max(minPos, Math.min(maxPos, target));
    }

    public double kf(){
        if(target>slider.getCurrentPosition()) return up_f;
        return down_f;
    }
}