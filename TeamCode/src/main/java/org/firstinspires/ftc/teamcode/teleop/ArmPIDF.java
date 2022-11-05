package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp
public class ArmPIDF extends LinearOpMode {

    DcMotorEx arm;

    private PIDController controller;

    public static int facePos = 0;
    public static int midPos =  530;
    public static int backPos = 0;

    public static double p = 0, i = 0, d = 0;

    public static double f = 0;

    public static double target = 0; //target motor 1

    final double ticks_in_degree = 3007 / 360.0;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        controller = new PIDController(p, i, d);

        arm = hardwareMap.get(DcMotorEx.class, "arm");

        arm.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        waitForStart();

        while(opModeIsActive())
        {
            PIDF_Arm();
        }
    }

    public void PIDF_Arm()
    {
        controller.setPID(p, i, d);
        int armPos = arm.getCurrentPosition();
        double pid = controller.calculate(armPos, target);
        double kf = kf_generator(f, armPos);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * kf;

        double power = pid + ff;

        arm.setPower(power);

        telemetry.addLine("pos_1 " + armPos);
        telemetry.addLine("target_1 " + target);
        telemetry.addLine("PID Target "+ pid);
        telemetry.addLine("power_1 " + power);
        telemetry.addLine("kf " + kf);
        telemetry.update();
    }

    public double kf_generator(double f1, int _pos){

        double pos = _pos - midPos;

        if(pos<backPos && pos>facePos) return pos / midPos * f1 * 2;
        else if(pos<midPos) return f1;
        else return -f1;

    }

    public void InputManager(){
        if(gamepad1.dpad_down)
            target=0;
        else if(gamepad1.dpad_right)
            target=170;
        else if(gamepad1.dpad_up)
            target=320;
        else if(gamepad1.dpad_left)
            target=660;

    }
}