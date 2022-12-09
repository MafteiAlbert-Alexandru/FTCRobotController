package org.firstinspires.ftc.teamcode.experiments;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp
public class ArmPIDF extends LinearOpMode {

    DcMotorEx robotArm;

    private PIDController controller;

    public static double p = 0.006, i = 0.0001, d = 0.0005;

    public static double f = 0.1;

    public static float target = 0;

    float minPos, maxPos;

    final double ticks_in_degree = 537 / 180.0;

    @Override
    public void runOpMode() throws InterruptedException {

        robotArm = hardwareMap.get(DcMotorEx.class, "arm");

        robotArm.setDirection(DcMotorSimple.Direction.FORWARD);
        robotArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        controller = new PIDController(p, i, d);


        waitForStart();

        while(opModeIsActive())
        {

            target = clamp(target, minPos, maxPos);

            PIDF_Arm();
        }
    }

    public void PIDF_Arm()
    {
        controller.setPID(p, i, d);
        int armPos = robotArm.getCurrentPosition();
        double pid = controller.calculate(armPos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

        double power = pid + ff;

        robotArm.setPower(power);

        telemetry.addData("pos", armPos);
        telemetry.addData("target", target);
        telemetry.update();
    }

    public static float clamp(float val, float min, float max) {
        return Math.max(min, Math.min(max, val));
    }
}