package org.firstinspires.ftc.teamcode.subsystem;


import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDriveCancelable;

@Config

public class MovementSubsystem extends SmartSubsystem{
    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;
    public SampleMecanumDriveCancelable drive;
    public void move(double forward, double strafe, double turn)
    {
        forward=forward>1?1:(forward<-1)?-1:forward;
        strafe=strafe>1?1:(strafe<-1)?-1:strafe;
        turn=turn>1?1:(turn<-1)?-1:turn;
        double r = Math.hypot(strafe, forward);

        double robotAngle = Math.atan2(forward, strafe) - Math.PI / 4;

        final double v1 = (r * Math.cos(robotAngle)) + turn;
        final double v2 = (r * Math.sin(robotAngle)) - turn;
        final double v3 = (r * Math.sin(robotAngle)) + turn;
        final double v4 = (r * Math.cos(robotAngle)) - turn;
        opMode.telemetry.addData("frontLeft", frontLeft.getCurrentPosition());
        opMode.telemetry.addData("frontRight", frontRight.getCurrentPosition());
        opMode.telemetry.addData("backLeft", backLeft.getCurrentPosition());
        opMode.telemetry.addData("backRight", backRight.getCurrentPosition());

        frontLeft.setPower(v1);
        frontRight.setPower(v2);
        backLeft.setPower(v3);
        backRight.setPower(v4);
    }
    @Override
    public void run(SubsystemData data) {
        double Forward = data.driverGamepad.getLeftY();
        double Strafe = data.driverGamepad.getLeftX();
        double Turn = data.driverGamepad.getRightX();

        if(!data.driverGamepad.isDown(GamepadKeys.Button.LEFT_BUMPER)){
            Forward /= 1.5;
            Strafe  /= 1.5;
        }
        if(!data.driverGamepad.isDown(GamepadKeys.Button.RIGHT_BUMPER)){
            Turn /= 1.5;
        }
        double euler = Math.exp(1);
        move(Forward, Strafe,Turn);

    }

    @Override
    public void initSubsystem(OpMode opMode) {
        super.initSubsystem(opMode);

        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        backRight = hardwareMap.get(DcMotorEx.class, "backRightMotor");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRightMotor");

        drive = new SampleMecanumDriveCancelable(hardwareMap, frontLeft, backLeft,
                backRight, frontRight);

    }
}
