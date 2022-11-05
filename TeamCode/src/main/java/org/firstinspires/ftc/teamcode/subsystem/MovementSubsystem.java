package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MovementSubsystem extends SmartSubsystem{

    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;

    @Override
    public void initSubsystem(LinearOpMode linearOpMode, HardwareMap hardwareMap) {
        super.initSubsystem(linearOpMode, hardwareMap);

        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        frontLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        backLeft.setDirection(DcMotorEx.Direction.REVERSE);
        backRight.setDirection(DcMotorEx.Direction.FORWARD);
        frontRight.setDirection(DcMotorEx.Direction.FORWARD);

    }
    public void run(GamepadEx gamepad){

        double Forward = gamepad.getLeftY();
        double Strafe = gamepad.getLeftX();
        double Turn = gamepad.getRightX();

        if(!gamepad.isDown(GamepadKeys.Button.LEFT_BUMPER)){
            Forward /= 2;
            Strafe  /= 2;
        }
        if(!gamepad.isDown(GamepadKeys.Button.RIGHT_BUMPER)){
            Turn /= 2;
        }

        double r = Math.hypot(Strafe, Forward);

        double robotAngle = Math.atan2(Forward, Strafe) - Math.PI / 4;

        final double v1 = (r * Math.cos(robotAngle)) + Turn;
        final double v2 = (r * Math.sin(robotAngle)) - Turn;
        final double v3 = (r * Math.sin(robotAngle)) + Turn;
        final double v4 = (r * Math.cos(robotAngle)) - Turn;
        opMode.telemetry.addData("frontLeft", frontLeft.getCurrentPosition());
        opMode.telemetry.addData("frontRight", frontRight.getCurrentPosition());
        opMode.telemetry.addData("backLeft", backLeft.getCurrentPosition());
        opMode.telemetry.addData("backRight", backRight.getCurrentPosition());
        frontLeft.setPower(v1);
        frontRight.setPower(v2);
        backLeft.setPower(v3);
        backRight.setPower(v4);
    }
}
