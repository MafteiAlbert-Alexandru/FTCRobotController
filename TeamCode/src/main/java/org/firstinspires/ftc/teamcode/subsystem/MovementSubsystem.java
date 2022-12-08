package org.firstinspires.ftc.teamcode.subsystem;



import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceRunner;
import org.firstinspires.ftc.teamcode.roadrunner.util.LynxModuleUtil;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Config

public class MovementSubsystem extends SmartSubsystem{
    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;
    public SampleMecanumDriveCancelable drive;
    public void move(double forward, double strafe, double turn)
    {
//        forward=forward>1?1:(forward<0)?0:forward;
//        strafe=strafe>1?1:(strafe<0)?0:strafe;
//        turn=turn>1?1:(turn<0)?0:turn;
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
            Forward /= 2;
            Strafe  /= 2;
        }
        if(!data.driverGamepad.isDown(GamepadKeys.Button.RIGHT_BUMPER)){
            Turn /= 2;
        }

        move(Forward, Strafe,Turn);

    }

    @Override
    public void initSubsystem(LinearOpMode linearOpMode, HardwareMap hardwareMap) {
        super.initSubsystem(linearOpMode, hardwareMap);

        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");

        drive = new SampleMecanumDriveCancelable(hardwareMap, frontLeft, backLeft,
                backRight, frontRight);
    }
}
