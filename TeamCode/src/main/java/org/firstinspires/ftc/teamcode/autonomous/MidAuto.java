package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

@Autonomous
public class MidAuto extends LinearOpMode {

    Pose2d startPoseLeft = new Pose2d(-36, -60, Math.toRadians(90));
    Pose2d startPoseRight = new Pose2d(36, -60, Math.toRadians(90));

    Pose2d leftBlueJunction = new Pose2d(-24, -12, Math.toRadians(90));
    Pose2d leftBlueMidPos = new Pose2d(-34, -12, Math.toRadians(180));
    Pose2d leftBlueStack = new Pose2d(-55, -13, Math.toRadians(175));

    Pose2d startPose = new Pose2d(-36, -60, Math.toRadians(90));

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveCancelable drive = new SampleMecanumDriveCancelable(hardwareMap);

        drive.setPoseEstimate(startPoseLeft);

        TrajectorySequence traj = drive.trajectorySequenceBuilder(startPoseLeft)
                .forward(48)
                //Pun pe stalp preload ul
                .lineToLinearHeading(leftBlueJunction)
                .lineToLinearHeading(leftBlueMidPos)
                //Start cycle
                //1)
                .lineToLinearHeading(leftBlueStack)
                .lineToLinearHeading(leftBlueMidPos)

                .lineToLinearHeading(leftBlueJunction)
                .lineToLinearHeading(leftBlueMidPos)
                //2)
                .lineToLinearHeading(leftBlueStack)
                .lineToLinearHeading(leftBlueMidPos)
                .lineToLinearHeading(leftBlueJunction)
                .lineToLinearHeading(leftBlueMidPos)
                //3)
                .lineToLinearHeading(leftBlueStack)
                .lineToLinearHeading(leftBlueMidPos)
                .lineToLinearHeading(leftBlueJunction)
                .lineToLinearHeading(leftBlueMidPos)
                //4)
                .lineToLinearHeading(leftBlueStack)
                .lineToLinearHeading(leftBlueMidPos)
                .lineToLinearHeading(leftBlueJunction)
                .lineToLinearHeading(leftBlueMidPos)
                //5)
                .lineToLinearHeading(leftBlueStack)
                .lineToLinearHeading(leftBlueMidPos)
                .lineToLinearHeading(leftBlueJunction)
                .lineToLinearHeading(leftBlueMidPos)
                //Park
                .build();

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySequence(traj);
    }




}