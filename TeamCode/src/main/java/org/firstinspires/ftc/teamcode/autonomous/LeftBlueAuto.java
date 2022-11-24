package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.apriltag.ApriltagDetectionJNI;

@Config
@Autonomous
public class LeftBlueAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveCancelable drive = new SampleMecanumDriveCancelable(hardwareMap);

        Pose2d startPose = new Pose2d(-36, -60, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        TrajectorySequence traj = drive.trajectorySequenceBuilder(startPose)
                .forward(52)
                .back(4)
//                                .strafeRight(24)
//                                .forward(48)
                .lineToLinearHeading(new Pose2d(-36, -13, Math.toRadians(45)))

                //Pun pe stalp preload ul
                .lineToLinearHeading(new Pose2d(-35, -13, Math.toRadians(45)))

                //Start cycle
                //1)
                .lineToLinearHeading(new Pose2d(-55, -12, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(-36, -12, Math.toRadians(45)))
                //2)
                .lineToLinearHeading(new Pose2d(-55, -12, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(-36, -12, Math.toRadians(45)))
                //3)
                .lineToLinearHeading(new Pose2d(-55, -12, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(-36, -12, Math.toRadians(45)))
                //4)
                .lineToLinearHeading(new Pose2d(-55, -12, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(-36, -12, Math.toRadians(45)))
                //5)
                .lineToLinearHeading(new Pose2d(-55, -12, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(-36, -12, Math.toRadians(45)))

                .lineToLinearHeading(new Pose2d(-35, -12, Math.toRadians(90)))

                .build();

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySequence(traj);
    }
}
