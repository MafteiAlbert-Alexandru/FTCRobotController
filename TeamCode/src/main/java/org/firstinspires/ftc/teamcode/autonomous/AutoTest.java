package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

@Config
@Autonomous(name = "1_Auto")
public class AutoTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-36, -60, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(startPose)
                .strafeRight(24)
                .forward(47)
                //.lineToLinearHeading(new Pose2d(-12, -12, Math.toRadians(45)))

                //Pun pe stalp preload ul
                .lineToLinearHeading(new Pose2d(-24, -12, Math.toRadians(90)))

                //Start cycle
                //1)
                .lineToLinearHeading(new Pose2d(-55, -12, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(-24, -12, Math.toRadians(90)))
                //2)
                .lineToLinearHeading(new Pose2d(-55, -12, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(-24, -12, Math.toRadians(90)))
                //3)
                .lineToLinearHeading(new Pose2d(-55, -12, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(-24, -12, Math.toRadians(90)))
                //4)
                .lineToLinearHeading(new Pose2d(-55, -12, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(-24, -12, Math.toRadians(90)))
                //5)
                .lineToLinearHeading(new Pose2d(-55, -12, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(-24, -12, Math.toRadians(90)))
                //Park
                .strafeLeft(14)
                .back(24)
                .build();

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySequence(traj1);
    }
}
