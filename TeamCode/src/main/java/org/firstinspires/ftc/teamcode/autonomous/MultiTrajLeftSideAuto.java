package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

@Autonomous
public class MultiTrajLeftSideAuto extends LinearOpMode {

    Pose2d startPoseLeft = new Pose2d(-36, -60, Math.toRadians(90));

    Pose2d leftStack = new Pose2d(-55, -12, Math.toRadians(180));

    Pose2d leftJunction = new Pose2d(-15, -12, Math.toRadians(180));

    SampleMecanumDriveCancelable RoadRunner = new SampleMecanumDriveCancelable(hardwareMap);

    @Override
    public void runOpMode() throws InterruptedException {

        RoadRunner.setPoseEstimate(startPoseLeft);

        TrajectorySequence StartTraj = RoadRunner.trajectorySequenceBuilder(startPoseLeft)
                .forward(52)//merg in fata cat sa indepartez signal ul
                .back(4)//ma intorc in centru tile ului
                .strafeRight(12)
                .build();

        TrajectorySequence PlacePreLoad = RoadRunner.trajectorySequenceBuilder(StartTraj.end())
                .strafeLeft(12)
                .turn(Math.toRadians(90))
                .build();

        TrajectorySequence CycleCone = RoadRunner.trajectorySequenceBuilder(PlacePreLoad.end())
                .lineToLinearHeading(leftStack)
                .addTemporalMarker(this::PickFromStackCone)
                .lineToLinearHeading(leftJunction)
                .turn(Math.toRadians(-55))
                .addTemporalMarker(this::PlaceOnJunctionCone)
                .turn(Math.toRadians(55))
                .build();

        RoadRunner.followTrajectorySequenceAsync(StartTraj);
        RoadRunner.followTrajectorySequenceAsync(PlacePreLoad);
        RoadRunner.followTrajectorySequenceAsync(CycleCone);

        waitForStart();
        if (isStopRequested()) return;

        while(opModeIsActive()){
            RoadRunner.update();
        }

    }

    void PickFromStackCone(){}

    void PlaceOnJunctionCone(){}
}