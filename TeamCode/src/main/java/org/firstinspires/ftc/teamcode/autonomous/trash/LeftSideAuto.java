//package org.firstinspires.ftc.teamcode.autonomous;
//
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDriveCancelable;
//import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
//
//@Autonomous
//public class LeftSideAuto extends LinearOpMode {
//
//    Pose2d startPoseLeft = new Pose2d(-36, -60, Math.toRadians(90));
//
//    Pose2d leftStack = new Pose2d(-55, -12, Math.toRadians(180));
//
//    Pose2d leftJunction = new Pose2d(-15, -12, Math.toRadians(180));
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        SampleMecanumDriveCancelable drive = new SampleMecanumDriveCancelable(hardwareMap);
//
//        drive.setPoseEstimate(startPoseLeft);
//
//        TrajectorySequence traj = drive.trajectorySequenceBuilder(startPoseLeft)
//                .forward(52)//merg in fata cat sa indepartez signal ul
//                .back(4)//ma intorc in centru tile ului
//                .strafeRight(12)
//                .strafeLeft(12)
//
//                .turn(Math.toRadians(90))
//
//                .lineToLinearHeading(leftStack)
//                .lineToLinearHeading(leftJunction)
//                .turn(Math.toRadians(-55))
//                .turn(Math.toRadians(55))
//
//                .lineToLinearHeading(leftStack)
//                .lineToLinearHeading(leftJunction)
//                .turn(Math.toRadians(-55))
//                .turn(Math.toRadians(55))
//
//                .lineToLinearHeading(leftStack)
//                .lineToLinearHeading(leftJunction)
//                .turn(Math.toRadians(-55))
//                .turn(Math.toRadians(55))
//
//                .build();
//
//        waitForStart();
//
//        if (isStopRequested()) return;
//
//        drive.followTrajectorySequence(traj);
//    }
//
//
//
//
//}