//package org.firstinspires.ftc.teamcode.autonomous;
//
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.geometry.Vector2d;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDriveCancelable;
//import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
//
//@Autonomous
//public class AutoTrajTest extends LinearOpMode {
//
//    TrajectorySequence traj;
//
//
//
//    Pose2d startPoseLeft = new Pose2d(-36, -60, Math.toRadians(90));
////        Pose2d startPoseRight = new Pose2d(36, -60, Math.toRadians(90));
//
//    Vector2d leftBlueTransfer = new Vector2d(-8, -12);
//
////        Pose2d leftBlueCenter = new Pose2d(-36, -12, Math.toRadians(180));
//
//    Vector2d leftBlueStack = new Vector2d(-55, -12);
//
//    Pose2d leftBlueJunction = new Pose2d(-14, -8, Math.toRadians(135));
//
//    Pose2d leftBlueTranferBack = new Pose2d(-8, -12, Math.toRadians(180));
//
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        SampleMecanumDriveCancelable drive = new SampleMecanumDriveCancelable(hardwareMap);
//
////        Pose2d startPoseLeft = new Pose2d(-36, -60, Math.toRadians(90));
////        Pose2d leftBlueJunction = new Pose2d(-8, -12, Math.toRadians(-60));
////        Pose2d leftBlueStack = new Pose2d(-55, -12, Math.toRadians(180));\
//
////        drive.setPoseEstimate(startPoseLeft);
//
//
//        traj = drive.trajectorySequenceBuilder(startPoseLeft)
//                .forward(48)
//                .turn(Math.toRadians(90))
//                //Pun pe stalp preload ul
//                .lineToConstantHeading(leftBlueTransfer)
//                .lineToLinearHeading(leftBlueJunction)
//                .lineToLinearHeading(leftBlueTranferBack)
//
//                .lineToConstantHeading(leftBlueStack)
//
//                .lineToConstantHeading(leftBlueTransfer)
//                .lineToLinearHeading(leftBlueJunction)
//                .lineToLinearHeading(leftBlueTranferBack)
//                .lineToConstantHeading(leftBlueStack)
//
//                .lineToConstantHeading(leftBlueTransfer)
//                .lineToLinearHeading(leftBlueJunction)
//                .lineToLinearHeading(leftBlueTranferBack)
//                .lineToConstantHeading(leftBlueStack)
//
//                .lineToConstantHeading(leftBlueTransfer)
//                .lineToLinearHeading(leftBlueJunction)
//                .lineToLinearHeading(leftBlueTranferBack)
//                .lineToConstantHeading(leftBlueStack)
//
//                .lineToConstantHeading(leftBlueTransfer)
//                .lineToLinearHeading(leftBlueJunction)
//                .lineToLinearHeading(leftBlueTranferBack)
//                .lineToConstantHeading(leftBlueStack)
//
//                .lineToConstantHeading(leftBlueTransfer)
//                .lineToLinearHeading(leftBlueJunction)
//                .lineToLinearHeading(leftBlueTranferBack)
//                .lineToConstantHeading(leftBlueStack)
//                .build();
//
//        waitForStart();
//        drive.followTrajectorySequenceAsync(traj);
//        if (isStopRequested()) return;
//
//        while (opModeIsActive()){
//            drive.update();
//        }
//    }
//}
