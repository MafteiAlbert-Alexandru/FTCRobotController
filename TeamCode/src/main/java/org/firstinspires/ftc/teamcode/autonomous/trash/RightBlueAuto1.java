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
//public class RightBlueAuto1 extends LinearOpMode {
//
//    TrajectorySequence traj, traj1;
//
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        SampleMecanumDriveCancelable drive = new SampleMecanumDriveCancelable(hardwareMap);
//
//        Pose2d startPoseRight = new Pose2d(36, -60, Math.toRadians(90));
//
//        Pose2d rightBlueJunction = new Pose2d(0, -12, Math.toRadians(-90));
//
//        Pose2d rightBlueJunction2 = new Pose2d(4, -12, Math.toRadians(255));
//
//        Pose2d rightBlueStack = new Pose2d(60, -12, Math.toRadians(0));
//
//        drive.setPoseEstimate(startPoseRight);
//
//
//        traj = drive.trajectorySequenceBuilder(startPoseRight)
//                .addDisplacementMarker(() -> {})
//                .forward(48)
//                //Pun pe stalp preload ul
//                .lineToLinearHeading(rightBlueJunction)
//
//                //Start cycle
//                //1)
//                .lineToLinearHeading(rightBlueStack)
//
//                .lineToLinearHeading(rightBlueJunction)
//                //2)
//                .lineToLinearHeading(rightBlueStack)
//                .lineToLinearHeading(rightBlueJunction)
//                //3)
//                .lineToLinearHeading(rightBlueStack)
//                .lineToLinearHeading(rightBlueJunction)
//                //4)
//                .lineToLinearHeading(rightBlueStack)
//                .lineToLinearHeading(rightBlueJunction)
//                //5)
//                .lineToLinearHeading(rightBlueStack)
//                .lineToLinearHeading(rightBlueJunction)
//                .build();
//
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
