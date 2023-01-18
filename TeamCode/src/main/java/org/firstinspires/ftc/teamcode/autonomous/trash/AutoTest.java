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
//public class AutoTest extends LinearOpMode {
//
//    TrajectorySequence traj, traj1;
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
//        traj = drive.trajectorySequenceBuilder(new Pose2d())
//                .forward(10)
//                .addDisplacementMarker(() -> drive.followTrajectorySequenceAsync(traj1))
//                .build();
//
//        traj1 = drive.trajectorySequenceBuilder(traj.end())
//                        .lineToLinearHeading(new Pose2d(20, 0, Math.toRadians(10)))
//                        .build();
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
