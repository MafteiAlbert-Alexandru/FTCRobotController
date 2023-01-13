//package org.firstinspires.ftc.teamcode.autonomous;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDriveCancelable;
//import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
//
//@Autonomous
//public class RightBlueAuto2 extends LinearOpMode {
//
//    TrajectorySequence traj;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        telemetry=new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//        try{
//            SampleMecanumDriveCancelable drive = new SampleMecanumDriveCancelable(hardwareMap);
//
//            Pose2d startPoseRight = new Pose2d(36, -60, Math.toRadians(90));
//
//            Pose2d rightBlueMid = new Pose2d(22, -13, Math.toRadians(0));
//
//            Pose2d rightBlueJunction = new Pose2d(16, -7, Math.toRadians(35));
//
//            Pose2d rightBlueStack = new Pose2d(55, -12, Math.toRadians(0));
//
//            drive.setPoseEstimate(startPoseRight);
//
//            traj = drive.trajectorySequenceBuilder(startPoseRight)
//                    .forward(48)
//                    .strafeLeft(12)
//                    .strafeRight(12)
//                    .turn(Math.toRadians(-90))
//                    .lineToLinearHeading(rightBlueStack)
//                    .lineToLinearHeading(rightBlueMid)
//                    .setReversed(true)
//                    .splineToLinearHeading(rightBlueJunction, Math.toRadians(-45))
//                    .build();
//
//
//            waitForStart();
//            drive.followTrajectorySequenceAsync(traj);
//            if (isStopRequested()) return;
//
//            while (opModeIsActive()){
//                drive.update();
//            }
//        }
//        catch (Exception e) {
//            telemetry.addLine(e.toString());
//            telemetry.update();
//        }
//    }
//}
