//package org.firstinspires.ftc.teamcode.autonomous;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.teamcode.robot.Robot;
//import org.firstinspires.ftc.teamcode.robot.fsm.albert.State;
//import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDriveCancelable;
//import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
//
//@Autonomous
//public class OldAutoR extends LinearOpMode {
//
//    Pose2d startPoseRight = new Pose2d(36, -62, Math.toRadians(90));
//
//    Pose2d rightStack = new Pose2d(60, -11, Math.toRadians(0));
//
//    Pose2d rightStack2 = new Pose2d(61, -11, Math.toRadians(0));
//
//    Pose2d rightStack3 = new Pose2d(61, -11, Math.toRadians(0));
//
//    Pose2d rightJunction = new Pose2d(32, -12, Math.toRadians(0));
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        telemetry=new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//        try{
//            SampleMecanumDriveCancelable drive = new SampleMecanumDriveCancelable(hardwareMap);
//            Robot robot = new Robot(this, true);
//
//            drive.setPoseEstimate(startPoseRight);
//
//            TrajectorySequence traj1 = drive.trajectorySequenceBuilder(startPoseRight)
//                    .forward(53)
//                    .addDisplacementMarker(() ->{
//                        robot.SliderAndClampingFSM.goTo(robot.waitingState);
//                        robot.SliderAndClampingFSM.goTo(robot.loadedState);
//                    })
//                    .forward(1)
//                    .waitSeconds(0.75)
//                    .back(4)
//                    .addDisplacementMarker(() ->{
//                        robot.SliderAndClampingFSM.enqueStates(new State[]{robot.upperState});
//                    })
//                    .strafeLeft(13)
//                    .addDisplacementMarker(() ->{
//                        robot.clampSubsystem.release();
//                    })
//                    .waitSeconds(0.75)
//                    .addDisplacementMarker(()->{
//                        robot.SliderAndClampingFSM.goTo(robot.frontWaitingState);
//                    })
//                    .strafeLeft(0.5)
//                    .back(1)
//                    .strafeRight(12.5)
//
////                    //cycle 1
////
////                    .turn(Math.toRadians(-90))
////                    .addDisplacementMarker(() -> {
////                        AimPossliderSubsystem();
////                    })
////                    .lineToLinearHeading(rightStack)
////
////                    .addDisplacementMarker(() -> {
////                        TargetPosition(SliderSubsystem.cone5Pos);
////                        robot.clampSubsystem.clamp();
////                    })
////                    .back(0.5)
////                    .addDisplacementMarker(() -> {
////                        HighPossliderSubsystem();
////                    })
////
////                    .lineToLinearHeading(rightJunction)
////                    .turn(Math.toRadians(90))
////                    .strafeLeft(8)
////                    .addDisplacementMarker(() ->{
////                        robot.clampSubsystem.release();
////                    })
////                    .waitSeconds(0.2)
////                    .strafeLeft(0.5)
////                    .strafeRight(12.5)
////
////                    .turn(Math.toRadians(-90))
////                    .addDisplacementMarker(() -> {
////                        AimPossliderSubsystem();
////                    })
////                    .lineToLinearHeading(rightStack2)
////
////                    .addDisplacementMarker(() -> {
////                        TargetPosition(SliderSubsystem.cone4Pos);
////                        robot.clampSubsystem.clamp();
////                    })
////                    .back(0.5)
////                    .addDisplacementMarker(() -> {
////                        HighPossliderSubsystem();
////                    })
////                    .lineToLinearHeading(rightJunction)
////                    .turn(Math.toRadians(90))
////                    .strafeLeft(7.5)
////                    .addDisplacementMarker(() ->{
////                        robot.clampSubsystem.release();
////                    })
////                    .waitSeconds(0.2)
////                    .strafeLeft(0.5)
////                    .strafeRight(12.5)
////
////                    //cycle 3
////
////                    .turn(Math.toRadians(-90))
////                    .addDisplacementMarker(() -> {
////                        AimPossliderSubsystem();
////                    })
////                    .lineToLinearHeading(rightStack3)
////
////                    .addDisplacementMarker(() -> {
////                        TargetPosition(SliderSubsystem.cone3Pos);
////                        robot.clampSubsystem.clamp();
////                    })
////                    .back(0.5)
////                    .addDisplacementMarker(() -> {
////                        HighPossliderSubsystem();
////                    })
////                    .lineToLinearHeading(rightJunction)
////                    .turn(Math.toRadians(90))
////                    .strafeLeft(6)
////                    .addDisplacementMarker(() ->{
////                        robot.clampSubsystem.release();
////                    })
////                    .waitSeconds(0.2)
////                    .strafeLeft(0.5)
////                    .strafeRight(12.5)
////                    .addDisplacementMarker( () -> {
////                        robot.clampSubsystem.goToBackward();
////                        sliderSubsystem.goToTake();
////                    })
//                    .build();
//
//            waitForStart();
//
//            drive.followTrajectorySequenceAsync(traj1);
//            if (isStopRequested()) return;
//
//            while (opModeIsActive()){
//                drive.update();
//                robot.update();
//            }
//        }catch (Exception e) {
//            telemetry.addLine(e.toString());
//            telemetry.update();
//        }
//
//
//    }
//
//
//}