//package org.firstinspires.ftc.teamcode.autonomous.opModes;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
//import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDriveCancelable;
//import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
//import org.firstinspires.ftc.teamcode.subsystem.ClampSubsystem;
//import org.firstinspires.ftc.teamcode.subsystem.SliderSubsystem;
//import org.firstinspires.ftc.teamcode.experiments.SliderV2Subsystem;
//
//@Autonomous
//public class RightAuto extends LinearOpMode {
//
//    TrajectorySequence trajectorySequence;
//
//    SliderSubsystem sliderSubsystem = new SliderSubsystem();
//    ClampSubsystem clampSubsystem = new ClampSubsystem();
//
//    public static int speed = 45;
//
//    public static double delayJunction = 0.7;
//
//    public static double delayStack = 1;
//
//    public static double delayLift = 0.8;
//
//    public static double delayLoad = 0;
//
//
//
//    @Override
//    public void runOpMode() throws InterruptedException {
////        Thread.currentThread().setPriority(10);
//        telemetry=new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//        try{
//            SampleMecanumDriveCancelable drive = new SampleMecanumDriveCancelable(hardwareMap);
//            sliderSubsystem.initSubsystem(this);
//            clampSubsystem.initSubsystem(this);
////
////            Thread updateSubsystem = new Thread(
////                    () -> {
////                        Thread.currentThread().setPriority(1);
////                        waitForStart();
////                        while (opModeIsActive() && !isStopRequested()) {
////                            try {
////                                sliderSubsystem.update();
////                                clampSubsystem.update();
////                            } catch (InterruptedException e) {
////                                telemetry.addLine(e.toString());
////                            }
////                            telemetry.update();
////                        }
////                    }
////            );
//
////            AutoFSM autoFSM = new AutoFSM(this, sliderSubsystem, clampSubsystem);
//
//            Pose2d startPoseLeft = new Pose2d(36, -60, Math.toRadians(90));
//            Pose2d startPoseRight = new Pose2d(36, -60, Math.toRadians(90));
//
//            Pose2d leftBlueMidPos = new Pose2d(36, -12, Math.toRadians(0));
//            Pose2d leftBlueStack = new Pose2d(60, -12.5, Math.toRadians(0));
//
//            Pose2d leftBlueStackCycle = new Pose2d(60, -12.5, Math.toRadians(0));
//
//
//            Pose2d rightBlueJunction = new Pose2d(15.5 -12.5, Math.toRadians(0));
//
//            drive.setPoseEstimate(startPoseRight);
//
//            trajectorySequence = drive.trajectorySequenceBuilder(startPoseRight)
//                    .addDisplacementMarker(() -> sliderSubsystem.setTarget(SliderV2Subsystem.LowPos))
//                    .addTemporalMarker(0.4, () -> clampSubsystem.setPosition(ClampSubsystem.BackwardPos))
//                    .addTemporalMarker(0.6, () -> sliderSubsystem.setTarget(SliderV2Subsystem.LoadPos))
//                    .addTemporalMarker(0.8, () -> clampSubsystem.clamp())
//                    .addTemporalMarker(1.1, () -> sliderSubsystem.setTarget(SliderSubsystem.HighPos))
//                    .addTemporalMarker(1.25, () ->clampSubsystem.setPosition(ClampSubsystem.ForwardPos))
//                    .forward(49)
//                    .strafeLeft(12)
//                    .UNSTABLE_addTemporalMarkerOffset(0.3, () -> clampSubsystem.release())
//                    .waitSeconds(1)
//                    .back(3)
//                    .addDisplacementMarker(() -> sliderSubsystem.setTarget(SliderV2Subsystem.LowPos))
////                    .back(1)
////                    .strafeLeft(2)
//                    .strafeRight(12)
//                    .addDisplacementMarker(() -> sliderSubsystem.setTarget(SliderV2Subsystem.AimPos))
//                    .turn(Math.toRadians(-90))
//                    //Pun pe stalp preload ul
//                    //Start cycle
//                    //1)
//                    .lineToLinearHeading(leftBlueStack, SampleMecanumDriveCancelable.getVelocityConstraint(speed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                            SampleMecanumDriveCancelable.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                    .UNSTABLE_addTemporalMarkerOffset(delayLoad, () -> {
//                        sliderSubsystem.setTarget(SliderSubsystem.cone5Pos);
//                        clampSubsystem.clamp();
//                    })
//                    .UNSTABLE_addTemporalMarkerOffset(delayLift, () -> sliderSubsystem.setTarget(SliderSubsystem.HighPos))
//                    .waitSeconds(delayStack)
//                    .back(1)
//                    .lineToSplineHeading(rightBlueJunction)
//                    .UNSTABLE_addTemporalMarkerOffset(delayJunction, () -> clampSubsystem.release())
//                    .strafeLeft(12)
//                    .waitSeconds(0.5)
//                    .strafeRight(12)
//                    //yas
//                    .addDisplacementMarker(() -> sliderSubsystem.setTarget(SliderV2Subsystem.AimPos))
//                    .lineToLinearHeading(leftBlueStackCycle, SampleMecanumDriveCancelable.getVelocityConstraint(speed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                            SampleMecanumDriveCancelable.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                    .UNSTABLE_addTemporalMarkerOffset(delayLoad, () -> {
//                        sliderSubsystem.setTarget(SliderSubsystem.cone4Pos);
//                        clampSubsystem.clamp();
//                    })
//                    .UNSTABLE_addTemporalMarkerOffset(delayLift, () -> sliderSubsystem.setTarget(SliderSubsystem.HighPos))
//                    .waitSeconds(delayStack)
//                    .back(1)
//                    .lineToSplineHeading(rightBlueJunction)
//                    .UNSTABLE_addTemporalMarkerOffset(delayJunction, () -> clampSubsystem.release())
//                    .strafeLeft(12)
//                    .waitSeconds(0.5)
//                    .strafeRight(12)
//                    .addDisplacementMarker(() -> sliderSubsystem.setTarget(SliderV2Subsystem.AimPos))
//                    .lineToLinearHeading(leftBlueStackCycle, SampleMecanumDriveCancelable.getVelocityConstraint(speed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                            SampleMecanumDriveCancelable.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                    .UNSTABLE_addTemporalMarkerOffset(delayLoad, () -> {
//                        sliderSubsystem.setTarget(SliderSubsystem.cone3Pos);
//                        clampSubsystem.clamp();
//                    })
//                    .UNSTABLE_addTemporalMarkerOffset(delayLift, () -> sliderSubsystem.setTarget(SliderSubsystem.HighPos))
//                    .waitSeconds(delayStack)
//                    .back(1)
//                    .lineToSplineHeading(rightBlueJunction)
//                    .UNSTABLE_addTemporalMarkerOffset(delayJunction, () -> clampSubsystem.release())
//                    .strafeLeft(12)
//                    .waitSeconds(0.5)
//                    .strafeRight(12)
//                    .addDisplacementMarker(() -> sliderSubsystem.setTarget(SliderV2Subsystem.AimPos))
//                    .lineToLinearHeading(leftBlueStackCycle, SampleMecanumDriveCancelable.getVelocityConstraint(speed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                            SampleMecanumDriveCancelable.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                    .UNSTABLE_addTemporalMarkerOffset(delayLoad, () -> {
//                        sliderSubsystem.setTarget(SliderSubsystem.cone2Pos);
//                        clampSubsystem.clamp();
//                    })
//                    .UNSTABLE_addTemporalMarkerOffset(delayLift, () -> sliderSubsystem.setTarget(SliderSubsystem.HighPos))
//                    .waitSeconds(delayStack)
//                    .back(1)
//                    .lineToSplineHeading(rightBlueJunction)
//                    .UNSTABLE_addTemporalMarkerOffset(delayJunction, () -> clampSubsystem.release())
//                    .strafeLeft(12)
//                    .waitSeconds(0.5)
//                    .strafeRight(12)
//                    .addDisplacementMarker(() -> sliderSubsystem.setTarget(SliderV2Subsystem.AimPos))
//                    .lineToLinearHeading(leftBlueStackCycle, SampleMecanumDriveCancelable.getVelocityConstraint(speed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                            SampleMecanumDriveCancelable.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                    .UNSTABLE_addTemporalMarkerOffset(delayLoad, () -> {
//                        sliderSubsystem.setTarget(SliderSubsystem.cone1Pos);
//                        clampSubsystem.clamp();
//                    })
//                    .UNSTABLE_addTemporalMarkerOffset(delayLift, () -> sliderSubsystem.setTarget(SliderSubsystem.HighPos))
//                    .waitSeconds(delayStack)
//                    .back(1)
//                    .lineToSplineHeading(rightBlueJunction)
//                    .UNSTABLE_addTemporalMarkerOffset(delayJunction, () -> clampSubsystem.release())
//                    .strafeLeft(12)
//                    .waitSeconds(0.5)
//                    .strafeRight(12)
//                    .build();
//
//
////            traj2 = drive.trajectorySequenceBuilder(traj1.end())
////                    .strafeRight(12)
////
////                    //2)
////                    .lineToLinearHeading(leftBlueStack)
//////                                .lineToLinearHeading(leftBlueMidPos)
////                    .lineToSplineHeading(rightBlueJunction)
////                    .strafeLeft(12)
////                    .strafeRight(12)
////
////                    //3)
////                    .lineToLinearHeading(leftBlueStack)
//////                                .lineToLinearHeading(leftBlueMidPos)
////                    .lineToSplineHeading(rightBlueJunction)
////                    .strafeLeft(12)
////                    .build();
//
////            updateSubsystem.start();
//            waitForStart();
////            autoFSM.setState(AutoFSM.AutoState.Start);
//            drive.followTrajectorySequenceAsync(trajectorySequence);
////            autoFSM.setState(AutoFSM.AutoState.DropCone);
////            drive.followTrajectorySequenceAsync(traj2);
////            drive.followTrajectorySequenceAsync(traj2);
//            while (opModeIsActive()){
//                drive.update();
//                clampSubsystem.update();
//                sliderSubsystem.update();
//                telemetry.update();
//            }
//
//
//        }catch (Exception e) {
//            telemetry.addLine(e.toString());
//            telemetry.update();
//        }
//    }
//
//}
