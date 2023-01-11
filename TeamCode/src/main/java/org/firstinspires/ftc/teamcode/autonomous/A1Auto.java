package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystem.ClampSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.SliderSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.SliderV2Subsystem;

@Autonomous
public class A1Auto extends LinearOpMode {

    TrajectorySequence preLoadTraj, stackTraj, junctionTraj;

    SliderSubsystem sliderSubsystem = new SliderSubsystem();
    ClampSubsystem clampSubsystem = new ClampSubsystem();


    @Override
    public void runOpMode() throws InterruptedException {
//        Thread.currentThread().setPriority(10);
        telemetry=new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        try{
            SampleMecanumDriveCancelable drive = new SampleMecanumDriveCancelable(hardwareMap);
            sliderSubsystem.initSubsystem(this);
            clampSubsystem.initSubsystem(this);
//
//            Thread updateSubsystem = new Thread(
//                    () -> {
//                        Thread.currentThread().setPriority(1);
//                        waitForStart();
//                        while (opModeIsActive() && !isStopRequested()) {
//                            try {
//                                sliderSubsystem.update();
//                                clampSubsystem.update();
//                            } catch (InterruptedException e) {
//                                telemetry.addLine(e.toString());
//                            }
//                            telemetry.update();
//                        }
//                    }
//            );

//            AutoFSM autoFSM = new AutoFSM(this, sliderSubsystem, clampSubsystem);

            Pose2d startPoseLeft = new Pose2d(36, -60, Math.toRadians(90));
            Pose2d startPoseRight = new Pose2d(36, -60, Math.toRadians(90));

            Pose2d leftBlueMidPos = new Pose2d(36, -12, Math.toRadians(0));
            Pose2d leftBlueStack = new Pose2d(59.6, -13.7, Math.toRadians(0));

            Pose2d rightBlueJunction = new Pose2d(15, -12, Math.toRadians(0));

            drive.setPoseEstimate(startPoseRight);

            preLoadTraj = drive.trajectorySequenceBuilder(startPoseRight)
                    .addDisplacementMarker(() -> sliderSubsystem.setTarget(SliderV2Subsystem.LowPos))
                    .addTemporalMarker(0.4, () -> clampSubsystem.setPosition(ClampSubsystem.BackwardPos))
                    .addTemporalMarker(0.6, () -> sliderSubsystem.setTarget(SliderV2Subsystem.LoadPos))
                    .addTemporalMarker(0.8, () -> clampSubsystem.clamp())
                    .addTemporalMarker(1.1, () -> sliderSubsystem.setTarget(SliderSubsystem.HighPos))
                    .addTemporalMarker(1.25, () ->clampSubsystem.setPosition(ClampSubsystem.ForwardPos))
                    .forward(47)
                    .strafeLeft(12)
                    .addDisplacementMarker(() -> clampSubsystem.release())
                    .forward(1)
//                    .forward(1)
                    .back(3)
                    .addDisplacementMarker(() -> sliderSubsystem.setTarget(SliderV2Subsystem.LowPos))
//                    .back(1)
//                    .strafeLeft(2)
                    .strafeRight(12)
                    .addDisplacementMarker(() -> sliderSubsystem.setTarget(SliderV2Subsystem.AimPos))
                    .turn(Math.toRadians(-90))
                    //Pun pe stalp preload ul
                    //Start cycle
                    //1)
                    .lineToLinearHeading(leftBlueStack)
                    .addDisplacementMarker(() -> {
                        sliderSubsystem.setTarget(SliderSubsystem.StackLoad);
                        clampSubsystem.clamp();
                    })
                    .waitSeconds(0.2)
                    .addDisplacementMarker(() -> sliderSubsystem.setTarget(SliderSubsystem.HighPos))
                    .lineToSplineHeading(rightBlueJunction)
                    .strafeLeft(12)
                    .build();


//            traj2 = drive.trajectorySequenceBuilder(traj1.end())
//                    .strafeRight(12)
//
//                    //2)
//                    .lineToLinearHeading(leftBlueStack)
////                                .lineToLinearHeading(leftBlueMidPos)
//                    .lineToSplineHeading(rightBlueJunction)
//                    .strafeLeft(12)
//                    .strafeRight(12)
//
//                    //3)
//                    .lineToLinearHeading(leftBlueStack)
////                                .lineToLinearHeading(leftBlueMidPos)
//                    .lineToSplineHeading(rightBlueJunction)
//                    .strafeLeft(12)
//                    .build();

//            updateSubsystem.start();
            waitForStart();
//            autoFSM.setState(AutoFSM.AutoState.Start);
            drive.followTrajectorySequenceAsync(preLoadTraj);
//            autoFSM.setState(AutoFSM.AutoState.DropCone);
//            drive.followTrajectorySequenceAsync(traj2);
//            drive.followTrajectorySequenceAsync(traj2);
            while (opModeIsActive()){
                drive.update();
                clampSubsystem.update();
                sliderSubsystem.update();
            }


        }catch (Exception e) {
            telemetry.addLine(e.toString());
            telemetry.update();
        }
    }

}
