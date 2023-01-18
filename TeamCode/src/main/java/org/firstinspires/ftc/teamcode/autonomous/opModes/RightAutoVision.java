package org.firstinspires.ftc.teamcode.autonomous.opModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.autonomous.vision.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystem.ClampSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.SliderSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.SliderV2Subsystem;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous
@Config
public class RightAutoVision extends LinearOpMode
{
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    int autoCase = 2;

    TrajectorySequence trajectorySequence;

    SliderSubsystem sliderSubsystem = new SliderSubsystem();
    ClampSubsystem clampSubsystem = new ClampSubsystem();

    public static int speed = 45;

    public static double delayJunction = 0.7;

    public static double delayStack = 0.3;

    public static double delayLift = 0.6;

    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;


    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode()
    {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });
        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == 1 || tag.id == 2 || tag.id == 3){
                        autoCase = tag.id;
                        break;
                    }
                }

            }
            telemetry.update();
        }

        telemetry=new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        try{
            SampleMecanumDriveCancelable drive = new SampleMecanumDriveCancelable(hardwareMap);
            sliderSubsystem.initSubsystem(this);
            clampSubsystem.initSubsystem(this);

            Pose2d startPoseLeft = new Pose2d(36, -60, Math.toRadians(90));
            Pose2d leftBlueMidPos = new Pose2d(36, -12, Math.toRadians(0));


            Pose2d startPoseRight = new Pose2d(36, -60, Math.toRadians(90));

            Pose2d leftBlueStack = new Pose2d(60, -12.5, Math.toRadians(0));

            Pose2d leftBlueStackCycle = new Pose2d(60.3, -12.5, Math.toRadians(0));

            Pose2d rightBlueJunction = new Pose2d(16, -12.5, Math.toRadians(0));


            Pose2d case1Park = new Pose2d(12, -12, Math.toRadians(0));
            Pose2d case2Park = new Pose2d(36, -12, Math.toRadians(0));
            Pose2d case3Park = new Pose2d(60, -12, Math.toRadians(0));

            Pose2d parkPose = case2Park;

            if(autoCase == 1) parkPose = case1Park;
            else if(autoCase == 3) parkPose = case3Park;


            drive.setPoseEstimate(startPoseRight);

            trajectorySequence = drive.trajectorySequenceBuilder(startPoseRight)
                    .addDisplacementMarker(() -> sliderSubsystem.setTarget(SliderV2Subsystem.LowPos))
                    .addTemporalMarker(0.4, () -> clampSubsystem.setPosition(ClampSubsystem.BackwardPos))
                    .addTemporalMarker(0.6, () -> sliderSubsystem.setTarget(SliderV2Subsystem.LoadPos))
                    .addTemporalMarker(0.8, () -> clampSubsystem.clamp())
                    .addTemporalMarker(1.1, () -> sliderSubsystem.setTarget(SliderSubsystem.HighPos))
                    .addTemporalMarker(1.25, () ->clampSubsystem.setPosition(ClampSubsystem.ForwardPos))
                    .forward(49)
                    .strafeLeft(12)
                    .UNSTABLE_addTemporalMarkerOffset(0.3, () -> clampSubsystem.release())
                    .waitSeconds(1)
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
                    .lineToLinearHeading(leftBlueStack, SampleMecanumDriveCancelable.getVelocityConstraint(speed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDriveCancelable.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .addDisplacementMarker(() -> {
                        sliderSubsystem.setTarget(SliderSubsystem.cone5Pos);
                        clampSubsystem.clamp();
                    })
                    .UNSTABLE_addTemporalMarkerOffset(delayLift, () -> sliderSubsystem.setTarget(SliderSubsystem.HighPos))
                    .waitSeconds(delayStack)
                    .back(1)
                    .lineToSplineHeading(rightBlueJunction)
                    .UNSTABLE_addTemporalMarkerOffset(delayJunction, () -> clampSubsystem.release())
                    .strafeLeft(12)
                    .waitSeconds(0.5)
                    .strafeRight(12)
                    //yas
                    .addDisplacementMarker(() -> sliderSubsystem.setTarget(SliderV2Subsystem.AimPos))
                    .lineToLinearHeading(leftBlueStackCycle, SampleMecanumDriveCancelable.getVelocityConstraint(speed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDriveCancelable.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .addDisplacementMarker(() -> {
                        sliderSubsystem.setTarget(SliderSubsystem.cone4Pos);
                        clampSubsystem.clamp();
                    })
                    .UNSTABLE_addTemporalMarkerOffset(delayLift, () -> sliderSubsystem.setTarget(SliderSubsystem.HighPos))
                    .waitSeconds(delayStack)
                    .back(1)
                    .lineToSplineHeading(rightBlueJunction)
                    .UNSTABLE_addTemporalMarkerOffset(delayJunction, () -> clampSubsystem.release())
                    .strafeLeft(12)
                    .waitSeconds(0.5)
                    .strafeRight(12)
                    .addDisplacementMarker(() -> sliderSubsystem.setTarget(SliderV2Subsystem.AimPos))
                    .lineToLinearHeading(leftBlueStackCycle, SampleMecanumDriveCancelable.getVelocityConstraint(speed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDriveCancelable.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .addDisplacementMarker(() -> {
                        sliderSubsystem.setTarget(SliderSubsystem.cone3Pos);
                        clampSubsystem.clamp();
                    })
                    .UNSTABLE_addTemporalMarkerOffset(delayLift, () -> sliderSubsystem.setTarget(SliderSubsystem.HighPos))
                    .waitSeconds(delayStack)
                    .back(1)
                    .lineToSplineHeading(rightBlueJunction)
                    .UNSTABLE_addTemporalMarkerOffset(delayJunction, () -> clampSubsystem.release())
                    .strafeLeft(12)
                    .waitSeconds(0.5)
                    .strafeRight(12)
                    .addDisplacementMarker(() -> sliderSubsystem.setTarget(SliderV2Subsystem.AimPos))
                    .lineToLinearHeading(leftBlueStackCycle, SampleMecanumDriveCancelable.getVelocityConstraint(speed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDriveCancelable.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .addDisplacementMarker(() -> {
                        sliderSubsystem.setTarget(SliderSubsystem.cone2Pos);
                        clampSubsystem.clamp();
                    })
                    .UNSTABLE_addTemporalMarkerOffset(delayLift, () -> sliderSubsystem.setTarget(SliderSubsystem.HighPos))
                    .waitSeconds(delayStack)
                    .back(1)
                    .lineToSplineHeading(rightBlueJunction)
                    .UNSTABLE_addTemporalMarkerOffset(delayJunction, () -> clampSubsystem.release())
                    .strafeLeft(12)
                    .waitSeconds(0.5)
                    .strafeRight(12)
                    .addDisplacementMarker(() -> sliderSubsystem.setTarget(SliderV2Subsystem.AimPos))
                    .lineToLinearHeading(leftBlueStackCycle, SampleMecanumDriveCancelable.getVelocityConstraint(speed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDriveCancelable.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .addDisplacementMarker(() -> {
                        sliderSubsystem.setTarget(SliderSubsystem.cone1Pos);
                        clampSubsystem.clamp();
                    })
                    .UNSTABLE_addTemporalMarkerOffset(delayLift, () -> sliderSubsystem.setTarget(SliderSubsystem.HighPos))
                    .waitSeconds(delayStack)
                    .back(1)
                    .lineToSplineHeading(rightBlueJunction)
                    .UNSTABLE_addTemporalMarkerOffset(delayJunction, () -> clampSubsystem.release())
                    .strafeLeft(12)
                    .waitSeconds(0.5)
                    .strafeRight(12)
                    .lineToLinearHeading(parkPose)
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
            drive.followTrajectorySequenceAsync(trajectorySequence);
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

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));

    }
}