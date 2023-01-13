package org.firstinspires.ftc.teamcode.autonomous.opModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.profile.VelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystem.ClampSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.SliderSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.SliderV2Subsystem;
import org.firstinspires.ftc.teamcode.vision.AprilTagUtil;

@Autonomous
public class RightAuto extends LinearOpMode {

//    TrajectorySequence trajectorySequence;

    int speed = 40;
    int speedBack = 40;

    double delayJunction = 0.5;
    double delayStack = 0.6;
    double delayLift = 0.6;
    double delayLoad = 0;

    TrajectorySequence trajLeft;
    TrajectorySequence trajRight;
    TrajectorySequence trajMid;

    int autoCase;

//    Pose2d startPoseLeft = new Pose2d(36, -60, Math.toRadians(90));
    Pose2d startPose = new Pose2d(36, -60, Math.toRadians(90));

//    Pose2d leftBlueMidPos = new Pose2d(36, -12, Math.toRadians(0));
    Pose2d stackPose = new Pose2d(59.25, -12.4, Math.toRadians(0));

    Pose2d stackPoseCycle = new Pose2d(59.25, -12.4, Math.toRadians(0));

    Pose2d rightBlueJunction = new Pose2d(13 ,-12.5, Math.toRadians(0));

    TrajectoryVelocityConstraint velocityConstraint = SampleMecanumDriveCancelable.getVelocityConstraint(speed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH);

    TrajectoryAccelerationConstraint accelerationConstraint = SampleMecanumDriveCancelable.getAccelerationConstraint(DriveConstants.MAX_ACCEL);

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry=new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        try{
            SampleMecanumDriveCancelable drive = new SampleMecanumDriveCancelable(hardwareMap, startPose);
            SliderSubsystem sliderSubsystem = new SliderSubsystem(this);
            ClampSubsystem clampSubsystem = new ClampSubsystem(this);
            AprilTagUtil aprilTagUtil = new AprilTagUtil(this);

            //region RightTrajectorySequence
            trajRight = drive.trajectorySequenceBuilder(startPose)
                    .addDisplacementMarker(() -> sliderSubsystem.setTarget(SliderV2Subsystem.LowPos))
                    .addTemporalMarker(0.4, () -> clampSubsystem.setPosition(ClampSubsystem.BackwardPos))
                    .addTemporalMarker(0.6, () -> sliderSubsystem.setTarget(SliderV2Subsystem.LoadPos))
                    .addTemporalMarker(0.8, clampSubsystem::clamp)
                    .addTemporalMarker(1.1, () -> sliderSubsystem.setTarget(SliderSubsystem.HighPos))
                    .addTemporalMarker(1.25, () ->clampSubsystem.setPosition(ClampSubsystem.ForwardPos))
                    .forward(49)
                    .strafeLeft(12)
                    .UNSTABLE_addTemporalMarkerOffset(0.2, clampSubsystem::release)
                    .waitSeconds(0.3)
                    .back(3)
                    .addDisplacementMarker(() -> sliderSubsystem.setTarget(SliderV2Subsystem.LowPos))
//                    .back(1)
//                    .strafeLeft(2)
                    .strafeRight(13)
                    .addDisplacementMarker(() -> sliderSubsystem.setTarget(SliderV2Subsystem.AimPos))
                    .turn(Math.toRadians(-90))
                    //Pun pe stalp preload ul
                    //Start cycle
                    //1)
                    .lineToLinearHeading(stackPose, SampleMecanumDriveCancelable.getVelocityConstraint(speed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDriveCancelable.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .UNSTABLE_addTemporalMarkerOffset(delayLoad, () -> {
                        sliderSubsystem.setTarget(SliderSubsystem.cone5Pos);
                        clampSubsystem.clamp();
                    })
                    .UNSTABLE_addTemporalMarkerOffset(delayLift, () -> sliderSubsystem.setTarget(SliderSubsystem.HighPos))
                    .waitSeconds(delayStack)
                    .back(1)
                    .lineToLinearHeading(rightBlueJunction, velocityConstraint, accelerationConstraint)
                    .UNSTABLE_addTemporalMarkerOffset(delayJunction, clampSubsystem::release)
                    .strafeLeft(11)
                    .waitSeconds(0.3)
                    .strafeRight(13)
                    //yas
                    .addDisplacementMarker(() -> sliderSubsystem.setTarget(SliderV2Subsystem.AimPos))
                    .lineToLinearHeading(leftBlueStackCycle, SampleMecanumDriveCancelable.getVelocityConstraint(speed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDriveCancelable.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .UNSTABLE_addTemporalMarkerOffset(delayLoad, () -> {
                        sliderSubsystem.setTarget(SliderSubsystem.cone4Pos);    
                        clampSubsystem.clamp();
                    })
                    .UNSTABLE_addTemporalMarkerOffset(delayLift, () -> sliderSubsystem.setTarget(SliderSubsystem.HighPos))
                    .waitSeconds(delayStack)
                    .back(1)
                    .lineToLinearHeading(rightBlueJunction, velocityConstraint, accelerationConstraint)
                    .UNSTABLE_addTemporalMarkerOffset(delayJunction, clampSubsystem::release)
                    .strafeLeft(11)
                    .waitSeconds(0.3)
                    .strafeRight(13)
                    .addDisplacementMarker(() -> sliderSubsystem.setTarget(SliderV2Subsystem.AimPos))
                    .lineToLinearHeading(leftBlueStackCycle, SampleMecanumDriveCancelable.getVelocityConstraint(speed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDriveCancelable.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .UNSTABLE_addTemporalMarkerOffset(delayLoad, () -> {
                        sliderSubsystem.setTarget(SliderSubsystem.cone5Pos);
//                        clampSubsystem.clamp();
                    })
                    .UNSTABLE_addTemporalMarkerOffset(2, () -> sliderSubsystem.setTarget(SliderSubsystem.LowPos))
                    .waitSeconds(delayStack)
                    .back(1)
                    .UNSTABLE_addTemporalMarkerOffset(2, () -> sliderSubsystem.setTarget(0))
//                    .lineToLinearHeading(rightBlueJunction, SampleMecanumDriveCancelable.getVelocityConstraint(speedBack, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                            SampleMecanumDriveCancelable.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                    .UNSTABLE_addTemporalMarkerOffset(delayJunction, () -> clampSubsystem.release())
//                    .strafeLeft(11)
//                    .waitSeconds(0.5)
//                    .strafeRight(13)
                    .back(24)
                    .strafeRight(24)
                    .forward(24)
                    //park
                    .build();

            //endregion1
            //region MidTrajectorySequence
            trajMid = drive.trajectorySequenceBuilder(startPose)
                    .addDisplacementMarker(() -> sliderSubsystem.setTarget(SliderV2Subsystem.LowPos))
                    .addTemporalMarker(0.4, () -> clampSubsystem.setPosition(ClampSubsystem.BackwardPos))
                    .addTemporalMarker(0.6, () -> sliderSubsystem.setTarget(SliderV2Subsystem.LoadPos))
                    .addTemporalMarker(0.8, clampSubsystem::clamp)
                    .addTemporalMarker(1.1, () -> sliderSubsystem.setTarget(SliderSubsystem.HighPos))
                    .addTemporalMarker(1.25, () ->clampSubsystem.setPosition(ClampSubsystem.ForwardPos))
                    .forward(49)
                    .strafeLeft(12)
                    .UNSTABLE_addTemporalMarkerOffset(0.2, clampSubsystem::release)
                    .waitSeconds(0.3)
                    .back(3)
                    .addDisplacementMarker(() -> sliderSubsystem.setTarget(SliderV2Subsystem.LowPos))
//                    .back(1)
//                    .strafeLeft(2)
                    .strafeRight(13)
                    .addDisplacementMarker(() -> sliderSubsystem.setTarget(SliderV2Subsystem.AimPos))
                    .turn(Math.toRadians(-90))
                    //Pun pe stalp preload ul
                    //Start cycle
                    //1)
                    .lineToLinearHeading(stackPose, velocityConstraint, accelerationConstraint)
                    .UNSTABLE_addTemporalMarkerOffset(delayLoad, () -> {
                        sliderSubsystem.setTarget(SliderSubsystem.cone5Pos);
                        clampSubsystem.clamp();
                    })
                    .UNSTABLE_addTemporalMarkerOffset(delayLift, () -> sliderSubsystem.setTarget(SliderSubsystem.HighPos))
                    .waitSeconds(delayStack)
                    .back(1)
                    .lineToLinearHeading(rightBlueJunction, velocityConstraint, accelerationConstraint)
                    .UNSTABLE_addTemporalMarkerOffset(delayJunction, clampSubsystem::release)
                    .strafeLeft(11)
                    .waitSeconds(0.3)
                    .strafeRight(13)
                    //yas
                    .addDisplacementMarker(() -> sliderSubsystem.setTarget(SliderV2Subsystem.AimPos))
                    .lineToLinearHeading(stackPoseCycle, velocityConstraint, accelerationConstraint)
                    .UNSTABLE_addTemporalMarkerOffset(delayLoad, () -> {
                        sliderSubsystem.setTarget(SliderSubsystem.cone4Pos);
                        clampSubsystem.clamp();
                    })
                    .UNSTABLE_addTemporalMarkerOffset(delayLift, () -> sliderSubsystem.setTarget(SliderSubsystem.HighPos))
                    .waitSeconds(delayStack)
                    .back(1)
                    .lineToLinearHeading(rightBlueJunction, velocityConstraint, accelerationConstraint)
                    .UNSTABLE_addTemporalMarkerOffset(delayJunction, clampSubsystem::release)
                    .strafeLeft(11)
                    .waitSeconds(0.3)
                    .strafeRight(13)
                    .addDisplacementMarker(() -> sliderSubsystem.setTarget(SliderV2Subsystem.AimPos))
                    .lineToLinearHeading(stackPoseCycle, velocityConstraint, accelerationConstraint)
                    .UNSTABLE_addTemporalMarkerOffset(delayLoad, () -> {
                        sliderSubsystem.setTarget(SliderSubsystem.cone5Pos);
//                        clampSubsystem.clamp();
                    })
                    .UNSTABLE_addTemporalMarkerOffset(2, () -> sliderSubsystem.setTarget(SliderSubsystem.LowPos))
                    .waitSeconds(delayStack)
                    .back(1)
                    .UNSTABLE_addTemporalMarkerOffset(2, () -> sliderSubsystem.setTarget(0))
//                    .lineToLinearHeading(rightBlueJunction, SampleMecanumDriveCancelable.getVelocityConstraint(speedBack, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                            SampleMecanumDriveCancelable.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                    .UNSTABLE_addTemporalMarkerOffset(delayJunction, () -> clampSubsystem.release())
//                    .strafeLeft(11)
//                    .waitSeconds(0.5)
//                    .strafeRight(13)
                    .back(24)
                    .turn(90)
                    //park
                    .build();

            //endregion1
            //region LeftTrajectorySequence
            trajLeft = drive.trajectorySequenceBuilder(startPose)
                    .addDisplacementMarker(() -> sliderSubsystem.setTarget(SliderV2Subsystem.LowPos))//Imi ridic glisiera
                    .addTemporalMarker(0.4, () -> clampSubsystem.setPosition(ClampSubsystem.BackwardPos))//Dau in spate hook-ul
                    .addTemporalMarker(0.6, () -> sliderSubsystem.setTarget(SliderV2Subsystem.LoadPos))//Incarc conul din piramida
                    .addTemporalMarker(0.8, clampSubsystem::clamp)//Il agat
                    .addTemporalMarker(1.1, () -> sliderSubsystem.setTarget(SliderSubsystem.HighPos))//Ridic la stalpul inalt
                    .addTemporalMarker(1.25, () ->clampSubsystem.setPosition(ClampSubsystem.ForwardPos))//Dau in fata hook-ul
                    .forward(49)//Merg 49 de inch (fix cat imi trebuie ca sa pun preload-ul
                    .strafeLeft(12)//Ma duc cu fata la stalp
                    .UNSTABLE_addTemporalMarkerOffset(0.2, clampSubsystem::release)//Dau drumul la con
                    .waitSeconds(0.3)//pauza intre actiuni
                    .back(3)//Ma dau in spate ca sa nu lovesc stalpul
                    .addDisplacementMarker(() -> sliderSubsystem.setTarget(SliderV2Subsystem.LowPos))//Cobor la pozitia sigura
                    .strafeRight(13)//Ma duc pe tile ul sigur pentru turn
                    .addDisplacementMarker(() -> sliderSubsystem.setTarget(SliderV2Subsystem.AimPos))//Cobor glisiera ca sa fiu putin peste stack
                    .turn(Math.toRadians(-90))//Ma rotesc ca sa fiu cu fata la stack
                    .lineToLinearHeading(stackPose, SampleMecanumDriveCancelable.getVelocityConstraint(speed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDriveCancelable.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .UNSTABLE_addTemporalMarkerOffset(delayLoad, () -> {
                        sliderSubsystem.setTarget(SliderSubsystem.cone5Pos);
                        clampSubsystem.clamp();
                    })
                    .UNSTABLE_addTemporalMarkerOffset(delayLift, () -> sliderSubsystem.setTarget(SliderSubsystem.HighPos))
                    .waitSeconds(delayStack)
                    .back(1)
                    .lineToLinearHeading(rightBlueJunction, velocityConstraint, accelerationConstraint)
                    .UNSTABLE_addTemporalMarkerOffset(delayJunction, clampSubsystem::release)
                    .strafeLeft(11)
                    .waitSeconds(0.3)
                    .strafeRight(13)

                    //first cycle
                    .addDisplacementMarker(() -> sliderSubsystem.setTarget(SliderV2Subsystem.AimPos))
                    .lineToLinearHeading(leftBlueStackCycle, SampleMecanumDriveCancelable.getVelocityConstraint(speed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDriveCancelable.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .UNSTABLE_addTemporalMarkerOffset(delayLoad, () -> {
                        sliderSubsystem.setTarget(SliderSubsystem.cone4Pos);
                        clampSubsystem.clamp();
                    })
                    .UNSTABLE_addTemporalMarkerOffset(delayLift, () -> sliderSubsystem.setTarget(SliderSubsystem.HighPos))
                    .waitSeconds(delayStack)
                    .back(1)
                    .lineToLinearHeading(rightBlueJunction, velocityConstraint, accelerationConstraint)
                    .UNSTABLE_addTemporalMarkerOffset(delayJunction, clampSubsystem::release)
                    .strafeLeft(11)
                    .waitSeconds(0.3)
                    .strafeRight(13)

                    //last cycle
                    .addDisplacementMarker(() -> sliderSubsystem.setTarget(SliderV2Subsystem.AimPos))
                    .lineToLinearHeading(leftBlueStackCycle, SampleMecanumDriveCancelable.getVelocityConstraint(speed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDriveCancelable.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .UNSTABLE_addTemporalMarkerOffset(delayLoad, () -> sliderSubsystem.setTarget(SliderSubsystem.cone5Pos))
                    .UNSTABLE_addTemporalMarkerOffset(2, () -> sliderSubsystem.setTarget(SliderSubsystem.LowPos))
                    .waitSeconds(delayStack)
                    .back(1)
                    .UNSTABLE_addTemporalMarkerOffset(2, () -> sliderSubsystem.setTarget(0))
                    .back(48)
                    .turn(90)
                    //park
                    .build();

            //endregion1

            waitForStart();
            autoCaseId(aprilTagUtil);

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

    void autoCaseId(AprilTagUtil aprilTagUtil){
        int id = aprilTagUtil.getId();
        SampleMecanumDriveCancelable driveCancelable = aprilTagUtil.sampleMecanumDriveCancelable;
             if(id == 3) driveCancelable.followTrajectorySequenceAsync(trajRight);
        else if(id == 2) driveCancelable.followTrajectorySequenceAsync(trajMid);
                    else driveCancelable.followTrajectorySequenceAsync(trajLeft);
    }

}
