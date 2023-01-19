package org.firstinspires.ftc.teamcode.autonomous.opModes.rightAuto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystem.ClampSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.SliderSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.SliderV2Subsystem;
import org.firstinspires.ftc.teamcode.vision.AprilTagUtil;

@Autonomous(group = "right")
@Disabled
public class RightAutoDrop3Cone extends LinearOpMode {

    int speed = 40;

    double delayJunction = 0.65;
    double delayStack = 0.6;
    double delayLift = 0.6;
    double delayLoad = 0;


    Pose2d startPose = new Pose2d(36, -60, Math.toRadians(0));
    Pose2d stackPose = new Pose2d(60, -12.16, Math.toRadians(180));
    Pose2d stackPoseCycle = new Pose2d(59.2, -12.6, Math.toRadians(0));
    Pose2d highJunction = new Pose2d(13.5 ,-12.5, Math.toRadians(0));
    Pose2d lowJunction = new Pose2d(37 ,-12.5, Math.toRadians(0));

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

            //region TrajectorySequence
            TrajectorySequence traj = drive.trajectorySequenceBuilder(startPose)
                    .addDisplacementMarker(() -> sliderSubsystem.setTarget(SliderV2Subsystem.LowPos))//Imi ridic glisiera
                    .addTemporalMarker(0.4, () -> clampSubsystem.setPosition(ClampSubsystem.BackwardPos))//Dau in spate hook-ul
                    .addTemporalMarker(0.6, () -> sliderSubsystem.setTarget(SliderV2Subsystem.LoadPos))//Incarc conul din piramida
                    .addTemporalMarker(0.8, clampSubsystem::clamp)//Il agat
                    .addTemporalMarker(1.1, () -> sliderSubsystem.setTarget(SliderSubsystem.HighPos))//Ridic la stalpul inalt
                    .addTemporalMarker(1.25, () ->clampSubsystem.setPosition(ClampSubsystem.ForwardPos))//Dau in fata hook-ul
                    .strafeLeft(49)
                    .back(26)//Ma duc cu fata la stalp
                    .strafeLeft(24)
                    .UNSTABLE_addTemporalMarkerOffset(0.2, clampSubsystem::release)//Dau drumul la con
                    .waitSeconds(0.3)//pauza intre actiuni
                    //o pus conu
                    .strafeRight(26)
                    .lineToLinearHeading(stackPose)
                    .splineToLinearHeading(new Pose2d(12.45, -0.66, Math.toRadians(0.00)), Math.toRadians(86.10))
                    .build();


                    TrajectorySequence stackTraj = drive.trajectorySequenceBuilder(stackPose)
                    .build();


            //endregion1

            //region ParkTrajectory

//            trajLeft = drive.trajectorySequenceBuilder(traj.end())
//                    .strafeRight(12)
//                    .back(24)
//                    .turn(Math.toRadians(90))
//                    .build();
//
//            trajMid = drive.trajectorySequenceBuilder(traj.end())
//                    .strafeRight(12)
//                    .turn(Math.toRadians(90))
//                    .build();
//
//            trajRight = drive.trajectorySequenceBuilder(traj.end())
//                    .strafeRight(12)
//                    .forward(24)
//                    .turn(Math.toRadians(90))
//                    .build();

            //endregion

            waitForStart();
//            autoCaseId(aprilTagUtil);
                drive.followTrajectorySequenceAsync(traj);

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

//    void autoCaseId(AprilTagUtil aprilTagUtil){
//        int id = aprilTagUtil.getId();
////        SampleMecanumDriveCancelable driveCancelable = aprilTagUtil.sampleMecanumDriveCancelable;
//             if(id == 3) trajPark = trajRight;
//        else if(id == 2) trajPark = trajMid;
//                    else trajPark = trajLeft;
//    }

}
