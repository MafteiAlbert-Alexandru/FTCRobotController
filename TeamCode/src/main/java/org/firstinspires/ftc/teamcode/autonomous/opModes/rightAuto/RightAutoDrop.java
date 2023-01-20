package org.firstinspires.ftc.teamcode.autonomous.opModes.rightAuto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystem.ClampSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.SliderSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.SliderV2Subsystem;
import org.firstinspires.ftc.teamcode.vision.AprilTagUtil;

import java.util.List;
//love you tudor<3

//@Disabled
@Autonomous(group = "right")
public class RightAutoDrop extends LinearOpMode {

    int speed = 40;

    double delayJunction = 0.5;
    double delayStack = 0.6;
    double delayLift = 0.6;
    double delayLoad = 0;

    TrajectorySequence trajLeft;
    TrajectorySequence trajRight;
    TrajectorySequence trajMid;

    Pose2d startPose = new Pose2d(36, -60, Math.toRadians(90));
    Pose2d stackPose = new Pose2d(59.25, -12.4, Math.toRadians(0));
    Pose2d stackPoseCycle = new Pose2d(58.5, -12, Math.toRadians(0));
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
//            AprilTagUtil aprilTagUtil = new AprilTagUtil(this);

            //region RightTrajectorySequence
            trajRight = drive.trajectorySequenceBuilder(startPose)
                    .addDisplacementMarker(() -> sliderSubsystem.setTarget(SliderV2Subsystem.LowPos))//Imi ridic glisiera
                    .addTemporalMarker(0.4, () -> clampSubsystem.setPosition(ClampSubsystem.BackwardPos))//Dau in spate hook-ul
                    .addTemporalMarker(0.6, () -> sliderSubsystem.setTarget(SliderV2Subsystem.LoadPos))//Incarc conul din piramida
                    .addTemporalMarker(0.8, clampSubsystem::clamp)//Il agat
                    .addTemporalMarker(1.1, () -> sliderSubsystem.setTarget(SliderSubsystem.HighPos))//Ridic la stalpul inalt
                    .addTemporalMarker(1.25, () ->clampSubsystem.setPosition(ClampSubsystem.ForwardPos))//Dau in fata hook-ul
                    .forward(49)//Merg 49 de inch (fix cat imi trebuie ca sa pun preload-ul)
                    .strafeLeft(12)//Ma duc cu fata la stalp
                    .UNSTABLE_addTemporalMarkerOffset(0.3, clampSubsystem::release)//Dau drumul la con
                    .waitSeconds(0.4)//pauza intre actiuni
                    .back(3)//Ma dau in spate ca sa nu lovesc stalpul
                    .addDisplacementMarker(() -> sliderSubsystem.setTarget(SliderV2Subsystem.LowPos))//Cobor la pozitia sigura
                    .strafeRight(13)//Ma duc pe tile ul sigur pentru turn
                    .addDisplacementMarker(() -> sliderSubsystem.setTarget(SliderV2Subsystem.AimPos))//Cobor glisiera ca sa fiu putin peste stack
                    .turn(Math.toRadians(-90))//Ma rotesc ca sa fiu cu fata la stack
                    .lineToLinearHeading(stackPose, velocityConstraint, accelerationConstraint)//merg la stack
                    .UNSTABLE_addTemporalMarkerOffset(delayLoad, () -> { //iau conul 5
                        sliderSubsystem.setTarget(SliderSubsystem.cone5Pos);
                        clampSubsystem.clamp();
                    })
                    .UNSTABLE_addTemporalMarkerOffset(delayLift, () -> sliderSubsystem.setTarget(SliderSubsystem.HighPos))//ridic glisiera
                    .waitSeconds(delayStack)//ma chillez
                    .back(1)//sa nu zbor conu
                    .lineToLinearHeading(rightBlueJunction, velocityConstraint, accelerationConstraint)//ma duc la junction
                    .UNSTABLE_addTemporalMarkerOffset(delayJunction, clampSubsystem::release)//dau drumu la con dupa cateva milisec
                    .strafeLeft(11)//ma centrez pe junction
                    .waitSeconds(0.3)
                    .strafeRight(13)//ma intorc inapoi

                    //first cycle
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
                    .strafeRight(37)
                    .forward(48)
//
//                    //last cycle
//                    .addDisplacementMarker(() -> sliderSubsystem.setTarget(SliderV2Subsystem.AimPos))
//                    .lineToLinearHeading(stackPoseCycle, velocityConstraint, accelerationConstraint)
//                    .UNSTABLE_addTemporalMarkerOffset(delayLoad, () -> sliderSubsystem.setTarget(SliderSubsystem.cone5Pos))
//                    .UNSTABLE_addTemporalMarkerOffset(2, () -> sliderSubsystem.setTarget(SliderSubsystem.LowPos))
//                    .waitSeconds(delayStack)
//                    .back(1)
//                    .UNSTABLE_addTemporalMarkerOffset(2, () -> sliderSubsystem.setTarget(0))

                    //PARK
                    .back(24)
                    .strafeRight(24)
                    .forward(24)
                    //park
                    .build();

            //endregion1
            //region MidTrajectorySequence
            trajMid = drive.trajectorySequenceBuilder(startPose)
                    .addDisplacementMarker(() -> sliderSubsystem.setTarget(SliderV2Subsystem.LowPos))//Imi ridic glisiera
                    .addTemporalMarker(0.4, () -> clampSubsystem.setPosition(ClampSubsystem.BackwardPos))//Dau in spate hook-ul
                    .addTemporalMarker(0.6, () -> sliderSubsystem.setTarget(SliderV2Subsystem.LoadPos))//Incarc conul din piramida
                    .addTemporalMarker(0.8, clampSubsystem::clamp)//Il agat
                    .addTemporalMarker(1.1, () -> sliderSubsystem.setTarget(SliderSubsystem.HighPos))//Ridic la stalpul inalt
                    .addTemporalMarker(1.25, () ->clampSubsystem.setPosition(ClampSubsystem.ForwardPos))//Dau in fata hook-ul
                    .forward(49.3)//Merg 49 de inch (fix cat imi trebuie ca sa pun preload-ul)
                    .strafeLeft(12)//Ma duc cu fata la stalp
                    .UNSTABLE_addTemporalMarkerOffset(0.3, clampSubsystem::release)//Dau drumul la con
                    .waitSeconds(0.4)//pauza intre actiuni
                    .back(3)//Ma dau in spate ca sa nu lovesc stalpul
                    .addDisplacementMarker(() -> sliderSubsystem.setTarget(SliderV2Subsystem.LowPos))//Cobor la pozitia sigura
                    .strafeRight(13)//Ma duc pe tile ul sigur pentru turn
                    .addDisplacementMarker(() -> sliderSubsystem.setTarget(SliderV2Subsystem.AimPos))//Cobor glisiera ca sa fiu putin peste stack
                    .turn(Math.toRadians(-90))//Ma rotesc ca sa fiu cu fata la stack
                    .lineToLinearHeading(stackPose, velocityConstraint, accelerationConstraint)//merg la stack
                    .UNSTABLE_addTemporalMarkerOffset(delayLoad, () -> { //iau conul 5
                        sliderSubsystem.setTarget(SliderSubsystem.cone5Pos);
                        clampSubsystem.clamp();
                    })
                    .UNSTABLE_addTemporalMarkerOffset(delayLift, () -> sliderSubsystem.setTarget(SliderSubsystem.HighPos))//ridic glisiera
                    .waitSeconds(delayStack)//ma chillez
                    .back(1)//sa nu zbor conu
                    .lineToLinearHeading(rightBlueJunction, velocityConstraint, accelerationConstraint)//ma duc la junction
                    .UNSTABLE_addTemporalMarkerOffset(delayJunction, clampSubsystem::release)//dau drumu la con dupa cateva milisec
                    .strafeLeft(11)//ma centrez pe junction
                    .waitSeconds(0.3)
                    .strafeRight(13)//ma intorc inapoi

                    //first cycle
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


                    //last cycle
                    .UNSTABLE_addTemporalMarkerOffset(2, () -> sliderSubsystem.setTarget(0))

                    //PARK
//                    .back(24)
                    .turn(Math.toRadians(90))
                    .back(24)
                    .strafeRight(48)
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
                    .forward(49)//Merg 49 de inch (fix cat imi trebuie ca sa pun preload-ul)
                    .strafeLeft(12)//Ma duc cu fata la stalp
                    .UNSTABLE_addTemporalMarkerOffset(0.2, clampSubsystem::release)//Dau drumul la con
                    .waitSeconds(0.3)//pauza intre actiuni
                    .back(3)//Ma dau in spate ca sa nu lovesc stalpul
                    .addDisplacementMarker(() -> sliderSubsystem.setTarget(SliderV2Subsystem.LowPos))//Cobor la pozitia sigura
                    .strafeRight(13)//Ma duc pe tile ul sigur pentru turn
                    .addDisplacementMarker(() -> sliderSubsystem.setTarget(SliderV2Subsystem.AimPos))//Cobor glisiera ca sa fiu putin peste stack
                    .turn(Math.toRadians(-90))//Ma rotesc ca sa fiu cu fata la stack
                    .lineToLinearHeading(stackPose, velocityConstraint, accelerationConstraint)//merg la stack
                    .UNSTABLE_addTemporalMarkerOffset(delayLoad, () -> { //iau conul 5
                        sliderSubsystem.setTarget(SliderSubsystem.cone5Pos);
                        clampSubsystem.clamp();
                    })
                    .UNSTABLE_addTemporalMarkerOffset(delayLift, () -> sliderSubsystem.setTarget(SliderSubsystem.HighPos))//ridic glisiera
                    .waitSeconds(delayStack)//ma chillez
                    .back(1)//sa nu zbor conu
                    .lineToLinearHeading(rightBlueJunction, velocityConstraint, accelerationConstraint)//ma duc la junction
                    .UNSTABLE_addTemporalMarkerOffset(delayJunction, clampSubsystem::release)//dau drumu la con dupa cateva milisec
                    .strafeLeft(11)//ma centrez pe junction
                    .waitSeconds(0.3)
                    .strafeRight(13)//ma intorc inapoi

                    //first cycle
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

                    //last cycle
                    .addDisplacementMarker(() -> sliderSubsystem.setTarget(SliderV2Subsystem.AimPos))
                    .lineToLinearHeading(stackPoseCycle, velocityConstraint, accelerationConstraint)
                    .UNSTABLE_addTemporalMarkerOffset(delayLoad, () -> sliderSubsystem.setTarget(SliderSubsystem.cone5Pos))
                    .UNSTABLE_addTemporalMarkerOffset(2, () -> sliderSubsystem.setTarget(SliderSubsystem.LowPos))
                    .waitSeconds(delayStack)
                    .back(1)
                    .UNSTABLE_addTemporalMarkerOffset(2, () -> sliderSubsystem.setTarget(0))

                    //PARK
                    .back(48)
                    .turn(Math.toRadians(90))
                    .build();

            //endregion1

            waitForStart();
            drive.followTrajectorySequenceAsync(trajMid);
            List<LynxModule> modules = hardwareMap.getAll(LynxModule.class);
            while (opModeIsActive()){
                for(LynxModule module:modules) module.clearBulkCache();
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
