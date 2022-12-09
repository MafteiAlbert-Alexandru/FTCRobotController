package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystem.ClampSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.SliderSubsystem;

@Autonomous
public class OldAutoR extends LinearOpMode {

    Pose2d startPoseRight = new Pose2d(36, -62, Math.toRadians(90));

    Pose2d rightStack = new Pose2d(58, -13.25, Math.toRadians(0));

    Pose2d rightStack2 = new Pose2d(57.5, -13.25, Math.toRadians(0));

    Pose2d rightStack3 = new Pose2d(58, -13.25, Math.toRadians(0));

    Pose2d rightJunction = new Pose2d(36, -14.5, Math.toRadians(0));

    SliderSubsystem sliderSubsystem = new SliderSubsystem();
    ClampSubsystem clampSubsystem = new ClampSubsystem();

    @Override
    public void runOpMode() throws InterruptedException {

        //VARINATA VECHE

        SampleMecanumDriveCancelable drive = new SampleMecanumDriveCancelable(hardwareMap);
        sliderSubsystem.initSubsystem(this, hardwareMap);
        clampSubsystem.initSubsystem(this, hardwareMap);
        drive.setPoseEstimate(startPoseRight);

        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(startPoseRight)
                .addDisplacementMarker(() ->{
                    clampSubsystem.release();
                    TargetPosition(SliderSubsystem.clearPos);
                    clampSubsystem.goToBackward();
                })
                .forward(51)
                .addDisplacementMarker(() ->{
                    TargetPosition(45);
                })
                .forward(1)
                .addDisplacementMarker(() ->{
                    clampSubsystem.clamp();
                })
                .back(4)
                .addDisplacementMarker(() ->{
                    clampSubsystem.goToForward();
                    HighPossliderSubsystem();
                })
                .strafeLeft(11.5)
                .waitSeconds(0.75)
                .addDisplacementMarker(() ->{
                    clampSubsystem.release();
                })
                .strafeRight(12)

                //cycle 1

                .turn(Math.toRadians(-90))
                .addDisplacementMarker(() -> {
                    AimPossliderSubsystem();
                })
                .lineToLinearHeading(rightStack)

                .addDisplacementMarker(() -> {
                    TargetPosition(SliderSubsystem.cone5Pos);
                    clampSubsystem.clamp();
                })
                .back(0.5)
                .addDisplacementMarker(() -> {
                    HighPossliderSubsystem();
                })

                .lineToLinearHeading(rightJunction)
                .turn(Math.toRadians(90))
                .strafeLeft(12)
                .waitSeconds(0.75)
                .addDisplacementMarker(() ->{
                    clampSubsystem.release();
                })
                .strafeRight(12.3)

                //cycle 2

                .turn(Math.toRadians(-90))
                .addDisplacementMarker(() -> {
                    AimPossliderSubsystem();
                })
                .lineToLinearHeading(rightStack)

                .addDisplacementMarker(() -> {
                    TargetPosition(SliderSubsystem.cone4Pos);
                    clampSubsystem.clamp();
                })
                .back(0.5)
                .addDisplacementMarker(() -> {
                    HighPossliderSubsystem();
                })
                .lineToLinearHeading(rightJunction)
                .turn(Math.toRadians(90))
                .strafeLeft(12)
                .waitSeconds(0.75)
                .addDisplacementMarker(() ->{
                    clampSubsystem.release();
                })
                .strafeRight(12)

                //cycle 3

                .turn(Math.toRadians(-90))
                .addDisplacementMarker(() -> {
                    AimPossliderSubsystem();
                })
                .lineToLinearHeading(rightStack2)

                .addDisplacementMarker(() -> {
                    TargetPosition(SliderSubsystem.cone3Pos);
                    clampSubsystem.clamp();
                })
                .back(0.5)
                .addDisplacementMarker(() -> {
                    HighPossliderSubsystem();
                })
                .lineToLinearHeading(rightJunction)
                .turn(Math.toRadians(90))
                .strafeLeft(12)
                .waitSeconds(0.75)
                .addDisplacementMarker(() ->{
                    clampSubsystem.release();
                })
                .strafeRight(12)
                .build();

        waitForStart();

        HighPossliderSubsystem();

        drive.followTrajectorySequenceAsync(traj1);
        if (isStopRequested()) return;

        while (opModeIsActive()){
            drive.update();
            sliderSubsystem.update();
            clampSubsystem.update();
        }
    }

    void HighPossliderSubsystem(){
        sliderSubsystem.target = SliderSubsystem.highPos;
    }

    void AimPossliderSubsystem(){
        sliderSubsystem.target = SliderSubsystem.aimPos;
    }

    void TargetPosition(int pos){
        sliderSubsystem.target = pos;
    }
}