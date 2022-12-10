package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystem.ClampSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.SliderSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.SliderSubsystemAuto;
import org.firstinspires.ftc.teamcode.subsystem.SubsystemData;

@Autonomous
public class OldAutoR extends LinearOpMode {

    Pose2d startPoseRight = new Pose2d(36, -62, Math.toRadians(90));

    Pose2d rightStack = new Pose2d(60, -11, Math.toRadians(0));

    Pose2d rightStack2 = new Pose2d(61, -11, Math.toRadians(0));

    Pose2d rightStack3 = new Pose2d(61, -11, Math.toRadians(0));

    Pose2d rightJunction = new Pose2d(32, -12, Math.toRadians(0));

    SliderSubsystemAuto sliderSubsystem = new SliderSubsystemAuto();
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
                    TargetPosition(SliderSubsystem.ClearPos);
                    clampSubsystem.goToBackward();
                })
                .forward(53)
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
                .strafeLeft(13)
                .addDisplacementMarker(() ->{
                    clampSubsystem.release();
                })
                .waitSeconds(0.5)
                .strafeLeft(0.5)
                .back(1)
                .strafeRight(12.5)

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
                .strafeLeft(8)
                .addDisplacementMarker(() ->{
                    clampSubsystem.release();
                })
                .waitSeconds(0.2)
                .strafeLeft(0.5)
                .strafeRight(12.5)

                .turn(Math.toRadians(-90))
                .addDisplacementMarker(() -> {
                    AimPossliderSubsystem();
                })
                .lineToLinearHeading(rightStack2)

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
                .strafeLeft(7.5)
                .addDisplacementMarker(() ->{
                    clampSubsystem.release();
                })
                .waitSeconds(0.2)
                .strafeLeft(0.5)
                .strafeRight(12.5)

                //cycle 3

                .turn(Math.toRadians(-90))
                .addDisplacementMarker(() -> {
                    AimPossliderSubsystem();
                })
                .lineToLinearHeading(rightStack3)

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
                .strafeLeft(6)
                .addDisplacementMarker(() ->{
                    clampSubsystem.release();
                })
                .waitSeconds(0.2)
                .strafeLeft(0.5)
                .strafeRight(12.5)
                .addDisplacementMarker( () -> {
                    clampSubsystem.goToBackward();
                    sliderSubsystem.goToTake();
                })
                .build();

        waitForStart();

        HighPossliderSubsystem();

        drive.followTrajectorySequenceAsync(traj1);
        if (isStopRequested()) return;

        while (opModeIsActive()){
            drive.update();
            sliderSubsystem.run(new SubsystemData());
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