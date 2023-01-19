package org.firstinspires.ftc.teamcode.autonomous.opModes.leftAuto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.autonomous.vision.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystem.ClampSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.SliderSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.SliderV2Subsystem;
import org.firstinspires.ftc.teamcode.vision.AprilTagUtil;
import org.openftc.easyopencv.OpenCvCamera;

@Autonomous(name = "LeftParkLoad", group = "left")
public class LeftAutoParkPreload extends LinearOpMode {

    int autoCase = 1;

    TrajectorySequence trajLeft;
    TrajectorySequence trajRight;
    TrajectorySequence trajMid;

    Pose2d startPose = new Pose2d(36, -60, Math.toRadians(90));

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    double tagsize = 0.166;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry=new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        try{
            SampleMecanumDriveCancelable drive = new SampleMecanumDriveCancelable(hardwareMap, startPose);
            SliderSubsystem sliderSubsystem = new SliderSubsystem(this);
            ClampSubsystem clampSubsystem = new ClampSubsystem(this);

            AprilTagUtil aprilTagUtil =  new AprilTagUtil(this, drive);

            FtcDashboard.getInstance().startCameraStream(camera, 10);

            trajLeft = drive.trajectorySequenceBuilder(startPose)
                    .addDisplacementMarker(() -> sliderSubsystem.setTarget(SliderV2Subsystem.LowPos))//Imi ridic glisiera
                    .addTemporalMarker(0.4, () -> clampSubsystem.setPosition(ClampSubsystem.BackwardPos))//Dau in spate hook-ul
                    .addTemporalMarker(0.6, () -> sliderSubsystem.setTarget(SliderV2Subsystem.LoadPos))//Incarc conul din piramida
                    .addTemporalMarker(0.8, clampSubsystem::clamp)//Il agat
                    .addTemporalMarker(1.1, () -> sliderSubsystem.setTarget(SliderSubsystem.HighPos))//Ridic la stalpul inalt
                    .addTemporalMarker(1.25, () ->clampSubsystem.setPosition(ClampSubsystem.ForwardPos))//Dau in fata hook-ul
                    .forward(49)//Merg 49 de inch (fix cat imi trebuie ca sa pun preload-ul)
                    .strafeRight(12)//Ma duc cu fata la stalp
                    .UNSTABLE_addTemporalMarkerOffset(0.2, clampSubsystem::release)//Dau drumul la con
                    .waitSeconds(0.3)//pauza intre actiuni
                    .back(3)//Ma dau in spate ca sa nu lovesc stalpul
                    .addDisplacementMarker(() -> sliderSubsystem.setTarget(0))//Cobor la pozitia sigura
                    .strafeLeft(13)//Ma duc pe tile ul sigur pentru turn
//                    .addDisplacementMarker(() -> drive.followTrajectorySequenceAsync(trajPark))
                    .back(24)
                    .strafeLeft(26)
                    .build();

            trajMid = drive.trajectorySequenceBuilder(startPose)
                    .addDisplacementMarker(() -> sliderSubsystem.setTarget(SliderV2Subsystem.LowPos))//Imi ridic glisiera
                    .addTemporalMarker(0.4, () -> clampSubsystem.setPosition(ClampSubsystem.BackwardPos))//Dau in spate hook-ul
                    .addTemporalMarker(0.6, () -> sliderSubsystem.setTarget(SliderV2Subsystem.LoadPos))//Incarc conul din piramida
                    .addTemporalMarker(0.8, clampSubsystem::clamp)//Il agat
                    .addTemporalMarker(1.1, () -> sliderSubsystem.setTarget(SliderSubsystem.HighPos))//Ridic la stalpul inalt
                    .addTemporalMarker(1.25, () ->clampSubsystem.setPosition(ClampSubsystem.ForwardPos))//Dau in fata hook-ul
                    .forward(49)//Merg 49 de inch (fix cat imi trebuie ca sa pun preload-ul)
                    .strafeRight(12)//Ma duc cu fata la stalp
                    .UNSTABLE_addTemporalMarkerOffset(0.2, clampSubsystem::release)//Dau drumul la con
                    .waitSeconds(0.3)//pauza intre actiuni
                    .back(3)//Ma dau in spate ca sa nu lovesc stalpul
                    .addDisplacementMarker(() -> sliderSubsystem.setTarget(0))//Cobor la pozitia sigura
                    .strafeLeft(13)//Ma duc pe tile ul sigur pentru turn
//                    .addDisplacementMarker(() -> drive.followTrajectorySequenceAsync(trajPark))
                    .back(24)
                    .build();

            trajRight = drive.trajectorySequenceBuilder(startPose)
                    .addDisplacementMarker(() -> sliderSubsystem.setTarget(SliderV2Subsystem.LowPos))//Imi ridic glisiera
                    .addTemporalMarker(0.4, () -> clampSubsystem.setPosition(ClampSubsystem.BackwardPos))//Dau in spate hook-ul
                    .addTemporalMarker(0.6, () -> sliderSubsystem.setTarget(SliderV2Subsystem.LoadPos))//Incarc conul din piramida
                    .addTemporalMarker(0.8, clampSubsystem::clamp)//Il agat
                    .addTemporalMarker(1.1, () -> sliderSubsystem.setTarget(SliderSubsystem.HighPos))//Ridic la stalpul inalt
                    .addTemporalMarker(1.25, () ->clampSubsystem.setPosition(ClampSubsystem.ForwardPos))//Dau in fata hook-ul
                    .forward(49)//Merg 49 de inch (fix cat imi trebuie ca sa pun preload-ul)
                    .strafeRight(12)//Ma duc cu fata la stalp
                    .UNSTABLE_addTemporalMarkerOffset(0.2, clampSubsystem::release)//Dau drumul la con
                    .waitSeconds(0.3)//pauza intre actiuni
                    .back(3)//Ma dau in spate ca sa nu lovesc stalpul
                    .addDisplacementMarker(() -> sliderSubsystem.setTarget(0))//Cobor la pozitia sigura
                    .strafeLeft(13)//Ma duc pe tile ul sigur pentru turn
//                    .addDisplacementMarker(() -> drive.followTrajectorySequenceAsync(trajPark))
                    .back(24)
                    .strafeRight(26)
                    .build();

            //endregion
            waitForStart();
            autoCaseId(aprilTagUtil);

            while (opModeIsActive()){
                drive.update();
                clampSubsystem.update();
                sliderSubsystem.update();
                telemetry.update();
            }


        }catch (Exception e) {
            telemetry.addLine(e.toString());
            telemetry.update();
        }
    }

    void autoCaseId(AprilTagUtil aprilTagUtil){
        int id = aprilTagUtil.getId();
             if(id == 3) aprilTagUtil.sampleMecanumDriveCancelable.followTrajectorySequenceAsync(trajRight);
        else if(id == 2) aprilTagUtil.sampleMecanumDriveCancelable.followTrajectorySequenceAsync(trajMid);
                    else aprilTagUtil.sampleMecanumDriveCancelable.followTrajectorySequenceAsync(trajLeft);
    }

}
