package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp
public class SlideTestTeleOP extends LinearOpMode {
    public HomeostatisSliderSubsystem sliderSubsystem = new HomeostatisSliderSubsystem();
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        try {
            SmartSubsystem.initAllSubsystems(this);

            waitForStart();

            while (opModeIsActive()&&!isStopRequested()) {
                sliderSubsystem.run(new SubsystemData());
                telemetry.update();
            }
        } catch (Exception e) {
            telemetry.addLine(e.toString());
            telemetry.update();
        }





    }
}
