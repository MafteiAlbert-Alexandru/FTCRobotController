package org.firstinspires.ftc.teamcode.teleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.fsm.Robot;
@TeleOp
public class MainTeleOP extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        PhotonCore.enable();

        telemetry=new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        try{
            Robot robot = new Robot(this, Robot.OpModeType.TeleOp);
            waitForStart();
            if(isStopRequested()) return;
            while(opModeIsActive()&&!isStopRequested())
            {
                robot.update();
                telemetry.update();
            }

        }catch (Exception e) {
            telemetry.addLine(e.toString());
            telemetry.update();
        }
    }
}
