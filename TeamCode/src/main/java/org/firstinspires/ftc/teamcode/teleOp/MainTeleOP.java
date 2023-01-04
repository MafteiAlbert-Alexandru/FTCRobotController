package org.firstinspires.ftc.teamcode.teleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.fsm.Robot;
@TeleOp
public class MainTeleOP extends LinearOpMode {

    public ElapsedTime elapsedTime;
    public int beforeEndgameTimer = 85;
    public int beforeStopGame = 115;
    public boolean firstRumble = false;
    public boolean secondRumble = false;

    @Override
    public void runOpMode() throws InterruptedException {

        Gamepad.RumbleEffect rumbleEffect = new Gamepad.RumbleEffect.Builder()
                .addStep(0.0, 1.0, 250)
                .addStep(0.0, 0.0, 250)
                .addStep(1.0, 1.0, 250)
                .addStep(0.0, 0.0, 250)
                .addStep(1.0, 1.0, 250)
                .build();

        PhotonCore.enable();

        telemetry=new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        try{
            Robot robot = new Robot(this, Robot.OpModeType.TeleOp);
            waitForStart();
            elapsedTime = new ElapsedTime();
            elapsedTime.startTime();
            if(isStopRequested()) return;
            while(opModeIsActive()&&!isStopRequested())
            {
                robot.update();
                telemetry.addData("time", elapsedTime.seconds());
                telemetry.update();

                if(gamepad1.touchpad) elapsedTime.reset();

                if(elapsedTime.seconds()>=beforeEndgameTimer && !firstRumble) {
                    gamepad1.runRumbleEffect(rumbleEffect);
                    firstRumble = true;
                }
                else if(elapsedTime.seconds()>=beforeStopGame && !secondRumble) {
                    gamepad1.runRumbleEffect(rumbleEffect);
                    secondRumble = true;
                }
            }

        }catch (Exception e) {
            telemetry.addLine(e.toString());
            telemetry.update();
        }
    }
}
