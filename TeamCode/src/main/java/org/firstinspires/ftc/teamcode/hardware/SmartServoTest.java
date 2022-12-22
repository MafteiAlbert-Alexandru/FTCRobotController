package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Config
public class SmartServoTest extends LinearOpMode {
    public static double position=0;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry=new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        SmartServo servo = new SmartServo(hardwareMap.get(Servo.class, "leg"), SmartServo.SmartServoType.GOBILDA_SPEED);
        servo.setPosition(position);
        telemetry.addData("position", servo.getPosition(true));
        telemetry.update();
        waitForStart();

        while(opModeIsActive()&&!isStopRequested())
        {
            servo.setPosition(position);

            telemetry.addData("position", servo.getPosition(true));
            telemetry.update();
        }

    }
}
