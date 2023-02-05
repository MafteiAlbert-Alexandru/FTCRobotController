package org.firstinspires.ftc.teamcode.experiments;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.SmartMotor;
import org.firstinspires.ftc.teamcode.hardware.SmartMotorEx;

@TeleOp
@Config
public class TestTeleOP extends LinearOpMode {
    public static PIDExCoefficients posCoefficients = new PIDExCoefficients(0.12,0.03,0.15, 0.25, 0.004);
    public static PIDExCoefficients veloCoefficients = new PIDExCoefficients(0.3,0.001,0.2,0.1,0.001);
    public static int target = 0;
    public static int speed = 0;
    public static int tolerance=5;
    private SmartMotorEx motor;


    private long lastTime = -1;
    private int lastPosition=0;
    private double getSpeed()
    {
        if(lastTime==-1)
        {
            lastTime=System.currentTimeMillis();
            lastPosition=motor.getCurrentPosition();
            return 0;
        }else {
            long time = System.currentTimeMillis();
            double deltaTime = (time-lastTime)/1000.0;
            int position = motor.getCurrentPosition();
            double speed = (position-lastPosition)/deltaTime;
            lastPosition=position;
            lastTime=time;
            return speed;
        }

    }
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry=new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        motor= new SmartMotorEx(hardwareMap, "slider", SmartMotor.NeveRest.RPM_1780);
        waitForStart();
        motor.resetEncoder();
        motor.setRunMode(SmartMotor.RunMode.RawPower);
        PIDExController posController= new PIDExController(posCoefficients);
        PIDExController veloController = new PIDExController(veloCoefficients);
        double power=0;
        motor.setZeroPowerBehavior(SmartMotor.ZeroPowerBehavior.BRAKE);

        while(opModeIsActive()&&!isStopRequested())
        {
            double position = motor.getCurrentPosition();
            double commandSpeed=posController.update(target, position);
            double targetSpeed = Math.signum(commandSpeed)*Math.min(1.0, Math.abs(commandSpeed))*speed;
            double speed =getSpeed();
            power += veloController.update(targetSpeed, speed)* veloController.lastDT();
            if(Math.abs(motor.getCurrentPosition()-target)<tolerance)
                motor.setPower(0);
            motor.setPower(power);
            telemetry.addData("tspeed",targetSpeed);
            telemetry.addData("speed", speed);

            telemetry.addData("power", power);
            telemetry.addData("position", motor.getCurrentPosition());
            telemetry.update();
        }
    }
}
