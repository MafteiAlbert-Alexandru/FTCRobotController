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
    public static PIDExCoefficients posCoefficients = new PIDExCoefficients(0.12,0.3,0.25, 0.25, 0.004);
    public static PIDExCoefficients veloCoefficients = new PIDExCoefficients(1,1,1,1,1);
    public static int target = 0;
    public static int speed = 0;
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
        motor= new SmartMotorEx(hardwareMap, "motor", SmartMotor.GoBILDA.RPM_312);
        waitForStart();
        motor.resetEncoder();
        motor.setRunMode(SmartMotor.RunMode.RawPower);
        PIDExController posController= new PIDExController(telemetry, posCoefficients);
        PIDExController veloController = new PIDExController(telemetry, veloCoefficients);
        double power=0;

        while(opModeIsActive()&&!isStopRequested())
        {
            double commandSpeed=posController.update(target, motor.getCurrentPosition());
            double targetSpeed = Math.signum(commandSpeed)*Math.min(1.0, Math.abs(commandSpeed))*speed;
            double speed = getSpeed();
            power += veloController.update(targetSpeed, speed)* veloController.lastDT();
            if(Math.abs(power)>1.0)
                power=Math.signum(power);
            motor.setPower(power);
            telemetry.addData("speed", targetSpeed);
            telemetry.addData("velo", speed);

            telemetry.addData("power", power);
            telemetry.addData("position", motor.getCurrentPosition());
            telemetry.update();
        }
    }
}
