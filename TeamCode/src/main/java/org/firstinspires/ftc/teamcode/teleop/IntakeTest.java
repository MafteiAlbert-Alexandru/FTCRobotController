package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.customHardware.ToggleButton;

@Config
@TeleOp
public class IntakeTest extends LinearOpMode {
    public static double armUpperPos =0.2;
    public static double armLowerPos =0.67;
    public static double legClosedPos = 0;
    public static double legOpenedPos = 0.5;
    public static double intakePower = 0.67;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        try {
            Motor leftIntake = new Motor(hardwareMap, "leftIntake", Motor.GoBILDA.RPM_1620);
            Motor rightIntake = new Motor(hardwareMap, "rightIntake", Motor.GoBILDA.RPM_1620);
            rightIntake.setInverted(true);
            Servo arm = hardwareMap.get(Servo.class, "arm");
            Servo leg = hardwareMap.get(Servo.class, "leg");


            waitForStart();
            GamepadEx operatorGamepad = new GamepadEx(gamepad2);
            ToggleButton intakeToggle = new ToggleButton(operatorGamepad, GamepadKeys.Button.A);
            ToggleButton armToggle = new ToggleButton(operatorGamepad, GamepadKeys.Button.B);
            ToggleButton legToggle = new ToggleButton(operatorGamepad, GamepadKeys.Button.Y);
            while(opModeIsActive()&&!isStopRequested()) {
                operatorGamepad.readButtons();
                intakeToggle.update();
                armToggle.update();
                legToggle.update();

                if (intakeToggle.getToggle()) {
                    leftIntake.set(intakePower);
                    rightIntake.set(intakePower);
                } else {
                    leftIntake.set(0);
                    rightIntake.set(0);
                }
                if (armToggle.getToggle())
                {
                    arm.setPosition(armUpperPos);
                }else arm.setPosition(armLowerPos);
                if(legToggle.getToggle())
                {
                    leg.setPosition(legClosedPos);
                }else  leg.setPosition(legOpenedPos);

            }



        }catch (Exception e)
        {
            telemetry.addLine(e.toString());
            telemetry.update();
        }
    }
}
