package experiments;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.teamcode.fsm.tudor.StateBot;
import org.firstinspires.ftc.teamcode.subsystem.ClampSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.SliderSubsystem;

@TeleOp
public class StateTester extends LinearOpMode {



    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        GamepadEx leDriver = new GamepadEx(gamepad1);

        SliderSubsystem sliderSubsystem = new SliderSubsystem();
        sliderSubsystem.initSubsystem(this, hardwareMap);
        ClampSubsystem clampSubsystem = new ClampSubsystem();
        clampSubsystem.initSubsystem(this, hardwareMap);

        StateBot stateBot = new StateBot(sliderSubsystem, clampSubsystem);

        Thread stateThread = new Thread( () -> {
            while (opModeIsActive()) {
                stateBot.update();
                telemetry.addData("state", stateBot.botState);
                telemetry.update();
            }
            stop();
        });

        waitForStart();
        stateThread.start();

        while (opModeIsActive()){

            sliderSubsystem.update();
            clampSubsystem.update();

            leDriver.readButtons();

            if(leDriver.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) stateBot.setState(StateBot.BotState.Ground);
            else if(leDriver.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) stateBot.setState(StateBot.BotState.Low);
            else if(leDriver.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) stateBot.setState(StateBot.BotState.Mid);
            else if(leDriver.wasJustPressed(GamepadKeys.Button.DPAD_UP)) stateBot.setState(StateBot.BotState.High);
            else if(leDriver.wasJustPressed(GamepadKeys.Button.X)) stateBot.setState(StateBot.BotState.Load);
            else if(leDriver.wasJustPressed(GamepadKeys.Button.Y)) stateBot.setState(StateBot.BotState.Aim);

        }
        stateThread.interrupt();
    }
}
