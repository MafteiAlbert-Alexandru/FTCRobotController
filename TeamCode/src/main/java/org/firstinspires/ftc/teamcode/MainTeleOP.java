package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.teamcode.hardware.customHardware.ToggleButton;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.MisumiSliderSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.MovementSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.SmartSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.TransferSubsystem;

import java.lang.reflect.Field;
import java.util.List;
import java.util.Objects;

@TeleOp
public class MainTeleOP extends LinearOpMode {

    GamepadEx operatorGamepad, driverGamepad;

    private SliderSubsystem sliderSubsystem = new SliderSubsystem(operatorGamepad);
    private IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private TransferSubsystem transferSubsystem = new TransferSubsystem();
    private MovementSubsystem movementSubsystem = new MovementSubsystem();
    private MisumiSliderSubsystem sliderSubsystem = new MisumiSliderSubsystem();
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry= new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        try{
            GamepadEx driverGamepad = new GamepadEx(gamepad1);
            GamepadEx operatorGamepad = new GamepadEx(gamepad2);

            // Use Java reflection to access all fields of this TeleOP which are SmartSubsystems
            // and run initSubsystem on them
            for(Field field: this.getClass().getDeclaredFields())
            {
                if(field.getType().getSuperclass().equals(SmartSubsystem.class))
                {

                    SmartSubsystem subsystem =  ((SmartSubsystem) Objects.requireNonNull(field.get(this)));
                    try {
                       subsystem.initSubsystem((LinearOpMode) this, hardwareMap);
                    } catch (Exception e) {
                        subsystem.initialized=false;
                        telemetry.addLine(String.format("Failed initializing %s", field.getName()));
                        telemetry.addLine(e.toString());
                    }
                }

            }
            List<LynxModule> expansionHubs = hardwareMap.getAll(LynxModule.class);
            ToggleButtonReader intakeButton = new ToggleButtonReader(driverGamepad, GamepadKeys.Button.A);
            ToggleButtonReader clampButton = new ToggleButtonReader(driverGamepad, GamepadKeys.Button.LEFT_BUMPER);
            ToggleButton armButton = new ToggleButton(driverGamepad, GamepadKeys.Button.B);
            ButtonReader legReader = new ButtonReader(driverGamepad, GamepadKeys.Button.Y);
            telemetry.update();
            waitForStart();
            while(opModeIsActive()&&!isStopRequested())
            {
                driverGamepad.readButtons();
                operatorGamepad.readButtons();

                if(intakeSubsystem.initialized) intakeSubsystem.run(intakeButton);
                if(transferSubsystem.initialized) transferSubsystem.run(armButton, legReader);
                if(sliderSubsystem.initialized) sliderSubsystem.run();
//                if(movementSubsystem.initialized) movementSubsystem.run(driverGamepad);
                for(LynxModule hub: expansionHubs)
                {

                    telemetry.addData(hub.getDeviceName()+" voltage", hub.getInputVoltage(VoltageUnit.VOLTS));
                    telemetry.addData(hub.getDeviceName()+" current", hub.getCurrent(CurrentUnit.AMPS));
                }
                telemetry.update();
            }
        }catch(Exception e)
        {
            telemetry.addLine(e.toString());
            telemetry.update();
        }
    }
}
