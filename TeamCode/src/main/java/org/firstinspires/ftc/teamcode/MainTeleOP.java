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
import org.firstinspires.ftc.teamcode.subsystem.ClampSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.ClampSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.MovementSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.SliderSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.SmartSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.SubsystemData;
import org.firstinspires.ftc.teamcode.subsystem.TransferSubsystem;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.concurrent.Executor;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.ThreadPoolExecutor;

@TeleOp
public class MainTeleOP extends LinearOpMode {

    private IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private TransferSubsystem transferSubsystem = new TransferSubsystem();
    private MovementSubsystem movementSubsystem = new MovementSubsystem();
    private SliderSubsystem sliderSubsystem = new SliderSubsystem();
    private ClampSubsystem clampSubsystem = new ClampSubsystem();
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry= new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        try{

            List<SmartSubsystem> smartSubsystems = new ArrayList<SmartSubsystem>();
            // Use Java reflection to access all fields of this TeleOP which are SmartSubsystems
            // and run initSubsystem on them
            for(Field field: this.getClass().getDeclaredFields())
            {
                if(field.getType().getSuperclass().equals(SmartSubsystem.class))
                {

                    SmartSubsystem subsystem =  ((SmartSubsystem) Objects.requireNonNull(field.get(this)));
                    try {
                       subsystem.initSubsystem((LinearOpMode) this, hardwareMap);
                       smartSubsystems.add(subsystem);
                    } catch (Exception e) {
                        telemetry.addLine(String.format("Failed initializing %s", field.getName()));
                        telemetry.addLine(e.toString());
                    }
                }
            }

            List<LynxModule> expansionHubs = hardwareMap.getAll(LynxModule.class);
            for(LynxModule expansionHub: expansionHubs)
            {
                expansionHub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
            }

            telemetry.update();
            //ExecutorService executor = Executors.newFixedThreadPool(smartSubsystems.size());
            waitForStart();

            SubsystemData data = new SubsystemData();
            data.driverGamepad= new GamepadEx(gamepad1);
            data.operatorGamepad = new GamepadEx(gamepad2);

            for(SmartSubsystem subsystem : smartSubsystems)
            {
                subsystem.run(data);
            }
            while(opModeIsActive()&&!isStopRequested())
            {
                data.driverGamepad.readButtons();
                data.operatorGamepad.readButtons();
                for(SmartSubsystem subsystem : smartSubsystems)
                {
                    subsystem.run(data);
                }
                // Adauga orice citire paralela aici
                telemetry.update();
            }
        }catch(Exception e)
        {
            telemetry.addLine(e.toString());
            telemetry.update();
        }
    }
}
