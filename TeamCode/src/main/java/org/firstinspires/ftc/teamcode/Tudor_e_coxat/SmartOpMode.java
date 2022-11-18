package org.firstinspires.ftc.teamcode.Tudor_e_coxat;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.teamcode.subsystem.SmartSubsystem;

import java.lang.reflect.Field;
import java.util.List;
import java.util.Objects;

public class SmartOpMode extends LinearOpMode {

    GamepadEx driverGamepad, operatorGamepad;

    public void OpInit(){}
    public void OpLoop(){}

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry= new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        try{
            driverGamepad = new GamepadEx(gamepad1);
            operatorGamepad = new GamepadEx(gamepad2);
            for(Field field: this.getClass().getDeclaredFields())
            {
                if(field.getType().getSuperclass().equals(SmartSubsystem.class))
                {
                    SmartSubsystem subsystem =  ((SmartSubsystem) Objects.requireNonNull(field.get(this)));
                    try { subsystem.initSubsystem((LinearOpMode) this, hardwareMap); }
                    catch (Exception e) {
                        telemetry.addLine(String.format("Failed initializing %s", field.getName()));
                        telemetry.addLine(e.toString());
                    }
                }

            }
            List<LynxModule> expansionHubs = hardwareMap.getAll(LynxModule.class);
            telemetry.update();

            OpInit();

            waitForStart();
            while(opModeIsActive()&&!isStopRequested())
            {
                OpLoop();
                driverGamepad.readButtons();
                operatorGamepad.readButtons();

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
