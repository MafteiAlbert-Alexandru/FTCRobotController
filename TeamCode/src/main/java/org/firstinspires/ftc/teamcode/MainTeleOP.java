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
import org.firstinspires.ftc.teamcode.fsm.ButtonTransition;
import org.firstinspires.ftc.teamcode.fsm.NullState;
import org.firstinspires.ftc.teamcode.fsm.SmartFSM;
import org.firstinspires.ftc.teamcode.fsm.SmartState;
import org.firstinspires.ftc.teamcode.fsm.SmartTransition;
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
    private boolean stupidState = false;



    @Override
    public void runOpMode() throws InterruptedException {
        telemetry= new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        try{

            List<SmartSubsystem> smartSubsystems = new ArrayList<SmartSubsystem>();
            // Use Java reflection to access all fields of this TeleOP which are SmartSubsystems
            // and run initSubsystem on them
            for(Field field: this.getClass().getDeclaredFields())
            {
                if(field.getType().getSuperclass()!=null && field.getType().getSuperclass().equals(SmartSubsystem.class))
                {

                    SmartSubsystem subsystem =  ((SmartSubsystem) Objects.requireNonNull(field.get(this)));
                    try {
                       subsystem.initSubsystem((LinearOpMode) this, hardwareMap);
                       smartSubsystems.add(subsystem);
                    } catch (Exception e) {
                        subsystem.initialized=false;
                        telemetry.addLine(String.format("Failed initializing %s", field.getName()));
                        telemetry.addLine(e.toString());
                    }
                }
            }

            List<LynxModule> expansionHubs = hardwareMap.getAll(LynxModule.class);


            telemetry.update();
            waitForStart();

            SubsystemData data = new SubsystemData();
            data.driverGamepad= new GamepadEx(gamepad1);
            data.operatorGamepad = new GamepadEx(gamepad2);

            SmartFSM fsm = new SmartFSM();
            SmartState upperSliderState = new NullState(fsm) {};
            SmartState mediumSliderState = new NullState(fsm) {};
            SmartState lowerSliderState = new NullState(fsm) {};
            SmartState groundSliderState = new NullState(fsm) {};
            SmartState waitingSliderState = new NullState(fsm) {};
            SmartState[] safeStates = {upperSliderState, mediumSliderState, lowerSliderState, groundSliderState, waitingSliderState};

            SmartState loadedSliderState = new NullState(fsm) {};
            SmartState initialSliderState = new NullState() {};
            fsm.setInitialState(initialSliderState);
            ExecutorService executor = Executors.newFixedThreadPool(4);
            fsm.addTransition(new ButtonTransition(initialSliderState, waitingSliderState, data.operatorGamepad, GamepadKeys.Button.Y)
            {
                @Override
                public void run(){
                    sliderSubsystem.goToClear();
                    executor.execute(()->{
                        while(!sliderSubsystem.isClear());
                        clampSubsystem.release();
                        clampSubsystem.goToBackward();

                    });
                }
            });
            fsm.addTransition(new ButtonTransition(waitingSliderState, loadedSliderState, data.operatorGamepad, GamepadKeys.Button.Y)
            {
                @Override
                public void run(){
                    executor.execute(()->{
                        clampSubsystem.release();
                        sliderSubsystem.goToTake();
                        try {
                            Thread.sleep(900);
                        } catch (InterruptedException ex) {
                            ex.printStackTrace();
                        }
                        clampSubsystem.clamp();
                    });
                }
            });

            fsm.addTransition(new ButtonTransition(loadedSliderState, groundSliderState, data.operatorGamepad, GamepadKeys.Button.DPAD_DOWN)
            {
                @Override
                public void run()
                {
                    sliderSubsystem.goToClear();

                    executor.execute(()->{
                        while(!sliderSubsystem.isClear());
                        clampSubsystem.goToForward();
                        try {
                            Thread.sleep(500);
                        } catch (InterruptedException ex) {
                            ex.printStackTrace();
                        }
                        sliderSubsystem.goToPosition(SliderSubsystem.groundPos);
                    });
                }
            });
            fsm.addTransition(new ButtonTransition(loadedSliderState, lowerSliderState, data.operatorGamepad, GamepadKeys.Button.DPAD_LEFT)
            {
                @Override
                public void run()
                {
                    sliderSubsystem.goToClear();

                    executor.execute(()->{
                        while(!sliderSubsystem.isClear());
                        clampSubsystem.goToForward();
                        try {
                            Thread.sleep(500);
                        } catch (InterruptedException ex) {
                            ex.printStackTrace();
                        }
                        sliderSubsystem.goToPosition(SliderSubsystem.lowPos);
                    });
                }
            });
            fsm.addTransition(new ButtonTransition(loadedSliderState, mediumSliderState, data.operatorGamepad, GamepadKeys.Button.DPAD_RIGHT)
            {
                @Override
                public void run()
                {
                    sliderSubsystem.goToClear();

                    executor.execute(()->{
                        while(!sliderSubsystem.isClear());
                        clampSubsystem.goToForward();
                        try {
                            Thread.sleep(500);
                        } catch (InterruptedException ex) {
                            ex.printStackTrace();
                        }
                        sliderSubsystem.goToPosition(SliderSubsystem.midPos);
                    });
                }
            });
            fsm.addTransition(new ButtonTransition(loadedSliderState, upperSliderState, data.operatorGamepad, GamepadKeys.Button.DPAD_UP)
            {
                @Override
                public void run()
                {
                    sliderSubsystem.goToClear();

                    executor.execute(()->{
                        while(!sliderSubsystem.isClear());
                        clampSubsystem.goToForward();
                        try {
                            Thread.sleep(500);
                        } catch (InterruptedException ex) {
                            ex.printStackTrace();
                        }
                        sliderSubsystem.goToPosition(SliderSubsystem.highPos);
                    });
                }
            });

            fsm.addTransition(new ButtonTransition(groundSliderState, waitingSliderState, data.operatorGamepad, GamepadKeys.Button.Y)
            {
                @Override
                public void run() {
                    sliderSubsystem.goToClear();
                    executor.execute(()->{
                        while(!sliderSubsystem.isClear());
                        clampSubsystem.goToBackward();

                    });
                }
            });
            fsm.addTransition(new ButtonTransition(lowerSliderState, waitingSliderState, data.operatorGamepad, GamepadKeys.Button.Y)
            {
                @Override
                public void run() {
                    sliderSubsystem.goToClear();
                    executor.execute(()->{
                        while(!sliderSubsystem.isClear());
                        clampSubsystem.goToBackward();

                    });
                }
            });
            fsm.addTransition(new ButtonTransition(mediumSliderState, waitingSliderState, data.operatorGamepad, GamepadKeys.Button.Y)
            {
                @Override
                public void run() {
                    sliderSubsystem.goToClear();
                    executor.execute(()->{
                        while(!sliderSubsystem.isClear());
                        clampSubsystem.goToBackward();

                    });
                }
            });
            fsm.addTransition(new ButtonTransition(upperSliderState, waitingSliderState, data.operatorGamepad, GamepadKeys.Button.Y)
            {
                @Override
                public void run() {
                    sliderSubsystem.goToClear();
                    executor.execute(()->{
                        while(!sliderSubsystem.isClear());
                        clampSubsystem.goToBackward();

                    });
                }
            });

            fsm.build();
            waitForStart();
            while(opModeIsActive()&&!isStopRequested()) {
                data.driverGamepad.readButtons();
                data.operatorGamepad.readButtons();

                fsm.update();


                if (movementSubsystem.initialized) {
                    movementSubsystem.run(data);
                }
                if (intakeSubsystem.initialized) {
                    if (data.operatorGamepad.getButton(GamepadKeys.Button.LEFT_BUMPER)) {
                        intakeSubsystem.intake();
                    } else if (data.operatorGamepad.getButton(GamepadKeys.Button.RIGHT_BUMPER)) {
                        intakeSubsystem.expulse();
                    } else intakeSubsystem.stop();
                }


//                if(sliderSubsystem.initialized && clampSubsystem.initialized)
////                {
//
//                    if(data.operatorGamepad.wasJustPressed(GamepadKeys.Button.Y))
//                    {
//                        if(!stupidState)
//                        {
//
//                        }else if(stupidState)
//                        {
//
//                        }
//                        sliderSubsystem.goToClear();
//                        executor.execute(()->{
//                            while(!sliderSubsystem.isClear());
//                            clampSubsystem.release();
//                            clampSubsystem.goToBackward();
//
//                            try {
//                                Thread.sleep(900);
//                            } catch (InterruptedException e) {
//                                e.printStackTrace();
//                            }
//                            sliderSubsystem.goToTake();
//                        });
//
//                    }else if(data.operatorGamepad.wasJustPressed(GamepadKeys.Button.DPAD_UP)){
//                        if(!sliderSubsystem.isClear())
//                            sliderSubsystem.goToClear();
//
//                        executor.execute(()->{
//                            while(!sliderSubsystem.isClear());
//                            sliderSubsystem.goToPosition(SliderSubsystem.highPos);
//                            clampSubsystem.goToForward();
//
//                            });
//
//
//                    }else if(data.operatorGamepad.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
//                        if (!sliderSubsystem.isClear())
//                            sliderSubsystem.goToClear();
//
//                        executor.execute(() -> {
//                            while (!sliderSubsystem.isClear()) ;
//                            sliderSubsystem.goToPosition(SliderSubsystem.midPos);
//                            clampSubsystem.goToForward();
//
//                        });
//                    }else if(data.operatorGamepad.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
//                        if(!sliderSubsystem.isClear())
//                            sliderSubsystem.goToClear();
//
//                        executor.execute(()->{
//                            while(!sliderSubsystem.isClear());
//                            sliderSubsystem.goToPosition(SliderSubsystem.lowPos);
//                            clampSubsystem.goToForward();
//
//                        });
//                    }else if(data.operatorGamepad.wasJustPressed(GamepadKeys.Button.DPAD_DOWN))
//                    {
//                        if(!sliderSubsystem.isClear())
//                            sliderSubsystem.goToClear();
//
//                        executor.execute(()->{
//                            while(!sliderSubsystem.isClear());
//                            sliderSubsystem.goToPosition(SliderSubsystem.groundPos);
//                            clampSubsystem.goToForward();
//
//                        });
//                    }
                sliderSubsystem.run(data);
                clampSubsystem.run(data);
//                    sliderSubsystem.move((float) (data.operatorGamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)-data.operatorGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)));
//                }

                if (transferSubsystem.initialized)
                    transferSubsystem.run(data);

                // Adauga orice citire paralela aici*/
                telemetry.update();
            }
            } catch (IllegalAccessException illegalAccessException) {
            illegalAccessException.printStackTrace();

    }catch(Exception e)
        {
            telemetry.addLine(e.toString());
            telemetry.update();
        }
    }
}
