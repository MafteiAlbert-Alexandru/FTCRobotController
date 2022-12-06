package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.fsm.ButtonTransition;
import org.firstinspires.ftc.teamcode.fsm.MovementTransition;
import org.firstinspires.ftc.teamcode.fsm.NullState;
import org.firstinspires.ftc.teamcode.fsm.SmartFSM;
import org.firstinspires.ftc.teamcode.fsm.SmartState;
import org.firstinspires.ftc.teamcode.junctionCalibration.PixelJunctionAdjuster;
import org.firstinspires.ftc.teamcode.subsystem.ClampSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.MovementSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.SliderSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.SmartSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.SubsystemData;
import org.firstinspires.ftc.teamcode.subsystem.TransferSubsystem;
import org.firstinspires.ftc.teamcode.vision.WebcamUtil;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
@Config
@TeleOp
public class MainTeleOP extends LinearOpMode {

    private IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private TransferSubsystem transferSubsystem = new TransferSubsystem();
    private MovementSubsystem movementSubsystem = new MovementSubsystem();
    private SliderSubsystem sliderSubsystem = new SliderSubsystem();
    private ClampSubsystem clampSubsystem = new ClampSubsystem();
    private boolean stupidState = false;
    public static double angle =45;
    public static double speed=0;

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

            WebcamUtil webcamUtil = new WebcamUtil(hardwareMap);

            PixelJunctionAdjuster junctionAdjuster = new PixelJunctionAdjuster(webcamUtil,2.54, telemetry);

            webcamUtil.start(true);
            telemetry.update();
            waitForStart();

            SubsystemData data = new SubsystemData();
            data.driverGamepad= new GamepadEx(gamepad1);
            data.operatorGamepad = new GamepadEx(gamepad2);

            SmartFSM movementFSM = new SmartFSM();
            SmartState movementState = new SmartState() {
                public void update()
                {
                    telemetry.addData("aaa",1);
                    movementSubsystem.run(data);
                }
            };
            SmartState homingState = new SmartState() {
                public void update()
                {
                    telemetry.addData("aaa",2);
                    Vector2d direction = junctionAdjuster.value(speed, new PixelJunctionAdjuster.Vec2(-2.3,6.5), Math.toDegrees(webcamUtil.getAngle()));
                    movementSubsystem.move(direction.getY(), direction.getX(),0);
                }
            };
            movementFSM.addTransition(new ButtonTransition(movementState,homingState,data.driverGamepad, GamepadKeys.Button.A) {});
            movementFSM.addTransition(new MovementTransition(homingState, movementState, data.driverGamepad) {});
            movementFSM.setInitialState(movementState);
            movementFSM.addState(homingState);
            movementFSM.build();

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
                            Thread.sleep(400);
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
                        sliderSubsystem.goToPosition(SliderSubsystem.midPos);
                        while(!sliderSubsystem.isClear());
                        clampSubsystem.goToForward();


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
                        sliderSubsystem.goToPosition(SliderSubsystem.highPos);
                        clampSubsystem.goToForward();


                    });
                }
            });

            fsm.addTransitionsTo(waitingSliderState, new SmartState[]{mediumSliderState,groundSliderState,lowerSliderState,upperSliderState}, new ButtonTransition(data.operatorGamepad, GamepadKeys.Button.Y)
            {
                @Override
                public void run(){
                    clampSubsystem.goToBackward();
                    sliderSubsystem.goToClear();
                    executor.execute(()->{
                        while(!sliderSubsystem.isClear());

                    });
                }
            });
            fsm.addTransitionsTo(upperSliderState, new SmartState[]{mediumSliderState,groundSliderState,lowerSliderState}, new ButtonTransition(data.operatorGamepad, GamepadKeys.Button.DPAD_UP) {
                @Override
                public void run() {
                    sliderSubsystem.goToPosition(SliderSubsystem.highPos);
                }
            });
            fsm.addTransitionsTo(mediumSliderState, new SmartState[]{upperSliderState,groundSliderState,lowerSliderState}, new ButtonTransition(data.operatorGamepad, GamepadKeys.Button.DPAD_RIGHT) {
                @Override
                public void run() {
                    sliderSubsystem.goToPosition(SliderSubsystem.midPos);
                }
            });
            fsm.addTransitionsTo(lowerSliderState, new SmartState[]{upperSliderState,groundSliderState,mediumSliderState}, new ButtonTransition(data.operatorGamepad, GamepadKeys.Button.DPAD_LEFT) {
                @Override
                public void run() {
                    sliderSubsystem.goToPosition(SliderSubsystem.lowPos);
                }
            });
            fsm.addTransitionsTo(groundSliderState, new SmartState[]{upperSliderState,mediumSliderState,lowerSliderState}, new ButtonTransition(data.operatorGamepad, GamepadKeys.Button.DPAD_DOWN) {
                @Override
                public void run() {
                    sliderSubsystem.goToPosition(SliderSubsystem.groundPos);
                }
            });
            fsm.build();
            waitForStart();
            while(opModeIsActive()&&!isStopRequested()) {
                data.driverGamepad.readButtons();
                data.operatorGamepad.readButtons();
                webcamUtil.setAngle(Math.toRadians(angle));
                fsm.update();
                movementFSM.update();



                if (intakeSubsystem.initialized) {
                    if (data.operatorGamepad.getButton(GamepadKeys.Button.LEFT_BUMPER)) {
                        intakeSubsystem.intake();
                    } else if (data.operatorGamepad.getButton(GamepadKeys.Button.RIGHT_BUMPER)) {
                        intakeSubsystem.expulse();
                    } else intakeSubsystem.stop();
                }

                sliderSubsystem.run(data);
                clampSubsystem.run(data);
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
