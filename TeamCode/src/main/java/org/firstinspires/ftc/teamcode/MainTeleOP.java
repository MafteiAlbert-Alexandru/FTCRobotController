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

import org.firstinspires.ftc.teamcode.fsm.MovementTransition;
import org.firstinspires.ftc.teamcode.fsm.albert.ButtonTransition;
import org.firstinspires.ftc.teamcode.fsm.albert.FSM;
import org.firstinspires.ftc.teamcode.fsm.albert.State;
import org.firstinspires.ftc.teamcode.junctionCalibration.JunctionAdjuster;
import org.firstinspires.ftc.teamcode.subsystem.ClampSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.MovementSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.SliderSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.SmartSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.SubsystemData;
import org.firstinspires.ftc.teamcode.subsystem.TransferSubsystem;
import org.firstinspires.ftc.teamcode.vision.WebcamUtil;

import java.lang.reflect.Field;
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
    public static double visionAngle = 45;
    public static double visionSpeed = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry= new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        try{


            // Use Java reflection to access all fields of this TeleOP which are SmartSubsystems
            // and run initSubsystem on them
            for(Field field: this.getClass().getDeclaredFields())
            {
                if(field.getType().getSuperclass()!=null && field.getType().getSuperclass().equals(SmartSubsystem.class))
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

            WebcamUtil webcamUtil = new WebcamUtil(hardwareMap);

            JunctionAdjuster junctionAdjuster = new JunctionAdjuster(webcamUtil, 2.54, telemetry);
            webcamUtil.registerListener(junctionAdjuster);
            webcamUtil.start(true);
            telemetry.update();
            waitForStart();

            SubsystemData data = new SubsystemData();
            data.driverGamepad= new GamepadEx(gamepad1);
            data.operatorGamepad = new GamepadEx(gamepad2);

            FSM movementFSM = new FSM();
            State movementState = new State() {
                public void update()
                {
                    movementSubsystem.run(data);
                }
            };
            State homingState = new State() {
                public void update()
                {
                    Vector2d direction = junctionAdjuster.value(visionSpeed, new JunctionAdjuster.Vec2(-10.2,4.4));
                    movementSubsystem.move(direction.getY(), direction.getX(),0);
                }
            };
            movementFSM.add(new ButtonTransition(movementState,homingState,data.driverGamepad, GamepadKeys.Button.A) {});
            movementFSM.add(new MovementTransition(homingState, movementState, data.driverGamepad){});
            movementFSM.setInitialState(movementState);
            movementFSM.add(homingState);
            movementFSM.build();

            FSM fsm = new FSM();
            State upperSliderState = new State(fsm) {};
            State mediumSliderState = new State(fsm) {};
            State lowerSliderState = new State(fsm) {};
            State groundSliderState = new State(fsm) {};
            State waitingSliderState = new State(fsm) {};

            State loadedSliderState = new State(fsm) {};
            State initialSliderState = new State() {};
            fsm.setInitialState(initialSliderState);
            ExecutorService executor = Executors.newFixedThreadPool(4);
            fsm.add(new ButtonTransition(initialSliderState, waitingSliderState, data.operatorGamepad, GamepadKeys.Button.Y)
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
            fsm.add(new ButtonTransition(waitingSliderState, loadedSliderState, data.operatorGamepad, GamepadKeys.Button.Y)
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

            fsm.add(new ButtonTransition(loadedSliderState, groundSliderState, data.operatorGamepad, GamepadKeys.Button.DPAD_DOWN)
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
                        sliderSubsystem.goToPosition(SliderSubsystem.GroundPos);
                    });
                }
            });
            fsm.add(new ButtonTransition(loadedSliderState, lowerSliderState, data.operatorGamepad, GamepadKeys.Button.DPAD_LEFT)
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
            fsm.add(new ButtonTransition(loadedSliderState, mediumSliderState, data.operatorGamepad, GamepadKeys.Button.DPAD_RIGHT)
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
            fsm.add(new ButtonTransition(loadedSliderState, upperSliderState, data.operatorGamepad, GamepadKeys.Button.DPAD_UP)
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

            fsm.addTransitionsTo(waitingSliderState, new State[]{mediumSliderState,groundSliderState,lowerSliderState,upperSliderState}, new ButtonTransition(data.operatorGamepad, GamepadKeys.Button.Y)
            {
                @Override
                public boolean check() {
                    return super.check() && !clampSubsystem.isClamping();
                }
                @Override
                public void run(){

                    sliderSubsystem.goToClear();
                    executor.execute(()->{
                        while(!sliderSubsystem.isClear());
                        clampSubsystem.goToBackward();

                    });
                }
            });
            fsm.addTransitionsTo(upperSliderState, new State[]{mediumSliderState,groundSliderState,lowerSliderState}, new ButtonTransition(data.operatorGamepad, GamepadKeys.Button.DPAD_UP) {
                @Override
                public void run() {
                    sliderSubsystem.goToPosition(SliderSubsystem.highPos);
                }
            });
            fsm.addTransitionsTo(mediumSliderState, new State[]{upperSliderState,groundSliderState,lowerSliderState}, new ButtonTransition(data.operatorGamepad, GamepadKeys.Button.DPAD_RIGHT) {
                @Override
                public void run() {
                    sliderSubsystem.goToPosition(SliderSubsystem.midPos);
                }
            });
            fsm.addTransitionsTo(lowerSliderState, new State[]{upperSliderState,groundSliderState,mediumSliderState}, new ButtonTransition(data.operatorGamepad, GamepadKeys.Button.DPAD_LEFT) {
                @Override
                public void run() {
                    sliderSubsystem.goToPosition(SliderSubsystem.lowPos);
                }
            });
            fsm.addTransitionsTo(groundSliderState, new State[]{upperSliderState,mediumSliderState,lowerSliderState}, new ButtonTransition(data.operatorGamepad, GamepadKeys.Button.DPAD_DOWN) {
                @Override
                public void run() {
                    sliderSubsystem.goToPosition(SliderSubsystem.GroundPos);
                }
            });
            fsm.build();
            waitForStart();
            webcamUtil.setAngle(Math.toRadians(visionAngle));
            while(opModeIsActive()&&!isStopRequested()) {
                data.driverGamepad.readButtons();
                data.operatorGamepad.readButtons();
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
            webcamUtil.stop();
            } catch (IllegalAccessException illegalAccessException) {
            illegalAccessException.printStackTrace();

    }catch(Exception e)
        {
            telemetry.addLine(e.toString());
            telemetry.update();
        }
    }
}
