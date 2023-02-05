package org.firstinspires.ftc.teamcode.robot;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.junction.JunctionAdjuster;
import org.firstinspires.ftc.teamcode.robot.fsm.ButtonTransition;
import org.firstinspires.ftc.teamcode.robot.fsm.FSM;
import org.firstinspires.ftc.teamcode.robot.fsm.MovementTransition;
import org.firstinspires.ftc.teamcode.robot.fsm.ReleaseTransition;
import org.firstinspires.ftc.teamcode.robot.fsm.State;
import org.firstinspires.ftc.teamcode.robot.fsm.Transition;
import org.firstinspires.ftc.teamcode.subsystem.ClampSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.MovementSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.SensorSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.SliderSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.SmartSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.SubsystemData;
import org.firstinspires.ftc.teamcode.subsystem.TransferSubsystem;
import org.firstinspires.ftc.teamcode.vision.WebcamUtil;

import java.util.List;

public class Robot {

    public enum OpModeType{
        Auto,
        TeleOp
    }

    public OpModeType opModeType;//love you tudor<3

    //region Subsystem
    public  IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    public  TransferSubsystem transferSubsystem = new TransferSubsystem();
    public  MovementSubsystem movementSubsystem = new MovementSubsystem();
    public SliderSubsystem sliderV2Subsystem = new SliderSubsystem();
    public ClampSubsystem clampSubsystem = new ClampSubsystem();
    public   SensorSubsystem sensorSubsystem = new SensorSubsystem();

    GamepadKeys.Button highButton = GamepadKeys.Button.DPAD_UP;
    GamepadKeys.Button midButton = GamepadKeys.Button.DPAD_RIGHT;
    GamepadKeys.Button lowButton = GamepadKeys.Button.DPAD_DOWN;
    GamepadKeys.Button groundButton;

    //endregion

    public State initialState;
    public State waitingState;
    public State loadedState;
    public State upperState;
    public State mediumState;
    public State lowerState;
    public State groundState;
    public State frontWaitingState;

    public State movingState;
    public State homingState;

    public State inactiveState;
    public State liftingState;
    public State loweredState;
    public State expulseState;

    public final FSM SliderAndClampingFSM = new FSM();
    public final FSM MovementFSM = new FSM();
    public final FSM IntakeFSM = new FSM();

    private GamepadEx driverGamepad;
    private GamepadEx operatorGamepad;

    public Robot(OpMode opMode, OpModeType opModeType, boolean camera) throws IllegalAccessException {
        setup(opMode, opModeType, camera);
    }
    public Robot(OpMode opMode, OpModeType opModeType) throws IllegalAccessException {
     setup(opMode, opModeType, false);
    }
    public void setup(OpMode opMode, OpModeType opModeType, boolean camera) throws IllegalAccessException {
        this.opModeType = opModeType;
        SmartSubsystem.initAllSubsystems(this, opMode);
        //initializeaza toate subsystemele

        //defintesc state urile
        initialState = new State(SliderAndClampingFSM, "initialState") {
        };
        waitingState = new State(SliderAndClampingFSM, "waitingState") {
        };
        loadedState = new State(SliderAndClampingFSM, "loadedState") {
        };
        upperState = new State(SliderAndClampingFSM, "upperState") {
        };
        mediumState = new State(SliderAndClampingFSM, "mediumState") {
        };
        lowerState = new State(SliderAndClampingFSM, "lowerState") {
        };
        groundState = new State(SliderAndClampingFSM, "groundState") {
        };
        frontWaitingState = new State(SliderAndClampingFSM, "frontWaitingState") {
        };
        inactiveState = new State(IntakeFSM, "inactiveState") {};
        loweredState = new State(IntakeFSM, "loweredState") {};
        liftingState = new State(IntakeFSM, "liftingState") {};
        expulseState = new State(IntakeFSM, "expulseState") {};
        //initializez gamepad-urile
        driverGamepad = new GamepadEx(opMode.gamepad1);
        operatorGamepad = new GamepadEx(opMode.gamepad2);

        //imi setez un state initial
        SliderAndClampingFSM.setInitialState(initialState);

        SliderAndClampingFSM.add(new Transition(initialState, waitingState) {

            @Override
            public boolean check(){
                return true;
            }

            //adaug un state intre cel initial si cel de waiting
            //isi ia trigger de la butonul Y (ala de sus de pe partea dreapta pt cei care nu le stiu dupa litere/daca esti hater la alea logitech)

            @Override
            public boolean run() throws InterruptedException {
                clampSubsystem.goTo(ClampSubsystem.ForwardPos);
                sliderV2Subsystem.goTo(SliderSubsystem.LowPos);
                clampSubsystem.goTo(ClampSubsystem.BackwardPos);
                sliderV2Subsystem.goTo(SliderSubsystem.PreLoadPos,500, 20);
                clampSubsystem.release();
                //hook-ul se duce in SafePos si pe spate (adica in robot)
                return true;
            }
        });

        SliderAndClampingFSM.add(new ButtonTransition(loadedState, waitingState, operatorGamepad, GamepadKeys.Button.Y) {
            //adaug un state intre cel initial si cel de waiting
            //isi ia trigger de la butonul Y (ala de sus de pe partea dreapta pt cei care nu le stiu dupa litere/daca esti hater la alea logitech)

            @Override
            public boolean run() throws InterruptedException {
                clampSubsystem.goTo(ClampSubsystem.BackwardPos);
                clampSubsystem.release();
                sliderV2Subsystem.goTo(SliderSubsystem.PreLoadPos,500, 20);
                //hook-ul se duce in SafePos si pe spate (adica in robot)
                return true;
            }
        });

        SliderAndClampingFSM.add(new Transition(waitingState, loadedState) {
            //Tranzitie care isi poate lua trigger ori de la buton ori de la alti factori (ex senzori)

            @Override
            //Asta verifica daca isi ia trigger tranzitia
            public boolean check() {
                return sensorSubsystem.coneIsLoaded() || subsystemData.operatorGamepad.wasJustPressed(GamepadKeys.Button.Y);
            }

            @Override
            public boolean run() throws InterruptedException {
                Thread.sleep(600);
                if(!sensorSubsystem.coneIsLoaded())
                    return false;
                //daca e sus mergi jos (vezi in codul sursa)
                if(transferSubsystem.isUp()) transferSubsystem.bControl =false;
                clampSubsystem.goTo(ClampSubsystem.BackwardPos);
                clampSubsystem.release(); //dau drumul la hook (siguranta)
                sliderV2Subsystem.goTo(SliderSubsystem.LoadPos, 750); //duc slider-ul in con (o bag tare)
                clampSubsystem.clamp();//imi deschid carligul/prind conul
                transferSubsystem.bControl =true;
                Thread.sleep(250);//FIX to ensure the operator does not lift before it is clamped
                return true;
            }
        });

        //restul se inteleg daca ai citit si codul sursa de la subsysteme
//        SliderAndClampingFSM.add(new ButtonTransition(waitingState, frontWaitingState, operatorGamepad, GamepadKeys.Button.LEFT_STICK_BUTTON) {
//            @Override
//            public boolean run() throws InterruptedException {
//                sliderV2Subsystem.goTo(SliderSubsystem.LowPos, 750);
//                clampSubsystem.goTo(ClampSubsystem.ForwardPos);
//                return true;
//            }
//        });
//        SliderAndClampingFSM.add(new ButtonTransition(frontWaitingState, upperState, operatorGamepad, highButton){
//            @Override
//            public boolean run() throws InterruptedException {
//                sliderV2Subsystem.goTo(SliderSubsystem.LowPos, 750);
//                clampSubsystem.goTo(ClampSubsystem.ForwardPos);
//                return true;
//            }
//        });
        SliderAndClampingFSM.add(new ButtonTransition(loadedState, upperState, operatorGamepad, highButton) {
            @Override
            public boolean run() throws InterruptedException {
                sliderV2Subsystem.goTo(SliderSubsystem.HighPos,750);
                clampSubsystem.goTo(ClampSubsystem.ForwardPos);
                return true;
            }
        });
        SliderAndClampingFSM.add(new ButtonTransition(loadedState, mediumState, operatorGamepad, midButton) {
            @Override
            public boolean run() throws InterruptedException {
                sliderV2Subsystem.goTo(SliderSubsystem.MediumPos);
                clampSubsystem.goTo(ClampSubsystem.ForwardPos);
                return true;
            }
        });
        SliderAndClampingFSM.add(new ButtonTransition(loadedState, lowerState, operatorGamepad, lowButton) {
            @Override
            public boolean run() throws InterruptedException {
                sliderV2Subsystem.goTo(SliderSubsystem.LowPos);
                clampSubsystem.goTo(ClampSubsystem.ForwardPos);
                return true;
            }
        });
        SliderAndClampingFSM.add(new ButtonTransition(loadedState, groundState, operatorGamepad, groundButton) {
            @Override
            public boolean run() throws InterruptedException {
                sliderV2Subsystem.goTo(SliderSubsystem.SafePos);
                clampSubsystem.goTo(ClampSubsystem.ForwardPos);
                sliderV2Subsystem.goTo(SliderSubsystem.GroundPos);
                return true;
            }
        });
        SliderAndClampingFSM.add(new ButtonTransition(upperState, mediumState, operatorGamepad, midButton) {
            @Override
            public boolean run() throws InterruptedException {
                clampSubsystem.goTo(ClampSubsystem.BackwardPos);
                sliderV2Subsystem.goTo(SliderSubsystem.MediumPos);
                clampSubsystem.goTo(ClampSubsystem.ForwardPos);
                return true;
            }
        });
        SliderAndClampingFSM.add(new ButtonTransition(upperState, lowerState, operatorGamepad, lowButton) {
            @Override
            public boolean run() throws InterruptedException {
                clampSubsystem.goTo(ClampSubsystem.BackwardPos);
                sliderV2Subsystem.goTo(SliderSubsystem.LowPos);
                clampSubsystem.goTo(ClampSubsystem.ForwardPos);
                return true;
            }
        });
        SliderAndClampingFSM.add(new ButtonTransition(upperState, groundState, operatorGamepad, groundButton) {
            @Override
            public boolean run() throws InterruptedException {
                clampSubsystem.goTo(ClampSubsystem.BackwardPos);
                sliderV2Subsystem.goTo(SliderSubsystem.SafePos);
                clampSubsystem.goTo(ClampSubsystem.ForwardPos);
                sliderV2Subsystem.goTo(SliderSubsystem.GroundPos);
                return true;
            }
        });
        SliderAndClampingFSM.add(new ButtonTransition(mediumState, upperState, operatorGamepad, highButton) {
            @Override
            public boolean run() throws InterruptedException {
                sliderV2Subsystem.goTo(SliderSubsystem.HighPos);
                return true;
            }
        });
        SliderAndClampingFSM.add(new ButtonTransition(mediumState, lowerState, operatorGamepad, lowButton) {
            @Override
            public boolean run() throws InterruptedException {
                clampSubsystem.goTo(ClampSubsystem.BackwardPos);
                sliderV2Subsystem.goTo(SliderSubsystem.LowPos);
                clampSubsystem.goTo(ClampSubsystem.ForwardPos);
                return true;
            }
        });
        SliderAndClampingFSM.add(new ButtonTransition(mediumState,groundState, operatorGamepad, groundButton) {
            @Override
            public boolean run() throws InterruptedException {
                clampSubsystem.goTo(ClampSubsystem.BackwardPos);
                sliderV2Subsystem.goTo(SliderSubsystem.SafePos);
                clampSubsystem.goTo(ClampSubsystem.ForwardPos);
                sliderV2Subsystem.goTo(SliderSubsystem.GroundPos);

                return true;
            }
        });
        SliderAndClampingFSM.add(new ButtonTransition(lowerState,upperState, operatorGamepad, highButton) {
            @Override
            public boolean run() throws InterruptedException {

                sliderV2Subsystem.goTo(SliderSubsystem.HighPos);
                return true;
            }
        });
        SliderAndClampingFSM.add(new ButtonTransition(lowerState,mediumState, operatorGamepad, midButton) {
            @Override
            public boolean run() throws InterruptedException {

                sliderV2Subsystem.goTo(SliderSubsystem.MediumPos);
                return true;
            }
        });
        SliderAndClampingFSM.add(new ButtonTransition(lowerState,groundState, operatorGamepad, groundButton) {
            @Override
            public boolean run() throws InterruptedException {

                sliderV2Subsystem.goTo(SliderSubsystem.GroundPos);
                return true;
            }
        });
        SliderAndClampingFSM.add(new ButtonTransition(groundState,upperState, operatorGamepad, highButton) {
            @Override
            public boolean run() throws InterruptedException {

                sliderV2Subsystem.goTo(SliderSubsystem.HighPos);
                return true;
            }
        });
        SliderAndClampingFSM.add(new ButtonTransition(groundState,mediumState, operatorGamepad, midButton) {
            @Override
            public boolean run() throws InterruptedException {

                sliderV2Subsystem.goTo(SliderSubsystem.MediumPos);
                return true;
            }
        });
        SliderAndClampingFSM.add(new ButtonTransition(groundState,lowerState, operatorGamepad, lowButton) {
            @Override
            public boolean run() throws InterruptedException {

                sliderV2Subsystem.goTo(SliderSubsystem.LowPos);
                return true;
            }
        });



//        State[] sliderSafeStates = {loadedState, upperState, mediumState, lowerState, groundState};
//        SliderAndClampingFSM.addTransitionsTo(upperState, sliderSafeStates, new ButtonTransition(operatorGamepad, GamepadKeys.Button.DPAD_UP) {
//            @Override
//            public boolean run() throws InterruptedException {
//                clampSubsystem.goTo(ClampSubsystem.BackwardPos);
//                SliderV2Subsystem.goTo(SliderV2Subsystem.HighPos);
//                clampSubsystem.goTo(ClampSubsystem.ForwardPos);
//                return true;
//            }
//        });
//        SliderAndClampingFSM.addTransitionsTo(mediumState, sliderSafeStates, new ButtonTransition(operatorGamepad, GamepadKeys.Button.DPAD_RIGHT) {
//
//            @Override
//            public boolean run() throws InterruptedException {
//                clampSubsystem.goTo(ClampSubsystem.BackwardPos);
//                SliderV2Subsystem.goTo(SliderV2Subsystem.MediumPos);
//                clampSubsystem.goTo(ClampSubsystem.ForwardPos);
//                return true;
//            }
//        });
//        SliderAndClampingFSM.addTransitionsTo(lowerState, sliderSafeStates, new ButtonTransition(operatorGamepad, GamepadKeys.Button.DPAD_LEFT) {
//            @Override
//            public boolean run() throws InterruptedException {
//                clampSubsystem.goTo(ClampSubsystem.BackwardPos);
//                SliderV2Subsystem.goTo(SliderV2Subsystem.LowPos);
//                clampSubsystem.goTo(ClampSubsystem.ForwardPos);
//                return true;
//            }
//        });
//        SliderAndClampingFSM.addTransitionsTo(groundState, sliderSafeStates, new ButtonTransition(operatorGamepad, GamepadKeys.Button.DPAD_DOWN) {
//            @Override
//            public boolean run() throws InterruptedException {
//                if(SliderV2Subsystem.isSafe()) clampSubsystem.goToBackward();
//                if(SliderV2Subsystem.getPosition()>=SliderV2Subsystem.GroundPos+10)
//                    clampSubsystem.goToForward();
//                SliderV2Subsystem.goTo(SliderV2Subsystem.GroundPos);
//                return true;
//            }
//        });
        State[] outsideStates = new State[]{lowerState, mediumState, upperState,groundState};
        SliderAndClampingFSM.addTransitionsTo(waitingState, outsideStates, new Transition() {
            Long lastTimeSinceClamped = null;

            @Override
            public boolean check() {

                if (lastTimeSinceClamped != null) {
                    if (System.currentTimeMillis() - lastTimeSinceClamped > 500) {
                        lastTimeSinceClamped = null;
                        return true;
                    }
                } else {
                    if (!clampSubsystem.isClamping()) {
                        lastTimeSinceClamped = System.currentTimeMillis();
                    }
                    return false;
                }
                return false;


            }

            @Override
            public boolean run() throws InterruptedException {
                if (sliderV2Subsystem.isSafe()) {
                    clampSubsystem.goTo(ClampSubsystem.BackwardPos);
                    sliderV2Subsystem.goTo(SliderSubsystem.PreLoadPos);
                } else {
                    sliderV2Subsystem.goTo(SliderSubsystem.SafePos);
                    clampSubsystem.goTo(ClampSubsystem.BackwardPos);
                    sliderV2Subsystem.goTo(SliderSubsystem.PreLoadPos);
                }
                return true;
            }
        });
        SliderAndClampingFSM.build();

        opMode.telemetry.update();
        Robot robot = this;

        if(camera) {

            WebcamUtil webcamUtil = new WebcamUtil(opMode.hardwareMap, opMode.telemetry);

            JunctionAdjuster junctionAdjuster = new JunctionAdjuster(webcamUtil, 2.54, opMode.telemetry, 45);
            webcamUtil.registerListener(junctionAdjuster);
            webcamUtil.start(true);
            movingState = new State(MovementFSM, "movementState") {
                @Override
                public void update() {
                    movementSubsystem.run(new SubsystemData() {{
                        this.driverGamepad = robot.driverGamepad;
                    }});
                    junctionAdjuster.moveCamera();
                }
            };
            homingState = new State(MovementFSM, "homingState") {
                @Override
                public void update() {
                    JunctionAdjuster.visionResults results = junctionAdjuster.value();
                    movementSubsystem.move(results.movementData.getY(), results.movementData.getX(), results.turn);
                }
            };
            MovementFSM.add(new ButtonTransition(movingState, homingState, driverGamepad, GamepadKeys.Button.B) {
                @Override
                public boolean run() {
                    junctionAdjuster.start();
                    return true;
                }
            });
            MovementFSM.add(new MovementTransition(homingState, movingState, driverGamepad) {
                @Override
                public boolean check() {
                    return super.check() || junctionAdjuster.inReach();
                }
            });
        }else {
            movingState = new State(MovementFSM, "movementState") {
                @Override
                public void update() {
                    movementSubsystem.run(new SubsystemData() {{
                        this.driverGamepad = robot.driverGamepad;
                    }});
                }
            };
        }
        MovementFSM.setInitialState(movingState);
        MovementFSM.build();
        subsystemData.driverGamepad=driverGamepad;
        subsystemData.operatorGamepad=operatorGamepad;

        IntakeFSM.setInitialState(inactiveState);
        IntakeFSM.add(new ButtonTransition(inactiveState, loweredState, operatorGamepad, GamepadKeys.Button.LEFT_BUMPER) {
            @Override
            public boolean run() throws InterruptedException {
                transferSubsystem.bControl=false;
                transferSubsystem.goTo(TransferSubsystem.lowerArmPos);
                transferSubsystem.blockWithLeg();
                Thread.sleep(100);
                intakeSubsystem.intake(IntakeSubsystem.power);
                return true;
            }
        });
        IntakeFSM.add(new ReleaseTransition(loweredState, inactiveState, operatorGamepad, GamepadKeys.Button.LEFT_BUMPER) {
            @Override
            
            public boolean run() throws InterruptedException {
                transferSubsystem.bControl=true;
                transferSubsystem.goTo(TransferSubsystem.idleArmPos);
                intakeSubsystem.stop();
                return true;
            }
        });
        IntakeFSM.add(new ButtonTransition(inactiveState, expulseState, operatorGamepad, GamepadKeys.Button.RIGHT_BUMPER) {
            @Override
            public boolean run() throws InterruptedException {
                intakeSubsystem.expulse();
                return true;
            }
        });
        IntakeFSM.add(new ReleaseTransition(expulseState, inactiveState, operatorGamepad, GamepadKeys.Button.RIGHT_BUMPER) {
            @Override
            public boolean run() throws InterruptedException {
                transferSubsystem.goTo(TransferSubsystem.idleArmPos);
                intakeSubsystem.stop();
                return true;
            }
        });
        IntakeFSM.add(new ButtonTransition(inactiveState, liftingState, operatorGamepad, GamepadKeys.Button.A) {
            @Override
            public boolean run() throws InterruptedException {
                transferSubsystem.bControl=false;
                transferSubsystem.retreatLeg();
                transferSubsystem.goTo(TransferSubsystem.upperArmPos);
                return true;
            }
        });
        IntakeFSM.add(new ReleaseTransition(liftingState, inactiveState, operatorGamepad, GamepadKeys.Button.A) {
            @Override
            public boolean run() throws InterruptedException {
                transferSubsystem.bControl=true;
                transferSubsystem.goTo(TransferSubsystem.idleArmPos);
                intakeSubsystem.stop();
                return true;
            }
        });
        IntakeFSM.build();
        modules=opMode.hardwareMap.getAll(LynxModule.class);
    }
    private boolean running = false;
    private final SubsystemData subsystemData = new SubsystemData();

    private List<LynxModule> modules;
    public void update() throws InterruptedException {
        for(LynxModule module: modules)
        {
            module.clearBulkCache();
        }
        if(opModeType == OpModeType.TeleOp)
        {
            subsystemData.driverGamepad.readButtons();
            subsystemData.operatorGamepad.readButtons();
        }else {

        }

//        SliderAndClampingFSM.update(opModeType == OpModeType.TeleOp);

        sliderV2Subsystem.run(subsystemData);
        clampSubsystem.run(subsystemData);

        if(opModeType == OpModeType.TeleOp)
        {
            MovementFSM.update(true);
            IntakeFSM.update(true);
            sensorSubsystem.run(null);

            transferSubsystem.run(subsystemData);
        }

    }
}