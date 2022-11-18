//package org.firstinspires.ftc.teamcode.teleop;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.arcrobotics.ftclib.gamepad.GamepadEx;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.teamcode.hardware.Hardware_Map;
//import org.firstinspires.ftc.teamcode.hardware.customHardware.ToggleButton;
//import org.firstinspires.ftc.teamcode.subsystem.SliderSubsystem;
//import org.firstinspires.ftc.teamcode.subsystem.MovementSubsystem;
//import org.firstinspires.ftc.teamcode.subsystem.TransferSubsystem;
//
//@TeleOp
//@Config
//public class PedroTeleOp extends LinearOpMode {
//
//    //region Subsystem
//    SliderSubsystem intakeSubsystem;
//    TransferSubsystem transferSubsystem;
//    SliderSubsystem sliderSubsystem;
//
//    ToggleButton intakeToggle;
//    ToggleButton armToggle;
//    ToggleButton jointToggle;
//    ToggleButton legToggle;
//
//    GamepadEx driverPad;
//    GamepadEx operatorPad;
//
//    Hardware_Map hardware;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
///*
//        hardware = new Hardware_Map(hardwareMap);
//
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//
//        driverPad = new GamepadEx(gamepad1);
//        operatorPad = new GamepadEx(gamepad2);
//        movementSubsystem = new MovementSubsystem(driverPad, hardware.frontLeft, hardware.backLeft, hardware.backRight, hardware.frontRight);
//        intakeSubsystem = new IntakeSubsystem(hardware.leftIntake, hardware.rightIntake);
//        transferSubsystem = new TransferSubsystem(hardware.arm, hardware.joint1, hardware.joint2, hardware.leg, this);
//        sliderSubsystem = new SliderSubsystem(operatorPad, hardware.Left_Slider, hardware.Right_Slider, this);
//
//        waitForStart();
//        sliderSubsystem.SetupSliders();
//        // Using a PS4 Controller: Cross = A, Circle = B, Triangle = Y, Square = X
//
//        intakeToggle = new ToggleButton(driverPad, GamepadKeys.Button.A);
//        armToggle = new ToggleButton(driverPad, GamepadKeys.Button.B);
//        jointToggle = new ToggleButton(driverPad, GamepadKeys.Button.X);
//        legToggle = new ToggleButton(driverPad, GamepadKeys.Button.Y);
//
//        while (opModeIsActive()){
//            GamepadInput();
//
//            InputManager();
//            TelemetryManager(hardware.Left_Slider);
//        }*/
//    }
///*
//    public void InputManager(){
//
//        intakeSubsystem.runIntake(intakeToggle.getToggle());
//        transferSubsystem.runArm(armToggle.getToggle());
//        transferSubsystem.runJoint(jointToggle.getToggle());
//        transferSubsystem.runLeg(legToggle.getToggle());
//        movementSubsystem.movementMecanum();
//    }
//
//    public void TelemetryManager(SmartMotorEx leftSlider){
//        telemetry.addLine("Intake toggle " + intakeToggle.getToggle());
//        telemetry.addLine("Lift toggle " + armToggle.getToggle());
//        telemetry.addLine("Joint toggle" + jointToggle.getToggle());
//        telemetry.addLine("leg toggle " + legToggle.getToggle());
//        telemetry.addLine("Slider pos" + leftSlider.getCurrentPosition());
//        telemetry.addLine("Arm pos" + hardware.arm.getPosition());
//        telemetry.update();
//    }
//
//    public void GamepadInput(){
//        driverPad.readButtons();
//        operatorPad.readButtons();
//        intakeToggle.update();
//        armToggle.update();
//        jointToggle.update();
//        legToggle.update();
//    }*/
//}