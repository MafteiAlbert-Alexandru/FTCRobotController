package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.apache.commons.math3.geometry.euclidean.twod.Line;
import org.firstinspires.ftc.teamcode.hardware.customHardware.SmartMotor;
import org.firstinspires.ftc.teamcode.hardware.customHardware.SmartMotorEx;
import org.firstinspires.ftc.teamcode.hardware.customHardware.ToggleButton;


@Config
public class SliderSubsystem extends SmartSubsystem {

    public SmartMotorEx slider;
    public static boolean autoBrake = false;
    public static double target = 0;
    public static double DPP = 1;
    public static double tolerance = 1000;
    public static double coefficient = 0.2;
    public static double pow = 0.1;
    public static double inputTickCoef = 3;
    public static int groundPos = 0;
    public static int lowPos = 0;
    public static int midPos = 0;
    public static int highPos = 600;

    public static double power = 0.65;

    public GamepadEx gamepadEx;
    public void run()
    {
        if(!opMode.opModeIsActive()) return;
        //Siguranta in cazul in care cineva batut in cap foloseste Subsystemul asta gresit
        SliderInput();
        SetupSlider(); //slider go brr
    }

    @Override
    public void initSubsystem(LinearOpMode opMode, HardwareMap hardwareMap, GamepadEx gamepad) {
        this.gamepadEx = gamepad;
        super.initSubsystem(opMode, hardwareMap);
        slider=new SmartMotorEx(hardwareMap, "slider", SmartMotor.NeveRest.RPM_1780, SmartMotor.MotorDirection.REVERSE);
    }

    public SliderSubsystem(GamepadEx gamepad){
        this.gamepad = gamepad;
        SetupSlider(); //bag setup ul in constructor ca sa mi l cheme in init (asa nu mai stau eu sa ciordesc in clasa opModeIsActive)
    }

    void SetupSlider(){
        slider = new SmartMotorEx(hardwareMap, "slider");
        slider.resetEncoder();
        slider.setRunMode(SmartMotor.RunMode.PositionControl);
        slider.setZeroPowerBehavior(SmartMotor.ZeroPowerBehavior.FLOAT);
        slider.setInverted(true);
    }


    void SliderInput(){
        if(gamepadEx.wasJustPressed(GamepadKeys.Button.DPAD_UP)) target = highPos;
        else if(gamepadEx.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) target = midPos;
        else if(gamepadEx.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) target = lowPos;
        else if(gamepadEx.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) target = groundPos;
        else{
            if(gamepadEx.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)>0) target +=gamepadEx.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)*inputTickCoef; //Cand apas pe LB liftul urca
            if(gamepadEx.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)>0) target -=gamepadEx.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)*inputTickCoef; //Cand apas pe LB liftul urca
        }

        TargetClamp();
    }

    void SliderPID(){
        //Aici am vrut brake ul intern al motorului
        if(autoBrake) slider.setZeroPowerBehavior(SmartMotor.ZeroPowerBehavior.BRAKE);
        else slider.setZeroPowerBehavior(SmartMotor.ZeroPowerBehavior.FLOAT);

        //La fiecare tick dau "refresh" la variabile ca daca vreau sa mai ajustez PID ul
        slider.setDistancePerPulse(DPP);
        slider.setPositionTolerance(tolerance);
        slider.setPositionCoefficient(coefficient);
        slider.set(pow);
        slider.setTargetPosition((int)target); //l am convertit in int pt ca (vezi linia 74 si 75) ajungeam sa am zecimale
        //si targetul poate sa fie doar int
    }

    //Nu am gasit o funtie de clamp nu ma judeca:))
    void TargetClamp(){
        if(target > highPos) target = highPos;
        else if(target < groundPos) target = groundPos;
    }
}