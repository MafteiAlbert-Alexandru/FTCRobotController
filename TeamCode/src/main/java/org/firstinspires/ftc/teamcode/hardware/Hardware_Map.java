package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.customHardware.SmartMotor;
import org.firstinspires.ftc.teamcode.hardware.customHardware.SmartMotorEx;

public class Hardware_Map{

    public HardwareMap hardwareMap;

    public DcMotorEx frontLeft, backLeft, backRight, frontRight;

    public DcMotor leftIntake, rightIntake;

    public SmartMotorEx Left_Slider, Right_Slider;

    public Servo arm, joint1, joint2, leg;

    public Hardware_Map(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        SetupHardware();
    }

    public void SetupHardware(){
        frontLeft = hardwareMap.get(DcMotorEx.class, "fl");
        backLeft = hardwareMap.get(DcMotorEx.class, "bl");
        backRight = hardwareMap.get(DcMotorEx.class, "br");
        frontRight = hardwareMap.get(DcMotorEx.class, "fr");

        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        frontLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        backLeft.setDirection(DcMotorEx.Direction.REVERSE);
        backRight.setDirection(DcMotorEx.Direction.FORWARD);
        frontRight.setDirection(DcMotorEx.Direction.FORWARD);

        Left_Slider = new SmartMotorEx(hardwareMap, "Left_Slider", SmartMotorEx.GoBILDA.RPM_223, SmartMotorEx.MotorDirection.FORWARD);
        Right_Slider = new SmartMotorEx(hardwareMap, "Right_Slider", SmartMotorEx.GoBILDA.RPM_223, SmartMotorEx.MotorDirection.REVERSE);

        leftIntake = hardwareMap.get(DcMotor.class, "leftIntake");
        rightIntake = hardwareMap.get(DcMotor.class, "rightIntake");

        leftIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftIntake.setDirection(DcMotorSimple.Direction.FORWARD);
        rightIntake.setDirection(DcMotorSimple.Direction.REVERSE);

        arm = hardwareMap.get(Servo.class, "arm");
        joint1 = hardwareMap.get(Servo.class, "joint1");
        joint2 = hardwareMap.get(Servo.class, "joint2");
        leg = hardwareMap.get(Servo.class, "leg");

        arm.setDirection(Servo.Direction.FORWARD);
        joint1.setDirection(Servo.Direction.FORWARD);
        joint2.setDirection(Servo.Direction.REVERSE);
        leg.setDirection(Servo.Direction.FORWARD);
    }
}