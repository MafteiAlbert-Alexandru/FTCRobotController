package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

public class MovementSubsystem {

    private final DcMotorEx frontLeft;
    private final DcMotorEx frontRight;
    private final DcMotorEx backLeft;
    private final DcMotorEx backRight;
    private final GamepadEx gamepad;

    public MovementSubsystem(GamepadEx gamepad, DcMotorEx frontLeft, DcMotorEx backLeft, DcMotorEx backRight, DcMotorEx frontRight){
        this.frontLeft = frontLeft;
        this.backLeft = backLeft;
        this.backRight = backRight;
        this.frontRight = frontRight;
        this.gamepad = gamepad;
    }

    public void movementMecanum(){

        double Forward = gamepad.getLeftY();
        double Strafe = gamepad.getLeftX();
        double Turn = gamepad.getRightX();

        if(!gamepad.isDown(GamepadKeys.Button.LEFT_BUMPER)){
            Forward /= 2;
            Strafe  /= 2;
        }
        if(!gamepad.isDown(GamepadKeys.Button.RIGHT_BUMPER)){
            Turn /= 2;
        }//

        double r = Math.hypot(Strafe, Forward);

        double robotAngle = Math.atan2(Forward, Strafe) - Math.PI / 4;

        final double v1 = (r * Math.cos(robotAngle)) + Turn;
        final double v2 = (r * Math.sin(robotAngle)) - Turn;
        final double v3 = (r * Math.sin(robotAngle)) + Turn;
        final double v4 = (r * Math.cos(robotAngle)) - Turn;

        frontLeft.setPower(v1);
        frontRight.setPower(v2);
        backLeft.setPower(v3);
        backRight.setPower(v4);
    }
}
