package org.firstinspires.ftc.teamcode.hardware.customHardware;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.apache.commons.math3.analysis.function.Pow;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class SmartMotorEx extends SmartMotor {

    public DcMotorEx motorEx;

    /**
     * Constructs the instance motor for the wrapper
     *
     * @param hMap the hardware map from the OpMode
     * @param id   the device id from the RC config
     */
    public SmartMotorEx(@NonNull HardwareMap hMap, String id) {
        this(hMap, id, GoBILDA.NONE, MotorDirection.FORWARD);
        ACHIEVABLE_MAX_TICKS_PER_SECOND = motorEx.getMotorType().getAchieveableMaxTicksPerSecond();
    }
    /**
     * Constructs the instance motor for the wrapper
     *
     * @param hMap        the hardware map from the OpMode
     * @param id          the device id from the RC config
     * @param gobildaType the type of gobilda 5202 series motor being used
     */
    public SmartMotorEx(@NonNull HardwareMap hMap, String id, @NonNull GoBILDA gobildaType, MotorDirection type) {
        super(hMap, id, gobildaType, type);
        motorEx = (DcMotorEx) super.motor;
    }

    public SmartMotorEx(@NonNull HardwareMap hMap, String id, @NonNull GoBILDA gobildaType) {
        super(hMap, id, gobildaType, MotorDirection.FORWARD);
        motorEx = (DcMotorEx) super.motor;
    }

    public SmartMotorEx(@NonNull HardwareMap hMap, String id, @NonNull NeveRest gobildaType, MotorDirection type) {
        super(hMap, id, gobildaType, type);
        motorEx = (DcMotorEx) super.motor;
    }

    public SmartMotorEx(@NonNull HardwareMap hMap, String id, @NonNull NeveRest gobildaType) {
        super(hMap, id, gobildaType, MotorDirection.FORWARD);
        motorEx = (DcMotorEx) super.motor;
    }

    /**
     * Constructs an instance motor for the wrapper
     *
     * @param hMap the hardware map from the OpMode
     * @param id   the device id from the RC config
     * @param cpr  the counts per revolution of the motor
     * @param rpm  the revolutions per minute of the motor
     */
    public SmartMotorEx(@NonNull HardwareMap hMap, String id, double cpr, double rpm, MotorDirection Type) {
        super(hMap, id, cpr, rpm, Type);
        motorEx = (DcMotorEx) super.motor;
    }

    @Override
    public void set(double output) {
        if (runmode == RunMode.VelocityControl) {
            double speed = bufferFraction * output * ACHIEVABLE_MAX_TICKS_PER_SECOND;
            double velocity = veloController.calculate(getCorrectedVelocity(), speed) + feedforward.calculate(speed, getAcceleration());
            motorEx.setPower(velocity / ACHIEVABLE_MAX_TICKS_PER_SECOND);
        } else if (runmode == RunMode.PositionControl) {
            double error = positionController.calculate(encoder.getPosition());
            motorEx.setPower(output * error);
        } else {
            motorEx.setPower(output);
        }
    }

    /**
     * @param velocity the velocity in ticks per second
     */
    public void setVelocity(double velocity) {
        set(velocity / ACHIEVABLE_MAX_TICKS_PER_SECOND);
    }

    public void setPower(double Power){
        motorEx.setPower(Power);
    }

    /**
     * Sets the velocity of the motor to an angular rate
     *
     * @param velocity  the angular rate
     * @param angleUnit radians or degrees
     */
    public void setVelocity(double velocity, AngleUnit angleUnit) {
        setVelocity(getCPR() * AngleUnit.RADIANS.fromUnit(angleUnit, velocity) / (2 * Math.PI));
    }

    /**
     * @return the velocity of the motor in ticks per second
     */
    @Override
    public double getVelocity() {
        return motorEx.getVelocity();
    }

    /**
     * @return the acceleration of the motor in ticks per second squared
     */
    public double getAcceleration() {
        return encoder.getAcceleration();
    }

    @Override
    public String getDeviceType() {
        return "Extended " + super.getDeviceType();
    }
}

