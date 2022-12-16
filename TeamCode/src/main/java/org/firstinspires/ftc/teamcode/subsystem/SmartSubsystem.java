package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.lang.reflect.Field;
import java.util.Objects;


public abstract class SmartSubsystem {


    public OpMode opMode;
    public HardwareMap hardwareMap;
    public boolean initialized;
    public abstract void run(SubsystemData data) throws InterruptedException;

    public void initSubsystem(OpMode opMode) {
        this.opMode=opMode;
        this.hardwareMap=opMode.hardwareMap;
        this.initialized=true;
    }

    public static  void initAllSubsystems(OpMode opMode) throws IllegalAccessException {
        for(Field field: opMode.getClass().getDeclaredFields())
        {
            if(field.getType().getSuperclass()!=null && field.getType().getSuperclass().equals(SmartSubsystem.class))
            {
                SmartSubsystem subsystem =  ((SmartSubsystem) Objects.requireNonNull(field.get(opMode)));
                try {
                    subsystem.initSubsystem((OpMode) opMode);
                } catch (Exception e) {
                    subsystem.initialized=false;
                    opMode.telemetry.addLine(String.format("Failed initializing %s", field.getName()));
                    opMode.telemetry.addLine(e.toString());
                }
            }
        }
    }
    public static <T> void initAllSubsystems(T object, OpMode opMode) throws IllegalAccessException {

        for(Field field: object.getClass().getDeclaredFields())
        {
            if(field.getType().getSuperclass()!=null && field.getType().getSuperclass().equals(SmartSubsystem.class))
            {
                SmartSubsystem subsystem =  ((SmartSubsystem) Objects.requireNonNull(field.get(object)));
                try {
                    subsystem.initSubsystem((OpMode) opMode);
                } catch (Exception e) {
                    subsystem.initialized=false;
                    opMode.telemetry.addLine(String.format("Failed initializing %s", field.getName()));
                    opMode.telemetry.addLine(e.toString());
                }
            }
        }
    }
}

