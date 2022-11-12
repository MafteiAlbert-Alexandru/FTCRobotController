package org.firstinspires.ftc.teamcode.hardware.customHardware;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class ToggleButton {

    private GamepadEx gamepadEx;
    private GamepadKeys.Button button;
    public boolean toggle;

    public ToggleButton(GamepadEx gamepadEx, GamepadKeys.Button button){
        this.gamepadEx = gamepadEx;
        this.button = button;

    }

    public void update(){
        if(gamepadEx.wasJustPressed(button)) {
            toggle = !toggle;
        }
    }

    public boolean getToggle(){
        return toggle;
    }
}

