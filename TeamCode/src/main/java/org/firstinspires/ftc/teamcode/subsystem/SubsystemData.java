package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.gamepad.GamepadEx;

public class SubsystemData {
    public GamepadEx driverGamepad, operatorGamepad;

    public SubsystemData() {}
    // Adauga aici ca sa te asiguri ca se copie cum trebuie in multithreading
    public SubsystemData(SubsystemData data) {
        this.driverGamepad=data.driverGamepad;
        this.operatorGamepad=data.operatorGamepad;
        // NEAPARAT FACI SA COPIE AICI CAND ADAUGI DATE
    }
    // You add whatever data that you want here
    // poti adauga si valorile la encoder ca sa ruleze in paralel
}
