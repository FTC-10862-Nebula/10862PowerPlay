package org.firstinspires.ftc.teamcode.subsystems.Stuff;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class HardwareSubsystem extends SubsystemBase {
    protected static Telemetry telemetry;
    protected static HardwareMap hardwareMap;
    public HardwareSubsystem(OpMode opMode) {
        telemetry = opMode.telemetry;
        hardwareMap = opMode.hardwareMap;
        initializeConstants();
    }
    public static void initializeConstants() {

    }
    public abstract void init();
    public abstract void periodic();

}