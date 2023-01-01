package org.firstinspires.ftc.teamcode.commands.DriveCommands.TeleopCommands;

import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;

public class SlowDriveCommand extends DefaultDriveCommand {
    public SlowDriveCommand(MecanumDrive drive, GamepadEx driverGamepad, int choice) {
        super(drive, driverGamepad, false, choice);
        this.multiplier = 0.5;
    }
}