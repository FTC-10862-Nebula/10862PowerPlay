package org.firstinspires.ftc.teamcode.commands.DriveCommands.TeleopCommands;

import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.subsystems.Drive.Drivetrain;

public class SlowDriveCommand extends DefaultDriveCommand {
    public SlowDriveCommand(Drivetrain drive, GamepadEx driverGamepad) {
        super(drive, driverGamepad, false);
        this.multiplier = 0.35;
    }
}