package org.firstinspires.ftc.teamcode.commands.DriveCommands.TeleopCommands;

import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.subsystems.Drive.Drivetrain;

public class SlowerDriveCommand extends DefaultDriveCommand {
    public SlowerDriveCommand(Drivetrain drive, GamepadEx driverGamepad) {
        super(drive, driverGamepad, false, 2);
        this.multiplier = 0.35;
    }
}