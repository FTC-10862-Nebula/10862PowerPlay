package org.firstinspires.ftc.teamcode.commands.drive.teleopCommands;

import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;

public class SlowDriveCommand extends DefaultDriveCommand {
    public SlowDriveCommand(Drivetrain drive, GamepadEx driverGamepad) {
        super(drive, driverGamepad, false);
        this.multiplier = 0.3;
    }
}