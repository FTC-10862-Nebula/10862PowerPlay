package org.firstinspires.ftc.teamcode.commands.DriveCommands;

import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

public class SlowDriveCommandforKids extends DefaultDriveCommand {
    public SlowDriveCommandforKids(Drivetrain drive, GamepadEx driverGamepad) {
        super(drive, driverGamepad);
        this.multiplier = 0.5;
    }
}