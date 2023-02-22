package org.firstinspires.ftc.teamcode.commands.drive.teleop;

import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;

public class SlowerDriveCommand extends DefaultDriveCommand {
    public SlowerDriveCommand(Drivetrain drive, GamepadEx driverGamepad) {
        super(drive, driverGamepad, false);
        this.multiplier = 0.35;
    }
}