package org.firstinspires.ftc.teamcode.commands.driveCommands.teleopCommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;

public class DefaultDriveCommand extends CommandBase {
    private Drivetrain drive;
    private GamepadEx driverGamepad;

    protected double multiplier;
    boolean mecDrive;

    public DefaultDriveCommand(Drivetrain drive, GamepadEx driverGamepad, boolean isFieldCentric) {

        this.drive = drive;
        this.driverGamepad = driverGamepad;

        this.multiplier = 1.1;
        addRequirements(this.drive);

        this.mecDrive = isFieldCentric;
    }

    @Override
    public void execute() {
        if(mecDrive)
        {
            drive.mecDrive(
                driverGamepad.getLeftY() * multiplier, //Removed - from drivergamepad
                driverGamepad.getLeftX() * multiplier,
                driverGamepad.getRightX() * multiplier //Changed from -driverGamepad.getLeftY(), so the drive mturns right
            );
        } else{
            drive.fieldCentric(
                    driverGamepad.getLeftY() * multiplier,
                    driverGamepad.getLeftX() * multiplier,
                    -driverGamepad.getRightX() * multiplier
            );
        }
    }



    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }
}
