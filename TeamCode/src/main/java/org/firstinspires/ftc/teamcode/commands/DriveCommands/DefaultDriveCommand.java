package org.firstinspires.ftc.teamcode.commands.DriveCommands;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

public class DefaultDriveCommand extends CommandBase {
    private Drivetrain drive;
    private GamepadEx driverGamepad;

    protected double multiplier;

    public DefaultDriveCommand(Drivetrain drive, GamepadEx driverGamepad) {

        this.drive = drive;
        this.driverGamepad = driverGamepad;

        this.multiplier = 1.1;
        addRequirements(this.drive);
    }

    @Override
    public void execute() {
//      Arcade Drive
//      drive.arcadeDrive(driverGamepad.getLeftY() * multiplier, driverGamepad.getRightX() * multiplier);

//      Tank Drive
//      https://github.com/FTCLib/RoadRunner-FTCLib-Quickstart/blob/main/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/commands/MecanumDriveCommand.java
//      drive.tankDrive(driverGamepad.getLeftY() * multiplier, driverGamepad.getRightY() * multiplier)

//      Mecanum drive
        drive.mecDrive(
                -driverGamepad.getLeftY(), //Removed - from drivergamepad
                -driverGamepad.getLeftX() * multiplier,
                -driverGamepad.getRightX() //Changed from -driverGamepad.getLeftY(), so the drive mturns right
        );




//        FieldCentric
//        drive.fieldCentric(
//                -driverGamepad.getLeftY(),
//                driverGamepad.getRightX(),
//                -driverGamepad.getRightX()
//        );
    }


    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }
}
