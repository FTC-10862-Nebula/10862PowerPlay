package org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Trajectories;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

@Config
public class StrafeLeftCommand extends CommandBase{

    Drivetrain drive;
    double distance;
    Trajectory trajectory;
    MinVelocityConstraint constraint;
    public StrafeLeftCommand(Drivetrain drive, double distance) {
        this.drive = drive;
        this.distance = distance;
        constraint = Trajectories.velConstraint;
        this.addRequirements(drive);
    }

    public StrafeLeftCommand(Drivetrain drive, double distance, MinVelocityConstraint constraint) {
        this.drive = drive;
        this.distance = distance;
        this.constraint = constraint;
        this.addRequirements(drive);
    }

    @Override
    public void initialize() {
        trajectory = new TrajectoryBuilder(drive.getPoseEstimate(), constraint, Trajectories.accelConstraint).strafeLeft(distance).build();
//        trajectory = drive.trajectoryBuilder(new Pose2d())
//                .strafeLeft(distance)
//                .build();

        drive.followTrajectory(trajectory);

    }

    @Override
    public void execute() {
        drive.update();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            drive.stop();
        }
    }

    @Override
    public boolean isFinished() {
        return !drive.isBusy();
    }
}