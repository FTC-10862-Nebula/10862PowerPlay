package org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Trajectories;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.PoseStorage;

@Config
public class SlowSplineCommand extends CommandBase{

    MecanumDrive drive;
    Trajectory trajectory;
    boolean reverse = false;
    Vector2d splinePos;
    double endHeading;

    MinVelocityConstraint maxVelConstraint;
    public SlowSplineCommand(MecanumDrive drive, MinVelocityConstraint constraint, boolean reverse, Vector2d splinePos, double endHeading) {
        this.drive = drive;
        this.reverse = reverse;
        this.splinePos = splinePos;
        this.endHeading = endHeading;
        this.maxVelConstraint = constraint;
        this.addRequirements(drive);
    }

    public SlowSplineCommand(MecanumDrive drive, Vector2d splinePos, double endHeading) {
        this(drive, Trajectories.kindaSlowVelConstraint, false, splinePos, endHeading);
    }

    public SlowSplineCommand(MecanumDrive drive, Vector2d splinePos, double endHeading, boolean reverse) {
        this(drive, Trajectories.kindaSlowVelConstraint, reverse, splinePos, endHeading);
    }


    @Override
    public void initialize() {
//        trajectory = new TrajectoryBuilder(drive.getPoseEstimate(), reverse, maxVelConstraint, Trajectories.accelConstraint)
//                .splineTo(splinePos, endHeading)
//                .build();

        trajectory = new TrajectoryBuilder(PoseStorage.currentPose, reverse, maxVelConstraint, Trajectories.accelConstraint)
                .splineTo(splinePos, endHeading)
                .build();
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
        PoseStorage.currentPose = trajectory.end(); //TODO:Test
        return !drive.isBusy();
    }
}