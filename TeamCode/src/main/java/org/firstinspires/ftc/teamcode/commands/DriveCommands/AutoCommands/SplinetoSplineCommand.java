package org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Trajectories;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.util.PoseStorage;

@Config
public class SplinetoSplineCommand extends CommandBase{

    Drivetrain drive;
    Trajectory trajectory;
    boolean reverse = false;
    Pose2d splinePos;
    double endHeading;

    MinVelocityConstraint maxVelConstraint;
    public SplinetoSplineCommand(Drivetrain drive, MinVelocityConstraint constraint, boolean reverse, Pose2d splinePos, double endHeading) {
        this.drive = drive;
        this.maxVelConstraint = constraint;
        this.reverse = reverse;
        this.splinePos = splinePos;
        this.endHeading = endHeading;

        this.addRequirements(drive);
    }

    public SplinetoSplineCommand(Drivetrain drive, Pose2d splinePos, double endHeading) {
        this(drive, Trajectories.velConstraint, false, splinePos, endHeading);
    }

    public SplinetoSplineCommand(Drivetrain drive, Pose2d splinePos, double endHeading, boolean reverse) {
        this(drive, Trajectories.velConstraint, reverse, splinePos, endHeading);
    }


    @Override
    public void initialize() {
//        trajectory = new TrajectoryBuilder(drive.getPoseEstimate(), reverse, maxVelConstraint, Trajectories.accelConstraint)
//                .splineToSplineHeading(splinePos, endHeading)
//                .build();

        trajectory = new TrajectoryBuilder(PoseStorage.currentPose, reverse, maxVelConstraint, Trajectories.accelConstraint)
                .splineToSplineHeading(splinePos, endHeading)
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