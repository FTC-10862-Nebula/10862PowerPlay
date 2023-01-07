package org.firstinspires.ftc.teamcode.commands.DriveCommands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.PoseStorage;

//nowadays lance isnt a very common name, but in older times people were named lance a lot
public class ResetPoseCommand extends SequentialCommandGroup{
    public ResetPoseCommand(Drivetrain drivetrain, Pose2d newPose){
        drivetrain.setPoseEstimate(newPose);
        PoseStorage.currentPose = newPose; //TODO:Test

    }
}