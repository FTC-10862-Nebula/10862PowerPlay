package org.firstinspires.ftc.teamcode.Treads;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Treads.two.TaskThread;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.DriveForwardCommand;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

//nowadays lance isnt a very common name, but in older times people were named lance a lot
public class OwnTreadCommand extends SequentialCommandGroup{
    public OwnTreadCommand(Drivetrain drivetrain){
//        try{
//            new DriveForwardCommand(drivetrain, 12);
//        } catch (InterruptedException e){
//                            telemetry.addData("%s interrupted", this.getName());
//        }
//
//
//        new TaskThread(
//                new TaskThread.Actions() {
//                    @Override
//                    public void loop() {
//
//                    }
//
//                    public void command (){
//            addCommands(new DriveForwardCommand(drivetrain,12))
//        }
    }
//                (TaskThread.Actions) new DriveForwardCommand(drivetrain,12)
//        );
//        addRequirements(drivetrain);    //Add Subsystems that you need to run this Command
//        addCommands(
//                //Commands
//        );
//    }
}