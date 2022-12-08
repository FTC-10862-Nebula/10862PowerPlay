package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

//nowadays lance isnt a very common name, but in older times people were named lance a lot
public class ThreadComman extends SequentialCommandGroup{
    public ThreadComman(){
////        addRequirements(drivetrain);    //Add Subsystems that you need to run this Command
//        addCommands(
//                //Commands
//        );

        }
        public void runCommandGroupAsThreadNow(SequentialCommandGroup sequentialCommandGroup) {
            new Thread(
                    () -> {
//            if (!isStopRequested()) sequentialCommandGroup.initialize();
//
//            while (!isStopRequested() && !sequentialCommandGroup.isFinished()) {
                        sequentialCommandGroup.execute();
//            }
                    }
            ).start();
    }

}