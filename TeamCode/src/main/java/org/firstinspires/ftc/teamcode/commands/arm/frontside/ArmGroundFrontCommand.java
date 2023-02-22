package org.firstinspires.ftc.teamcode.commands.arm.frontside;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.Pivot;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Slide;
import org.firstinspires.ftc.teamcode.subsystems.TurnServo;

public class ArmGroundFrontCommand extends SequentialCommandGroup {
    public ArmGroundFrontCommand(Slide slide, Pivot pivot, Claw claw, TurnServo turnServo, boolean auto) {
        if (auto){
            addCommands(
//                    new InstantCommand(claw::setFClawPos),
                    new ParallelCommandGroup(
                            new InstantCommand(() ->
                                    new Thread(() -> {
                                        turnServo.setFClawPos();
                                        slide.slideGround();
                                    }).start())
                    ),
                    new WaitCommand(800),
                    new ParallelCommandGroup(
                            new InstantCommand(() ->
                                    new Thread(() -> {
                                        claw.clawClose();
//                                        slide.slideGround();
                                        pivot.moveFAuto();
                                    }).start())
                    )
            );
        }
        else {
            addCommands(
//                    new InstantCommand(claw::setFClawPos),
                    new ParallelCommandGroup(
                            new InstantCommand(() ->
                                    new Thread(() -> {
                                        turnServo.setFClawPos();
                                        slide.slideGround();
                                    }).start())
                    ),
                    new WaitCommand(1500),
                    new ParallelCommandGroup(
                            new InstantCommand(() ->
                                    new Thread(() -> {
                                        claw.clawClose();
//                                        slide.slideGround();
                                        pivot.moveGroundF();
                                    }).start())
                    )

            );
        }

    }
}
