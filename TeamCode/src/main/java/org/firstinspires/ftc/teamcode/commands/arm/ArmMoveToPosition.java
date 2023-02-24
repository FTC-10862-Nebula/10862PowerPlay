package org.firstinspires.ftc.teamcode.commands.arm;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.Arm;

import java.util.ArrayList;
import java.util.List;

public class ArmMoveToPosition extends SequentialCommandGroup {
    List<Command> commands = new ArrayList<Command>();
    public ArmMoveToPosition(Arm arm, Arm.State.Side side, Arm.State.Position position) {
        addRequirements(arm.slide, arm.claw, arm.pivot, arm.turnServo);
    }
}
