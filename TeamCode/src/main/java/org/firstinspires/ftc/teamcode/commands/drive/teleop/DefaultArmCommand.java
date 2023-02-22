package org.firstinspires.ftc.teamcode.commands.drive.teleop;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.subsystems.Pivot;

public class DefaultArmCommand extends CommandBase {
    private Pivot pivot;
    private GamepadEx operatorGamepad;

    protected double multiplier;
    boolean mecDrive = true;
    int choice = 0;

    public DefaultArmCommand(Pivot pivot, GamepadEx operatorGamepad) {

        this.pivot = pivot;
        this.operatorGamepad = operatorGamepad;

        addRequirements(this.pivot);

    }

    @Override
    public void execute() {
        if(operatorGamepad.getRightY()>0.5) {
            pivot.raiseClawManual();
        } else if(operatorGamepad.getRightY()<-0.5){
            pivot.lowerClawManual();
        }
    }


    @Override
    public void end(boolean interrupted) {
//        arm.stopArm();
    }
}
