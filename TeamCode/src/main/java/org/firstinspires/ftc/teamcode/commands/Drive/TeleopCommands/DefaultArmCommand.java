package org.firstinspires.ftc.teamcode.commands.Drive.TeleopCommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.subsystems.Arm;

public class DefaultArmCommand extends CommandBase {
    private Arm arm;
    private GamepadEx operatorGamepad;

    protected double multiplier;
    boolean mecDrive = true;
    int choice = 0;

    public DefaultArmCommand(Arm arm, GamepadEx operatorGamepad) {

        this.arm = arm;
        this.operatorGamepad = operatorGamepad;

        addRequirements(this.arm);

    }

    @Override
    public void execute() {
        if(operatorGamepad.getRightY()>0.5) {
            arm.raiseClawManual();
        } else if(operatorGamepad.getRightY()<-0.5){
            arm.lowerClawManual();
        }
    }


    @Override
    public void end(boolean interrupted) {
//        arm.stopArm();
    }
}
