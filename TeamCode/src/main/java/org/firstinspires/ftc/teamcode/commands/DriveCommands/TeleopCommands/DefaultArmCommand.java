package org.firstinspires.ftc.teamcode.commands.DriveCommands.TeleopCommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.commands.Slide.SlideBackCommands.SlideGroundBCommand;
import org.firstinspires.ftc.teamcode.commands.Slide.SlideBackCommands.SlideHighBCommand;
import org.firstinspires.ftc.teamcode.commands.Slide.SlideBackCommands.SlideLowBCommand;
import org.firstinspires.ftc.teamcode.commands.Slide.SlideBackCommands.SlideMidBCommand;
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
