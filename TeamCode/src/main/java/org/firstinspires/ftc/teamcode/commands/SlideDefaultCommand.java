package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Slide;

public class SlideDefaultCommand extends SequentialCommandGroup{
    private Slide slide;
    private GamepadEx operatorGamepad;

    public SlideDefaultCommand(Slide slide, GamepadEx operatorGamepad) {
            this.slide=slide;
            this.operatorGamepad=operatorGamepad;
            addRequirements(this.slide);
    }

    @Override
    public void execute(){
            slide.setPower(-operatorGamepad.getLeftY());
    }


    @Override
    public void end(boolean interrupted) {slide.stopSlide();}
    }
