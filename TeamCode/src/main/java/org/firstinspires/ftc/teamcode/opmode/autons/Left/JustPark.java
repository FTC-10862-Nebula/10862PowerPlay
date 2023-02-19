package org.firstinspires.ftc.teamcode.opmode.autons.Left;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.commands.Drive.AutoCommands.DriveForwardCommand;
import org.firstinspires.ftc.teamcode.commands.Drive.AutoCommands.StrafeLeftCommand;
import org.firstinspires.ftc.teamcode.commands.Drive.AutoCommands.StrafeRightCommand;
import org.firstinspires.ftc.teamcode.subsystems.Drive.MatchOpMode;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Drive.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Misc.TagVision;
import org.firstinspires.ftc.teamcode.subsystems.Slide;
import org.firstinspires.ftc.teamcode.subsystems.TurnServo;

@Autonomous
@Disabled
public class JustPark extends MatchOpMode {
    private int tagNum = 0;

    // Subsystems
    private Arm arm;
    private Claw claw;
    private Drivetrain drivetrain;
    private Slide slide;
    private TagVision tagVision;
    private TurnServo turnServo;


    @Override
    public void robotInit() {
        claw = new Claw( telemetry, hardwareMap);
        arm = new Arm( telemetry, hardwareMap);
        drivetrain = new Drivetrain(new MecanumDrive(hardwareMap, telemetry, false), telemetry, hardwareMap);
        drivetrain.init();
        slide = new Slide(telemetry, hardwareMap);
        turnServo = new TurnServo(telemetry, hardwareMap);

        tagVision = new TagVision(hardwareMap,  telemetry);
        while (!isStarted() && !isStopRequested())
        {
            tagVision.updateTagOfInterest();
            tagVision.tagToTelemetry();
            telemetry.update();
        }
        this.matchStart();
    }

    public void matchStart() {tagNum = tagVision.getTag();

//        SequentialCommandGroup autonGroup;
        switch (tagNum) {
            case 1: { //Left
                schedule(
                        new SequentialCommandGroup(
                                new DriveForwardCommand(drivetrain, 30),
                                new StrafeLeftCommand(drivetrain, 19.5)
                        )


//                new StrafeRightCommand(drivetrain, 30),
//                new DriveForwardCommand(drivetrain, 15)

                );
                return;
            }
            case 2: { //Mid
                schedule(

                        new SequentialCommandGroup(
                                new DriveForwardCommand(drivetrain, 33)
                        )


//                new StrafeRightCommand(drivetrain, 30)
                        );
                return;
            }
            default: { //High
                schedule(
                        new SequentialCommandGroup(
                                new DriveForwardCommand(drivetrain, 30),
                                 new StrafeRightCommand(drivetrain, 20.2)


//                new StrafeRightCommand(drivetrain, 30),
//                        new DriveForwardCommand(drivetrain, 15)
                                )
                        );
                return;
            }
        }
    }
}