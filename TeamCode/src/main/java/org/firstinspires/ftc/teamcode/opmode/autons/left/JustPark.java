package org.firstinspires.ftc.teamcode.opmode.autons.left;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.commands.driveCommands.autoCommands.DriveForwardCommand;
import org.firstinspires.ftc.teamcode.commands.driveCommands.autoCommands.StrafeLeftCommand;
import org.firstinspires.ftc.teamcode.commands.driveCommands.autoCommands.StrafeRightCommand;
import org.firstinspires.ftc.teamcode.subsystems.drive.MatchOpMode;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.misc.TagVision;
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