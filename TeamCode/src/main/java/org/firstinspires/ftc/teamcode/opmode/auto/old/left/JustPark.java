package org.firstinspires.ftc.teamcode.opmode.auto.old.left;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.commands.drive.trajectory.DriveForwardCommand;
import org.firstinspires.ftc.teamcode.commands.drive.trajectory.StrafeLeftCommand;
import org.firstinspires.ftc.teamcode.commands.drive.trajectory.StrafeRightCommand;
import org.firstinspires.ftc.teamcode.subsystems.Pivot;
import org.firstinspires.ftc.teamcode.util.MatchOpMode;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.misc.TagVision;
import org.firstinspires.ftc.teamcode.subsystems.Slide;
import org.firstinspires.ftc.teamcode.subsystems.TurnServo;

@Autonomous
//@Disabled
public class JustPark extends MatchOpMode {
    private int tagNum = 0;

    // Subsystems
    private Pivot pivot;
    private Claw claw;
    private Drivetrain drivetrain;
    private Slide slide;
    private TagVision tagVision;
    private TurnServo turnServo;


    @Override
    public void robotInit() {
        claw = new Claw( telemetry, hardwareMap);
        pivot = new Pivot( telemetry, hardwareMap);
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
                                new DriveForwardCommand(drivetrain, 34.5),
                                new StrafeLeftCommand(drivetrain, 22.5)
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
                                 new StrafeRightCommand(drivetrain, 23)


//                new StrafeRightCommand(drivetrain, 30),
//                        new DriveForwardCommand(drivetrain, 15)
                                )
                        );
                return;
            }
        }
    }
}