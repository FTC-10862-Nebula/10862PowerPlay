package org.firstinspires.ftc.teamcode.opmode.autons.Right;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.commands.Auto.AutoConeCommands.PickConeCommand;
import org.firstinspires.ftc.teamcode.commands.Auto.Old.RightHighJunctionCommand;
import org.firstinspires.ftc.teamcode.commands.Drive.AutoCommands.DriveForwardCommand;
import org.firstinspires.ftc.teamcode.commands.Drive.AutoCommands.StrafeRightCommand;
import org.firstinspires.ftc.teamcode.commands.Drive.AutoCommands.TurnCommand;
import org.firstinspires.ftc.teamcode.commands.Drive.AutoCommands.TurnToCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeAndOutake.DropAutoConeCommand;
import org.firstinspires.ftc.teamcode.commands.Slide.SlideFCommands.SlideHighFCommand;
import org.firstinspires.ftc.teamcode.commands.Slide.SlideResetUpAutonCommand;
import org.firstinspires.ftc.teamcode.subsystems.Drive.MatchOpMode;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Drive.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Slide;
import org.firstinspires.ftc.teamcode.subsystems.Misc.TagVision;
import org.firstinspires.ftc.teamcode.subsystems.TurnServo;
@Disabled
@Autonomous(name="rightNEW")
public class RightJunctionNEWAuton extends MatchOpMode {
//    private ATDetector tagDetector;

    private static final double startPoseX = 0;
    private static final double startPoseY = 0;
    private static final double startPoseHeading = 0;
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
                                new RightHighJunctionCommand(drivetrain, slide, arm, claw, turnServo),
                                /***Cone 3***/
                                new PickConeCommand(slide, claw),
                                new DriveForwardCommand(drivetrain, 51),

//                                new ParallelCommandGroup(
//                                        new SlideHighFCommand(slide, arm, claw, true),
//                                        new DriveForwardCommand(drivetrain, 29.3)
//                                ),
//                                new TurnCommand(drivetrain, -52.47),//oprg:300 to -60
//                                new DriveForwardCommand(drivetrain, 5.95),
//                                new DropAutoConeCommand(claw, slide, arm,true),




                                //Parking
                                new ParallelCommandGroup(
                                        new SlideResetUpAutonCommand(slide, arm, claw, turnServo),
                                        new TurnToCommand(drivetrain, 270, true)
                                )
//                                new DriveForwardCommand(drivetrain, -4),
//
//                                new StrafeLeftCommand(drivetrain, 22.5)
                        )
                );
                return;
            }
            case 2: { //Mid
                schedule(
                        new SequentialCommandGroup(
                                new RightHighJunctionCommand(drivetrain, slide, arm, claw, turnServo),
                                /***Cone 3***/
                                new PickConeCommand(slide, claw),
                                new ParallelCommandGroup(
                                        new SlideHighFCommand(slide, arm, claw, turnServo, true),
                                        new DriveForwardCommand(drivetrain, 29.5)
                                ),
                                new TurnCommand(drivetrain, -54.8),//oprg:300 to -60
                                new DriveForwardCommand(drivetrain, 6),
                                new DropAutoConeCommand(claw, slide, arm,true),




                                //Parking
                                new ParallelCommandGroup(
                                        new SlideResetUpAutonCommand(slide, arm, claw, turnServo),
                                        new TurnToCommand(drivetrain, 270)
                                ),
                                new DriveForwardCommand(drivetrain, -4),

                                new StrafeRightCommand(drivetrain, 12.7)
                        )
                );
                return;
            }
            default: { //High
                schedule(
                        new SequentialCommandGroup(
                                new RightHighJunctionCommand(drivetrain, slide, arm, claw, turnServo),
                                new PickConeCommand(slide, claw),
                                new WaitCommand(1000),

//                                /***Cone 3***/
//                                new PickConeCommand(slide, claw),
//                                new ParallelCommandGroup(
//                                        new SlideHighFCommand(slide, arm, claw, true),
//                                        new DriveForwardCommand(drivetrain, 29.3)
//                                ),
//                                new TurnCommand(drivetrain, -53.5),//oprg:300 to -60
//                                new DriveForwardCommand(drivetrain, 5.95),
//                                new DropAutoConeCommand(claw, slide, arm,true),




                                //Parking
                                new ParallelCommandGroup(
                                        new SlideResetUpAutonCommand(slide, arm, claw, turnServo),
                                        new TurnToCommand(drivetrain, 270, true)
                                )
//                                new DriveForwardCommand(drivetrain, -4),
//                                new StrafeRightCommand(drivetrain, 41)
                        )
                );
                return;
            }
        }
    }
}