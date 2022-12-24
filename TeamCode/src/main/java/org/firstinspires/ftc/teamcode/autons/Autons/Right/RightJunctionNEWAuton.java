package org.firstinspires.ftc.teamcode.autons.Autons.Right;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autons.AutonCommands.RightHighJunctionCommandNew;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.DriveForwardCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.StrafeRightCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.TurnCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.TurnToCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeAndDropConeCommands.DropAutoConeCommand;
import org.firstinspires.ftc.teamcode.commands.PickConeAutoCommands.Pick.PickCBCommand;
import org.firstinspires.ftc.teamcode.commands.Slide.SlideFCommands.SlideHighFCommand;
import org.firstinspires.ftc.teamcode.commands.Slide.SlideResetUpAutonCommand;
import org.firstinspires.ftc.teamcode.driveTrainAuton.MatchOpMode;
import org.firstinspires.ftc.teamcode.driveTrainAuton.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.ClawServos;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Slide;
import org.firstinspires.ftc.teamcode.subsystems.Vision.TagVision;

@Autonomous(name="rightNEW")
public class RightJunctionNEWAuton extends MatchOpMode {
//    private ATDetector tagDetector;

    private static final double startPoseX = 0;
    private static final double startPoseY = 0;
    private static final double startPoseHeading = 0;
    private int tagNum = 0;

    //Gamepad
//    private GamepadEx driverGamepad;

    // Subsystems
    private Arm arm;
    private ClawServos clawServos;
    private Drivetrain drivetrain;
    private Slide slide;
    private TagVision tagVision;

    @Override
    public void robotInit() {
        clawServos = new ClawServos( telemetry, hardwareMap);
        arm = new Arm( telemetry, hardwareMap);
        drivetrain = new Drivetrain(new SampleMecanumDrive(hardwareMap), telemetry, hardwareMap);
        drivetrain.init();
        slide = new Slide(telemetry, hardwareMap);
        drivetrain.setPoseEstimate(new Pose2d(startPoseX, startPoseY, Math.toRadians(startPoseHeading)));

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
                                new RightHighJunctionCommandNew(drivetrain, slide, arm, clawServos),
                                /***Cone 3***/
                                new PickCBCommand(slide, clawServos),
                                new DriveForwardCommand(drivetrain, 51),

//                                new ParallelCommandGroup(
//                                        new SlideHighFCommand(slide, arm, clawServos, true),
//                                        new DriveForwardCommand(drivetrain, 29.3)
//                                ),
//                                new TurnCommand(drivetrain, -52.47),//oprg:300 to -60
//                                new DriveForwardCommand(drivetrain, 5.95),
//                                new DropAutoConeCommand(clawServos, slide, arm,true),




                                //Parking
                                new ParallelCommandGroup(
                                        new SlideResetUpAutonCommand(slide, arm, clawServos),
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
                                new RightHighJunctionCommandNew(drivetrain, slide, arm, clawServos),
                                /***Cone 3***/
                                new PickCBCommand(slide, clawServos),
                                new ParallelCommandGroup(
                                        new SlideHighFCommand(slide, arm, clawServos, true),
                                        new DriveForwardCommand(drivetrain, 29.5)
                                ),
                                new TurnCommand(drivetrain, -54.8),//oprg:300 to -60
                                new DriveForwardCommand(drivetrain, 6),
                                new DropAutoConeCommand(clawServos, slide, arm,true),




                                //Parking
                                new ParallelCommandGroup(
                                        new SlideResetUpAutonCommand(slide, arm, clawServos),
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
                                new RightHighJunctionCommandNew(drivetrain, slide, arm, clawServos),
                                new PickCBCommand(slide, clawServos),
                                new WaitCommand(1000),

//                                /***Cone 3***/
//                                new PickCBCommand(slide, clawServos),
//                                new ParallelCommandGroup(
//                                        new SlideHighFCommand(slide, arm, clawServos, true),
//                                        new DriveForwardCommand(drivetrain, 29.3)
//                                ),
//                                new TurnCommand(drivetrain, -53.5),//oprg:300 to -60
//                                new DriveForwardCommand(drivetrain, 5.95),
//                                new DropAutoConeCommand(clawServos, slide, arm,true),




                                //Parking
                                new ParallelCommandGroup(
                                        new SlideResetUpAutonCommand(slide, arm, clawServos),
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