package org.firstinspires.ftc.teamcode.autons.Misc;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autons.AutonCommands.LeftRedSpline;
import org.firstinspires.ftc.teamcode.driveTrainAuton.MatchOpMode;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.SensorColor;
import org.firstinspires.ftc.teamcode.subsystems.Slide;
import org.firstinspires.ftc.teamcode.subsystems.TurnServo;

@Autonomous(group = "RED/BLUE")
public class TestAutonWithoutCam extends MatchOpMode {

    // Subsystems
    private Arm arm;
    private Claw claw;
    private Drivetrain drivetrain;
    private Slide slide;
    private TurnServo turnServo;
    private SensorColor sensorColor;

//    public MecanumDrive mecanumDrive;


    @Override
    public void robotInit() {
        claw = new Claw( telemetry, hardwareMap);
        arm = new Arm(telemetry, hardwareMap);
        slide = new Slide(telemetry, hardwareMap);
        turnServo = new TurnServo(telemetry, hardwareMap);
        sensorColor = new SensorColor(hardwareMap, telemetry);
        drivetrain = new Drivetrain(new MecanumDrive(hardwareMap, telemetry, false), telemetry, hardwareMap);
        drivetrain.init();
    }


    public void matchStart() {
//        waitForStart();
        schedule(
                new SequentialCommandGroup(
//                        new RightSpline(drivetrain, slide, arm, claw)
                      new LeftRedSpline(drivetrain, slide, arm, claw, turnServo, sensorColor)
//                        new Test(drivetrain, slide, arm, claw, turnServo)

                      /* new ParallelCommandGroup(
                                new DriveForwardCommand(drivetrain, 50),
                                new SlideHighFCommand(slide, arm, claw, turnServo, true)
                        ),
                        new TurnToCommand(drivetrain, 330, true),
                        new DriveForwardCommand(drivetrain, 8),

                        new DropAutoConeCommand(claw, slide, arm, true),
                        new DriveForwardCommand(drivetrain, -8),

                        new TurnToCommand(drivetrain, 90),
                        new ParallelCommandGroup(
                                new DriveForwardCommand(drivetrain, -10),
                                new PrePickB5Command(slide,claw,arm, turnServo)
                        ),
                        new PickConeCommand(claw, slide, arm),


                        new ParallelCommandGroup(
                                new TurnCommand(drivetrain, -55.3),
                                new SlideLowFCommand(slide, arm, claw, turnServo, true)
                        ),
                        new ParallelCommandGroup(
                                new SlowDriveForwardCommand(drivetrain, 3),
                                new DropAutoConeCommand(claw, slide, arm,true)
                        ),

                        new ParallelCommandGroup(
                                new SlowDriveForwardCommand(drivetrain, -3),
                                new PrePick5FCommand(slide, claw, arm, turnServo)
                        ),
                        new ParallelCommandGroup(
                                new TurnToCommand(drivetrain, 0, true),
                                new PrePickB4Command(slide,claw, arm, turnServo)
                        )*/
                )
        );
//        PoseStorage.currentPose = drivetrain.getPoseEstimate();
    }


};