package org.firstinspires.ftc.teamcode.autons.Misc;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.DriveForwardCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.SplineCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.SplinetoLinearCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.SplinetoSplineCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.TurnToCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.ResetPoseCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeAndDropConeCommands.DropAutoConeCommand;
import org.firstinspires.ftc.teamcode.commands.MuliplyCommand;
import org.firstinspires.ftc.teamcode.commands.PickConeAutoCommands.PrePickBConeCommands.PrePickB5Command;
import org.firstinspires.ftc.teamcode.commands.Slide.SlideFCommands.SlideHighFCommand;
import org.firstinspires.ftc.teamcode.driveTrainAuton.MatchOpMode;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.ClawServos;
import org.firstinspires.ftc.teamcode.subsystems.Slide;
import org.firstinspires.ftc.teamcode.util.PoseStorage;

@Autonomous(group = "RED/BLUE")
public class TestAutonWithoutCam extends MatchOpMode {

    // Subsystems
    private Arm arm;
    private ClawServos clawServos;
    private Drivetrain drivetrain;
    private Slide slide;
//    public MecanumDrive mecanumDrive;


    @Override
    public void robotInit() {
        clawServos = new ClawServos( telemetry, hardwareMap);
        arm = new Arm(telemetry, hardwareMap);
        slide = new Slide(telemetry, hardwareMap);
//        mecanumDrive = new MecanumDrive(hardwareMap, telemetry, false);
        drivetrain = new Drivetrain(new MecanumDrive(hardwareMap, telemetry, false), telemetry, hardwareMap);
        drivetrain.init();
        //        mecanumDrive.init();
//        drivetrain.setPoseEstimate(new Pose2d(startPoseX, startPoseY, Math.toRadians(startPoseHeading)));
    }


    public void matchStart() {
//        waitForStart();
        schedule(
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new SlideHighFCommand(slide, arm, clawServos, true),
                                new SplineCommand(drivetrain, new Vector2d(58, 8.5), Math.toRadians(23))    //Cycle
                        ),
                        new DropAutoConeCommand(clawServos, slide, arm, true),
                        new ParallelCommandGroup(
                                new PrePickB5Command(slide, clawServos, arm),
                                new SplineCommand(drivetrain, new Vector2d(55.8, -25), Math.toRadians(270), true)   //Load
                        ),
                        new MuliplyCommand(4,
                                new ParallelCommandGroup(
                                        new SlideHighFCommand(slide, arm, clawServos, true),
                                        new SplineCommand(drivetrain, new Vector2d(58, 8.5), Math.toRadians(23), PoseStorage.cycle)    //Cycle
                                ),
                                new DropAutoConeCommand(clawServos, slide, arm, true),
                                new ParallelCommandGroup(
                                        new PrePickB5Command(slide, clawServos, arm),
                                        new SplineCommand(drivetrain, new Vector2d(55.8, -25), Math.toRadians(270), PoseStorage.load, true)   //Load
                                )
                        ),


                        /**Cone 1**/
                        new ParallelCommandGroup(
                                new SlideHighFCommand(slide, arm, clawServos, true),
                                new SplineCommand(drivetrain, new Vector2d(58, 8.5), Math.toRadians(23), PoseStorage.cycle)    //Cycle
                        ),
                        new DropAutoConeCommand(clawServos, slide, arm, true),

                        new TurnToCommand(drivetrain, 0)
                )
        );
//        PoseStorage.currentPose = drivetrain.getPoseEstimate();
    }


};