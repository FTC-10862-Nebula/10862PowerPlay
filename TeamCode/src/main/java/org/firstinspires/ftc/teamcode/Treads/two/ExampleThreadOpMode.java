package org.firstinspires.ftc.teamcode.Treads.two;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.DriveForwardCommand;
import org.firstinspires.ftc.teamcode.driveTrainAuton.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
@Autonomous
//Extend ThreadOpMode rather than OpMode
public class ExampleThreadOpMode extends ThreadOpMode {
    private static double startPoseX = 0;
    private static double startPoseY = 0;
    private static double startPoseHeading = 0;

    //Define global variables
    private Drivetrain drivetrain;

    @Override
    public void mainInit() {
        //Perform your normal init

        drivetrain = new Drivetrain(new SampleMecanumDrive(hardwareMap), telemetry, hardwareMap);
        drivetrain.init();
        drivetrain.setPoseEstimate(new Pose2d(startPoseX, startPoseY, Math.toRadians(startPoseHeading)));

        //Below is a new thread
        registerThread(
                new TaskThread(
                        new TaskThread.Actions() {
                            @Override
                            public void loop() {
                                //The loop method should contain what to constantly run in the thread
                                //For instance, this drives a single DcMotor
                                new DriveForwardCommand(drivetrain, 10);
                            }
                        }
                )
        );
    }

    @Override
    public void mainLoop() {
        //Anything you want to constantly run in the MAIN thread goes here
    }

    public void matchStart(){

//        schedule{
//
//        }
    }
}