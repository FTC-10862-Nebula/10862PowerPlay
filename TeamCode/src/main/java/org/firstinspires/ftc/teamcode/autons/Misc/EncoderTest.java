package org.firstinspires.ftc.teamcode.autons.Misc;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.commands.Slide.SlideBackCommands.SlideHighBCommand;
import org.firstinspires.ftc.teamcode.commands.Slide.SlideBackCommands.SlideMidBCommand;
import org.firstinspires.ftc.teamcode.commands.Slide.SlideFCommands.SlideResetFCommandT;
import org.firstinspires.ftc.teamcode.driveTrainAuton.MatchOpMode;
import org.firstinspires.ftc.teamcode.driveTrainAuton.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.ClawServos;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Slide;
import org.firstinspires.ftc.teamcode.util.Encoder;

@Autonomous(group = "RED/BLUE")
public class EncoderTest extends MatchOpMode {
//    private ATDetector tagDetector;

    // Subsystems
    private Encoder leftEncoder, rightEncoder, frontEncoder;
    private int turn =0;
    private boolean bool = true;


    @Override
    public void robotInit() {
        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftFront"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightRear"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftRear"));


    }

    public void matchStart() {
//        waitForStart();
        schedule(
        );
        while(bool){
            periodic();
            turn++;
            if(turn<1000000000){
                bool = false;
            }
        }
//        PoseStorage.currentPose = drivetrain.getPoseEstimate();
    }


    public void periodic(){
//        while(bool)
//        {
            telemetry.addData("Front Encoder Velocity: ", frontEncoder.getCorrectedVelocity());
            telemetry.addData("Left Encoder Velocity: ", leftEncoder.getCorrectedVelocity());
            telemetry.addData("Right Encoder Velocity: ", rightEncoder.getCorrectedVelocity());
            telemetry.addLine("_____________________________________");
            telemetry.addData("Front Encoder Pos: ", frontEncoder.getCurrentPosition());
            telemetry.addData("Left Encoder Pos: ", leftEncoder.getCurrentPosition());
            telemetry.addData("Right Encoder Pos: ", rightEncoder.getCurrentPosition());
//        }

    }

};