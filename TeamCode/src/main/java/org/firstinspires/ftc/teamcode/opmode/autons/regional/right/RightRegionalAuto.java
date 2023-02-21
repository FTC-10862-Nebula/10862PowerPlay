package org.firstinspires.ftc.teamcode.opmode.autons.regional.right;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.Drive.AutoCommands.DisplacementCommand;
import org.firstinspires.ftc.teamcode.commands.Drive.AutoCommands.MarkerCommand;
import org.firstinspires.ftc.teamcode.commands.Drive.AutoCommands.TrajectorySequenceContainerFollowCommand;
import org.firstinspires.ftc.teamcode.commands.slide.slideFCommands.SlideHighFCommand;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Slide;
import org.firstinspires.ftc.teamcode.subsystems.TurnServo;
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drive.MatchOpMode;
import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.misc.TagVision;
import org.firstinspires.ftc.teamcode.trajectorysequence.container.Back;
import org.firstinspires.ftc.teamcode.trajectorysequence.container.Forward;
import org.firstinspires.ftc.teamcode.trajectorysequence.container.Pose2dContainer;
import org.firstinspires.ftc.teamcode.trajectorysequence.container.SetReversed;
import org.firstinspires.ftc.teamcode.trajectorysequence.container.SplineTo;
import org.firstinspires.ftc.teamcode.trajectorysequence.container.TrajectorySequenceConstraints;
import org.firstinspires.ftc.teamcode.trajectorysequence.container.TrajectorySequenceContainer;
import org.firstinspires.ftc.teamcode.util.PoseStorage;

@Autonomous
public class RightRegionalAuto extends MatchOpMode {
    // Subsystems
    private Arm arm;
    private Claw claw;
    private Drivetrain drivetrain;
    private Slide slide;
    private TagVision tagVision;
    private TurnServo turnServo;
    @Config
    public static class LeftRegionalAutoConstants {

        public static class Speed {
            public static double baseVel = DriveConstants.MAX_VEL; // value
            public static double baseAccel = DriveConstants.MAX_ACCEL; // value
            public static double turnVel = DriveConstants.MAX_VEL; // value
            public static double turnAccel = DriveConstants.MAX_ANG_ACCEL; // value
            public static double slowVel = baseVel * 0.8; // value
            public static double slowAccel = baseAccel * 0.8; // value
            public static double slowTurnVel = turnVel * 0.8; // value
            public static double slowTurnAccel = turnAccel * 0.8; // value
            static TrajectorySequenceConstraints getBaseConstraints() {
                return new TrajectorySequenceConstraints(baseVel, baseAccel, turnVel, turnAccel);
            }
            static TrajectorySequenceConstraints getSlowConstraints() {
                return new TrajectorySequenceConstraints(slowVel, slowAccel, slowTurnVel, slowTurnAccel);
            }
        }
        public static class Path {
            public static class PreLoad {
                public static Pose2dContainer startPose = new Pose2dContainer(35, -65, 90);
                public static Forward a = new Forward(36);
                public static SplineTo b = new SplineTo(30, -8, 135);
                static TrajectorySequenceContainer preload = new TrajectorySequenceContainer(Speed::getBaseConstraints, a, b);
            }

            public static class Cycle1Pickup {
                public static SetReversed a = new SetReversed(true);
                public static SplineTo b = new SplineTo(42, -12, 0);
                public static Back c = new Back(20);
                static TrajectorySequenceContainer cycle1Pickup = new TrajectorySequenceContainer(Speed::getBaseConstraints, a, b, c);
            }

            public static class Cycle1Drop {
                public static SetReversed a = new SetReversed(true);
                public static Forward b = new Forward(20);
                public static SplineTo c = new SplineTo(30, -16, -135);
                static TrajectorySequenceContainer cycle1Pickup = new TrajectorySequenceContainer(Speed::getBaseConstraints, a, b, c);
            }

            public static class Cycle2Pickup {
                public static SetReversed a = new SetReversed(true);
                public static SplineTo b = new SplineTo(42, -12, 0);
                public static Back c = new Back(20);
                static TrajectorySequenceContainer cycle1Pickup = new TrajectorySequenceContainer(Speed::getBaseConstraints, a, b, c);
            }

            public static class Cycle2Drop {
                public static SetReversed a = new SetReversed(true);
                public static Forward b = new Forward(20);
                public static SplineTo c = new SplineTo(30, -16, -135);
                static TrajectorySequenceContainer cycle1Pickup = new TrajectorySequenceContainer(Speed::getBaseConstraints, a, b, c);
            }

            public static class Park {
                public static double leftX = 12;
                public static double midX = 24;
                public static double rightX = 36;
            }
        }
    }



    @Override
    public void robotInit() {
        claw = new Claw(telemetry, hardwareMap);
        arm = new Arm(telemetry, hardwareMap);
        drivetrain = new Drivetrain(new MecanumDrive(hardwareMap, telemetry, false), telemetry, hardwareMap);
        drivetrain.init();
        slide = new Slide( telemetry, hardwareMap);
        turnServo = new TurnServo(telemetry, hardwareMap);
        tagVision = new TagVision(hardwareMap, telemetry);
        while (!isStarted() && !isStopRequested()) {
            tagVision.updateTagOfInterest();
            tagVision.tagToTelemetry();
            telemetry.update();
        }
        this.matchStart();
    }

    public void matchStart() {
        double finalX = 0;
        switch (tagVision.getTag()) {
            case 1:
                finalX = LeftRegionalAutoConstants.Path.Park.leftX;
                break;
            case 2:
                finalX = LeftRegionalAutoConstants.Path.Park.midX;
                break;
            case 3:
                finalX = LeftRegionalAutoConstants.Path.Park.rightX;
                break;
        }
        schedule(
                new SequentialCommandGroup(
                        run(() -> drivetrain.setPoseEstimate(LeftRegionalAutoConstants.Path.PreLoad.startPose.getPose())),
                        run(claw::clawClose),
                        run(slide::slideAutoHigh),
                        run(arm::moveHighFAuto),
                        new TrajectorySequenceContainerFollowCommand(drivetrain, LeftRegionalAutoConstants.Path.PreLoad.preload,
                                new DisplacementCommand(12, run(turnServo::setFClawPos))),
                        run(() -> PoseStorage.currentPose = drivetrain.getPoseEstimate())
                )
        );
    }
}