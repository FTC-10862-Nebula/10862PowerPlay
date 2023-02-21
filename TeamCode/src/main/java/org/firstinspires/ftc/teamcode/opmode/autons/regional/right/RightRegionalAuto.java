package org.firstinspires.ftc.teamcode.opmode.autons.regional.right;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.driveCommands.autoCommands.trajectoryCommands.TrajectorySequenceContainerFollowCommand;
import org.firstinspires.ftc.teamcode.commands.auto.autoConeCommands.PickConeCommand;
import org.firstinspires.ftc.teamcode.commands.auto.autoConeCommands.prePickB.PrePickB4Command;
import org.firstinspires.ftc.teamcode.commands.auto.autoConeCommands.prePickB.PrePickB5Command;
import org.firstinspires.ftc.teamcode.commands.intakeAndOutake.DropAutoConeCommand;
import org.firstinspires.ftc.teamcode.commands.slide.SlideResetUpAutonCommand;
import org.firstinspires.ftc.teamcode.commands.slide.slideFCommands.SlideHighFCommand;
import org.firstinspires.ftc.teamcode.commands.slide.slideFCommands.SlideMidFCommand;
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
import org.firstinspires.ftc.teamcode.trajectorysequence.container.LineToLinearHeading;
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
    public static class RightRegionalAutoConstants {

        public static Speed speed;
        public static class Speed {
            public static double baseVel = DriveConstants.MAX_VEL; // value
            public static double baseAccel = DriveConstants.MAX_ACCEL; // value
            public static double turnVel = DriveConstants.MAX_VEL; // value
            public static double turnAccel = DriveConstants.MAX_ANG_ACCEL; // value
            public static double slowVel = baseVel * 0.8; // value
            public static double slowAccel = baseAccel * 0.8; // value
            public static double slowTurnVel = turnVel * 0.8; // value
            public static double slowTurnAccel = turnAccel * 0.8; // value
            static TrajectorySequenceConstraints getPickupConstraints() {
                return new TrajectorySequenceConstraints(
                        (s, a, b, c) -> {
                            if (s > 24) {
                                return baseVel * 0.5;
                            } else {
                                return baseVel;
                            }

                        },
                        (s, a, b, c) -> baseAccel,
                        turnVel,
                        turnAccel
                );
            }
            static TrajectorySequenceConstraints getDropConstraints() {
                return new TrajectorySequenceConstraints(
                        (s, a, b, c) -> {
                            if (s > 24) {
                                return baseVel * 0.5;
                            } else {
                                return baseVel;
                            }

                        },
                        (s, a, b, c) -> baseAccel,
                        turnVel,
                        turnAccel
                );
            }
            static TrajectorySequenceConstraints getBaseConstraints() {
                return new TrajectorySequenceConstraints(baseVel, baseAccel, turnVel, turnAccel);
            }
            static TrajectorySequenceConstraints getSlowConstraints() {
                return new TrajectorySequenceConstraints(slowVel, slowAccel, slowTurnVel, slowTurnAccel);
            }
        }

        public static Path path;
        public static class Path {
            public static PreLoad preLoad;
            public static class PreLoad {
                public static Pose2dContainer startPose = new Pose2dContainer(35, -65, 90);
                public static Forward a = new Forward(36);
                public static SplineTo b = new SplineTo(27.5, -4, 135);
                static TrajectorySequenceContainer preload = new TrajectorySequenceContainer(Speed::getBaseConstraints, a, b);
            }

            public static Cycle1Pickup cycle1Pickup;
            public static class Cycle1Pickup {
                public static SetReversed a = new SetReversed(true);
                public static SplineTo b = new SplineTo(42, -10, 0);
                public static Back c = new Back(21);
                static TrajectorySequenceContainer cycle1Pickup = new TrajectorySequenceContainer(Speed::getPickupConstraints, a, b, c);
            }

            public static Cycle1Drop cycle1Drop;
            public static class Cycle1Drop {
                public static SetReversed a = new SetReversed(true);
                public static Forward b = new Forward(20);
                public static SplineTo c = new SplineTo(28, -14, -135);
                static TrajectorySequenceContainer cycle1Drop = new TrajectorySequenceContainer(Speed::getBaseConstraints, a, b, c);
            }

            public static Cycle2Pickup cycle2Pickup;
            public static class Cycle2Pickup {
                public static SetReversed a = new SetReversed(true);
                public static SplineTo b = new SplineTo(42, -10, 0);
                public static Back c = new Back(22);
                static TrajectorySequenceContainer cycle2Pickup = new TrajectorySequenceContainer(Speed::getPickupConstraints, a, b, c);
            }

            public static Cycle2Drop cycle2Drop;
            public static class Cycle2Drop {
                public static SetReversed a = new SetReversed(true);
                public static Forward b = new Forward(20);
                public static SplineTo c = new SplineTo(29, -15, -135);
                static TrajectorySequenceContainer cycle2Drop = new TrajectorySequenceContainer(Speed::getBaseConstraints, a, b, c);
            }

            public static Park park;
            public static class Park {
                public static double leftX = 12;
                public static double midX = 24;
                public static double rightX = 36;
                public static double y = -12;
                public static double heading = -90;
                static TrajectorySequenceContainer getPark(double x) {
                    return new TrajectorySequenceContainer(
                            Speed::getBaseConstraints,
                            new LineToLinearHeading(x, y, heading)
                    );
                }
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
                finalX = RightRegionalAutoConstants.Path.Park.leftX;
                break;
            case 2:
                finalX = RightRegionalAutoConstants.Path.Park.midX;
                break;
            case 3:
                finalX = RightRegionalAutoConstants.Path.Park.rightX;
                break;
        }
        drivetrain.setPoseEstimate(RightRegionalAutoConstants.Path.PreLoad.startPose.getPose());
        PoseStorage.trajectoryPose = RightRegionalAutoConstants.Path.PreLoad.startPose.getPose();
        schedule(
                new SequentialCommandGroup(
                        /* Preload */
                        new ParallelCommandGroup(
                                new TrajectorySequenceContainerFollowCommand(drivetrain, RightRegionalAutoConstants.Path.PreLoad.preload),
                                new SlideHighFCommand(slide, arm, claw, turnServo, true)
                        ),
                        new DropAutoConeCommand(claw, slide, arm, true),

                        /* Cycle 1 Pickup */
                        new ParallelCommandGroup(
                                new PrePickB5Command(slide, claw, arm, turnServo),
                                new TrajectorySequenceContainerFollowCommand(drivetrain, RightRegionalAutoConstants.Path.Cycle1Pickup.cycle1Pickup)
                        ),
                        new PickConeCommand(slide, claw),

                        /* Cycle 1 Drop */
                        new ParallelCommandGroup(
                                new SlideMidFCommand(slide, arm, claw, turnServo,true),
                                new TrajectorySequenceContainerFollowCommand(drivetrain, RightRegionalAutoConstants.Path.Cycle1Drop.cycle1Drop)
                        ),
                        new DropAutoConeCommand(claw, slide, arm, true),

                        /* Cycle 2 Pickup */
                        new ParallelCommandGroup(
                                new PrePickB4Command(slide, claw, arm, turnServo),
                                new TrajectorySequenceContainerFollowCommand(drivetrain, RightRegionalAutoConstants.Path.Cycle2Pickup.cycle2Pickup)
                        ),
                        new PickConeCommand(slide, claw),

                        /* Cycle 2 Drop */
                        new ParallelCommandGroup(
                                new SlideMidFCommand(slide, arm, claw, turnServo,true),
                                new TrajectorySequenceContainerFollowCommand(drivetrain, RightRegionalAutoConstants.Path.Cycle2Drop.cycle2Drop)
                        ),
                        new DropAutoConeCommand(claw, slide, arm, true),

                        /* Park */
                        new ParallelCommandGroup(
                                new TrajectorySequenceContainerFollowCommand(drivetrain, RightRegionalAutoConstants.Path.Park.getPark(finalX)),
                                new SlideResetUpAutonCommand(slide, arm, claw, turnServo)
                        ),

                        /* Save Pose and end opmode*/
                        run(() -> PoseStorage.currentPose = drivetrain.getPoseEstimate()),
                        run(this::stop)
                )
        );
    }
}