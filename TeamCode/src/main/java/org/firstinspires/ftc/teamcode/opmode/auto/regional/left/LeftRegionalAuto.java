package org.firstinspires.ftc.teamcode.opmode.auto.regional.left;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.arm.backside.auto.cone.ArmCone3BackCommand;
import org.firstinspires.ftc.teamcode.commands.arm.backside.auto.cone.ArmCone4BackCommand;
import org.firstinspires.ftc.teamcode.commands.arm.backside.auto.cone.ArmCone5BackCommand;
import org.firstinspires.ftc.teamcode.commands.arm.frontside.ArmHighFrontCommand;
import org.firstinspires.ftc.teamcode.commands.arm.frontside.ArmMidFrontCommand;
import org.firstinspires.ftc.teamcode.commands.arm.intake.AutoPickConeCommand;
import org.firstinspires.ftc.teamcode.commands.arm.outtake.AutoDropConeCommand;
import org.firstinspires.ftc.teamcode.commands.arm.slide.SlideResetUpAutonCommand;
import org.firstinspires.ftc.teamcode.commands.drive.trajectory.sequence.DisplacementCommand;
import org.firstinspires.ftc.teamcode.commands.drive.trajectory.sequence.TrajectorySequenceContainerFollowCommand;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Pivot;
import org.firstinspires.ftc.teamcode.subsystems.Slide;
import org.firstinspires.ftc.teamcode.subsystems.TurnServo;
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.misc.TagVision;
import org.firstinspires.ftc.teamcode.trajectorysequence.container.Back;
import org.firstinspires.ftc.teamcode.trajectorysequence.container.Forward;
import org.firstinspires.ftc.teamcode.trajectorysequence.container.Pose2dContainer;
import org.firstinspires.ftc.teamcode.trajectorysequence.container.SetReversed;
import org.firstinspires.ftc.teamcode.trajectorysequence.container.SplineTo;
import org.firstinspires.ftc.teamcode.trajectorysequence.container.SplineToSplineHeading;
import org.firstinspires.ftc.teamcode.trajectorysequence.container.StrafeLeft;
import org.firstinspires.ftc.teamcode.trajectorysequence.container.StrafeRight;
import org.firstinspires.ftc.teamcode.trajectorysequence.container.TrajectorySequenceConstraints;
import org.firstinspires.ftc.teamcode.trajectorysequence.container.TrajectorySequenceContainer;
import org.firstinspires.ftc.teamcode.trajectorysequence.container.Turn;
import org.firstinspires.ftc.teamcode.util.MatchOpMode;
import org.firstinspires.ftc.teamcode.util.PoseStorage;
//438247

@Autonomous
public class LeftRegionalAuto extends MatchOpMode {
    // Subsystems
    private Pivot pivot;
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
                            if (s > 18) {
                                return baseVel * 0.4;
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
                            if (s > 20) {
                                return baseVel * 0.65;
                            } else {
                                return baseVel;
                            }

                        },
                        (s, a, b, c) -> baseAccel,
                        turnVel,
                        turnAccel
                );
            }
            static TrajectorySequenceConstraints getPreLoadDropConstraints() {
                return new TrajectorySequenceConstraints(
                        (s, a, b, c) -> {
                            if (s > 48) {
                                return baseVel * 0.6;
                            } else {
                                return baseVel;
                            }

                        },
                        (s, a, b, c) -> baseAccel,
                        turnVel,
                        turnAccel
                );
            }
            static TrajectorySequenceConstraints getParkConstraint() {
                return new TrajectorySequenceConstraints(
                        (s, a, b, c) -> {
                            return baseVel * 0.6;
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
            public static PreLoad apreLoad;
            public static class PreLoad {
                public static Pose2dContainer startPose = new Pose2dContainer(-34.9, -65, 90);
                public static Forward a = new Forward(34);
                public static SplineTo b = new SplineTo(-28.5, -2.8, 50);
                static TrajectorySequenceContainer preload = new TrajectorySequenceContainer(Speed::getPreLoadDropConstraints, a, b);
            }

            public static Cycle1Pickup bcycle1Pickup;
            public static class Cycle1Pickup {
                public static SetReversed a = new SetReversed(true);
                public static SplineTo b = new SplineTo(-48, -8.7,180);
                public static Back c = new Back(13.75);
                static TrajectorySequenceContainer cycle1Pickup = new TrajectorySequenceContainer(Speed::getPickupConstraints, a, b, c);
            }

            public static Cycle1Drop ccycle1Drop;
            public static class Cycle1Drop {
                public static SetReversed a = new SetReversed(true);
                public static Forward b = new Forward(16);
                public static SplineTo c = new SplineTo(-30.9, -13.2, -45);
                static TrajectorySequenceContainer cycle1Drop = new TrajectorySequenceContainer(Speed::getDropConstraints, a, b, c);
            }

            public static Cycle2Pickup dcycle2Pickup;
            public static class Cycle2Pickup {
                public static SetReversed a = new SetReversed(true);
                public static SplineTo b = new SplineTo(-49.1, -8.6, 180);
                public static Back c = new Back(12.1);
                static TrajectorySequenceContainer cycle2Pickup = new TrajectorySequenceContainer(Speed::getPickupConstraints, a, b, c);
            }

            public static Cycle2Drop ecycle2Drop;
            public static class Cycle2Drop {
                public static SetReversed a = new SetReversed(true);
                public static Forward b = new Forward(16);
                public static SplineTo c = new SplineTo(-31, -13.3, -45);
                static TrajectorySequenceContainer cycle2Drop = new TrajectorySequenceContainer(Speed::getDropConstraints, a, b, c);
            }

            public static Cycle3Pickup fcycle3Pickup;
            public static class Cycle3Pickup {
                public static SetReversed a = new SetReversed(true);
                public static SplineTo b = new SplineTo(-51.5, -8.5, 180);
                public static Back c = new Back(11.36);
                static TrajectorySequenceContainer cycle3Pickup = new TrajectorySequenceContainer(Speed::getPickupConstraints, a, b, c);
            }

            public static Cycle3Drop gcycle3Drop;
            public static class Cycle3Drop {
                public static SetReversed a = new SetReversed(true);
                public static Forward b = new Forward(16);
                public static SplineTo c = new SplineTo(-30.95, -12.7, -45);
                static TrajectorySequenceContainer cycle3Drop = new TrajectorySequenceContainer(Speed::getDropConstraints, a, b, c);
            }

            public static Park jpark;
            public static class Park {
                public static double leftX = -7;
                public static double midX = -36;
                public static double rightX = -64;
                public static double y = -15.2;
                public static double heading = 90;
                public static double endHeading = 0;
                public enum AutoPosition {
                    lEFT,
                    MID,
                    RIGHT
                }
                public static AutoPosition autoPosition = AutoPosition.MID;
                static TrajectorySequenceContainer getPark(double x) {
                    switch (autoPosition) {
                        case lEFT:
                            return new TrajectorySequenceContainer(
                                    Speed::getParkConstraint,
                                    new Back(4),
                                    new Turn(-45),

                                    new Back(2),
                                    new StrafeRight(32.5),
                                    new Forward(7)
                            );

                        case MID:
                            return new TrajectorySequenceContainer(
                                    Speed::getParkConstraint,
                                    new Back(6),
                                    new Turn(-45),
                                    new Forward(7)
                            );
                        default:
                        case RIGHT:
                            return new TrajectorySequenceContainer(
                                    Speed::getParkConstraint,
                                    new Back(6),
                                    new Turn(-45),

                                    new Back(2),
                                    new StrafeLeft(29.5),
                                    new Forward(7)
                            );
                    }
                }
            }
        }
    }



    @Override
    public void robotInit() {
        claw = new Claw(telemetry, hardwareMap);
        pivot = new Pivot(telemetry, hardwareMap);
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
//                finalX = RightRegionalAutoConstants.Path.Park.leftX;
                RightRegionalAutoConstants.Path.Park.autoPosition = RightRegionalAutoConstants.Path.Park.AutoPosition.lEFT;
                break;
            case 2:
//                finalX = RightRegionalAutoConstants.Path.Park.midX;
                RightRegionalAutoConstants.Path.Park.autoPosition = RightRegionalAutoConstants.Path.Park.AutoPosition.MID;
                break;
            case 3:
//                finalX = RightRegionalAutoConstants.Path.Park.rightX;
                RightRegionalAutoConstants.Path.Park.autoPosition = RightRegionalAutoConstants.Path.Park.AutoPosition.RIGHT;
                break;
        }
        drivetrain.setPoseEstimate(RightRegionalAutoConstants.Path.PreLoad.startPose.getPose());
        PoseStorage.trajectoryPose = RightRegionalAutoConstants.Path.PreLoad.startPose.getPose();
        schedule(
                new SequentialCommandGroup(
                        /* Preload */
                        new ParallelCommandGroup(
                                new TrajectorySequenceContainerFollowCommand(drivetrain, RightRegionalAutoConstants.Path.PreLoad.preload),
                                new ArmHighFrontCommand(slide, pivot, claw, turnServo, true)
                        ),
                        new WaitCommand(100),
                        new AutoDropConeCommand(claw, slide, pivot, true),



                        /* Cycle 1 Pickup */
                        new ParallelCommandGroup(
                                new ArmCone5BackCommand(slide, claw, pivot, turnServo),
                                new TrajectorySequenceContainerFollowCommand(drivetrain, RightRegionalAutoConstants.Path.Cycle1Pickup.cycle1Pickup,
                                        new DisplacementCommand(30, new AutoPickConeCommand(slide, claw)))
                        ),
//                        new ParallelCommandGroup(
//                                new InstantCommand(claw::clawClose),
//                                new DriveForwardCommand(drivetrain, 0.1)
//                        ),
//                        new AutoPickConeCommand(slide, claw),



                        /* Cycle 1 Drop */
                        new ParallelCommandGroup(
                                new ArmMidFrontCommand(slide, pivot, claw, turnServo,true),
                                new TrajectorySequenceContainerFollowCommand(drivetrain, RightRegionalAutoConstants.Path.Cycle1Drop.cycle1Drop)
                        ),
                        new AutoDropConeCommand(claw, slide, pivot, true),

                        /* Cycle 2 Pickup */
                        new ParallelCommandGroup(
                                new ArmCone4BackCommand(slide, claw, pivot, turnServo),
                                new TrajectorySequenceContainerFollowCommand(drivetrain, RightRegionalAutoConstants.Path.Cycle2Pickup.cycle2Pickup,
                                        new DisplacementCommand(28.5, new AutoPickConeCommand(slide, claw)))
                        ),
//                        new AutoPickConeCommand(slide, claw),

                        /* Cycle 2 Drop */
                        new ParallelCommandGroup(
                                new ArmMidFrontCommand(slide, pivot, claw, turnServo,true),
                                new TrajectorySequenceContainerFollowCommand(drivetrain, RightRegionalAutoConstants.Path.Cycle2Drop.cycle2Drop)
                        ),
                        new AutoDropConeCommand(claw, slide, pivot, true),

                        /* Cycle 3 Pickup */
                        new ParallelCommandGroup(
                                new ArmCone3BackCommand(slide, claw, pivot, turnServo),
                                new TrajectorySequenceContainerFollowCommand(drivetrain, RightRegionalAutoConstants.Path.Cycle3Pickup.cycle3Pickup,
                                        new DisplacementCommand(28, new AutoPickConeCommand(slide, claw)))
                        ),
//                        new AutoPickConeCommand(slide, claw),

                        /* Cycle 3 Drop */
                        new ParallelCommandGroup(
                                new ArmMidFrontCommand(slide, pivot, claw, turnServo,true),
                                new TrajectorySequenceContainerFollowCommand(drivetrain, RightRegionalAutoConstants.Path.Cycle3Drop.cycle3Drop)
                        ),
                        new AutoDropConeCommand(claw, slide, pivot, true),


//                        /* Cycle 4 Pickup */
//                        new ParallelCommandGroup(
//                                new ArmCone2BackCommand(slide, claw, pivot, turnServo),
//                                new TrajectorySequenceContainerFollowCommand(drivetrain, RightRegionalAutoConstants.Path.Cycle4Pickup.cycle4Pickup,
//                                        new DisplacementCommand(27, new AutoPickConeCommand(slide, claw)))
//                        ),
////                        new AutoPickConeCommand(slide, claw),
//
//                        /* Cycle 4 Drop */
//                        new ParallelCommandGroup(
//                                new ArmHighFrontCommand(slide, pivot, claw, turnServo,true),
////                                new ArmMidFrontCommand(slide, pivot, claw, turnServo,true),
//                                new TrajectorySequenceContainerFollowCommand(drivetrain, RightRegionalAutoConstants.Path.Cycle4Drop.cycle4Drop)
//                        ),
//                        new AutoDropConeCommand(claw, slide, pivot, true),

                        /* Park */
                        new ParallelCommandGroup(
                                new TrajectorySequenceContainerFollowCommand(drivetrain, RightRegionalAutoConstants.Path.Park.getPark(finalX))
                        ),
                        new SequentialCommandGroup(
                                new SlideResetUpAutonCommand(slide, pivot, claw, turnServo)
//                                new WaitCommand(1000),
//                                run(pivot::stopArm)
                        ),
                        run(() -> PoseStorage.currentPose = drivetrain.getPoseEstimate()),

                        /* Save Pose and end opmode*/

                        run(this::stop)
                )
        );
    }
}