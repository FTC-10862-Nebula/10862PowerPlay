package org.firstinspires.ftc.teamcode.opmode.auto.regional.left;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.arm.backside.auto.cone.ArmCone2BackCommand;
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
import org.firstinspires.ftc.teamcode.trajectorysequence.container.TrajectorySequenceConstraints;
import org.firstinspires.ftc.teamcode.trajectorysequence.container.TrajectorySequenceContainer;
import org.firstinspires.ftc.teamcode.util.MatchOpMode;
import org.firstinspires.ftc.teamcode.util.PoseStorage;

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
            static TrajectorySequenceConstraints getPreLoadDropConstraints() {
                return new TrajectorySequenceConstraints(
                        (s, a, b, c) -> {
                            if (s > 48) {
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
            static TrajectorySequenceConstraints getParkConstraint() {
                return new TrajectorySequenceConstraints(
                        (s, a, b, c) -> {
                            return baseVel * 0.4;
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
                public static Pose2dContainer startPose = new Pose2dContainer(-35.2, -65, 90);
                public static Forward a = new Forward(36);
                public static SplineTo b = new SplineTo(-29.7, -4.7, 45);
                static TrajectorySequenceContainer preload = new TrajectorySequenceContainer(Speed::getPreLoadDropConstraints, a, b);
            }

            public static Cycle1Pickup bcycle1Pickup;
            public static class Cycle1Pickup {
                public static SetReversed a = new SetReversed(true);
                public static Back b = new Back(16);
                public static SplineTo c = new SplineTo(-47, -8.65, 180);
                static TrajectorySequenceContainer cycle1Pickup = new TrajectorySequenceContainer(Speed::getPickupConstraints, a, b, c);
            }

            public static Cycle1Drop ccycle1Drop;
            public static class Cycle1Drop {
                public static SetReversed a = new SetReversed(true);
                public static Forward b = new Forward(16);
                public static SplineTo c = new SplineTo(-32.8, -14.4, -45);
                static TrajectorySequenceContainer cycle1Drop = new TrajectorySequenceContainer(Speed::getDropConstraints, a, b, c);
            }

            public static Cycle2Pickup dcycle2Pickup;
            public static class Cycle2Pickup {
                public static SetReversed a = new SetReversed(true);
                public static Back b = new Back(15.5);
                public static SplineTo c = new SplineTo(-49.5, -8.8, 180);
                static TrajectorySequenceContainer cycle2Pickup = new TrajectorySequenceContainer(Speed::getPickupConstraints, a, b, c);
            }

            public static Cycle2Drop ecycle2Drop;
            public static class Cycle2Drop {
                public static SetReversed a = new SetReversed(true);
                public static Forward b = new Forward(16);
                public static SplineTo c = new SplineTo(-34.8, -13.9, -45);
                static TrajectorySequenceContainer cycle2Drop = new TrajectorySequenceContainer(Speed::getDropConstraints, a, b, c);
            }

            public static Cycle3Pickup fcycle3Pickup;
            public static class Cycle3Pickup {
                public static SetReversed a = new SetReversed(true);
                public static Back b = new Back(15.5);
                public static SplineTo c = new SplineTo(-51, -8.8, 180);
                static TrajectorySequenceContainer cycle3Pickup = new TrajectorySequenceContainer(Speed::getPickupConstraints, a, b, c);
            }

            public static Cycle3Drop gcycle3Drop;
            public static class Cycle3Drop {
                public static SetReversed a = new SetReversed(true);
                public static Forward b = new Forward(16);
                public static SplineTo c = new SplineTo(-35.9, -14.1, -45);
                static TrajectorySequenceContainer cycle3Drop = new TrajectorySequenceContainer(Speed::getDropConstraints, a, b, c);
            }

            public static Cycle4Pickup hcycle4Pickup;
            public static class Cycle4Pickup {
                public static SetReversed a = new SetReversed(true);
                public static Back b = new Back(15);
                public static SplineTo c = new SplineTo(-53.6, -8.3, 180);

                static TrajectorySequenceContainer cycle4Pickup = new TrajectorySequenceContainer(Speed::getPickupConstraints, a, b, c);
            }

            public static Cycle4Drop icycle4Drop;
            public static class Cycle4Drop {
//                public static SetReversed a = new SetReversed(true);
//                public static Forward b = new Forward(16);
//                public static SplineTo c = new SplineTo(-36.25, -14.1, -45);
//                static TrajectorySequenceContainer cycle4Drop = new TrajectorySequenceContainer(Speed::getDropConstraints, a, b, c);

                public static SetReversed a = new SetReversed(true);
                public static Forward b = new Forward(16);
                public static SplineTo c = new SplineTo(-47, -8.65, 180);
                static TrajectorySequenceContainer cycle4Drop = new TrajectorySequenceContainer(Speed::getDropConstraints, a, b, c);
            }

            public static Park jpark;
            public static class Park {
                public static double leftX = 8;
                public static double midX = 36;
                public static double rightX = 60;
                public static double y = -12;
                public static double heading = -90;
                public static double endHeading = -180;
                static TrajectorySequenceContainer getPark(double x) {
                    return new TrajectorySequenceContainer(
                            Speed::getParkConstraint,
                            new Back(5),
                            new SplineToSplineHeading(x, y, heading, endHeading)
                    );
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
                                new ArmHighFrontCommand(slide, pivot, claw, turnServo, true)
                        ),
                        new WaitCommand(100),
                        new AutoDropConeCommand(claw, slide, pivot, true),



                        /* Cycle 1 Pickup */
                        new ParallelCommandGroup(
                                new ArmCone5BackCommand(slide, claw, pivot, turnServo),
                                new TrajectorySequenceContainerFollowCommand(drivetrain, RightRegionalAutoConstants.Path.Cycle1Pickup.cycle1Pickup,
                                        new DisplacementCommand(27, new AutoPickConeCommand(slide, claw)))
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
                                        new DisplacementCommand(27, new AutoPickConeCommand(slide, claw)))
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
                                        new DisplacementCommand(27, new AutoPickConeCommand(slide, claw)))
                        ),
//                        new AutoPickConeCommand(slide, claw),

                        /* Cycle 3 Drop */
                        new ParallelCommandGroup(
                                new ArmMidFrontCommand(slide, pivot, claw, turnServo,true),
                                new TrajectorySequenceContainerFollowCommand(drivetrain, RightRegionalAutoConstants.Path.Cycle3Drop.cycle3Drop)
                        ),
                        new AutoDropConeCommand(claw, slide, pivot, true),


                        /* Cycle 4 Pickup */
                        new ParallelCommandGroup(
                                new ArmCone2BackCommand(slide, claw, pivot, turnServo),
                                new TrajectorySequenceContainerFollowCommand(drivetrain, RightRegionalAutoConstants.Path.Cycle4Pickup.cycle4Pickup,
                                        new DisplacementCommand(27, new AutoPickConeCommand(slide, claw)))
                        ),
//                        new AutoPickConeCommand(slide, claw),

                        /* Cycle 4 Drop */
                        new ParallelCommandGroup(
                                new ArmMidFrontCommand(slide, pivot, claw, turnServo,true),
                                new TrajectorySequenceContainerFollowCommand(drivetrain, RightRegionalAutoConstants.Path.Cycle4Drop.cycle4Drop)
                        ),
                        new AutoDropConeCommand(claw, slide, pivot, true),

                        /* Park */
                        new ParallelCommandGroup(
                                new TrajectorySequenceContainerFollowCommand(drivetrain, RightRegionalAutoConstants.Path.Park.getPark(finalX)),
                                new SequentialCommandGroup(
                                        new SlideResetUpAutonCommand(slide, pivot, claw, turnServo),
                                        new WaitCommand(200),
                                        run(pivot::stopArm)
                                )
                        ),
                        run(() -> PoseStorage.currentPose = drivetrain.getPoseEstimate()),

                        /* Save Pose and end opmode*/

                        run(this::stop)
                )
        );
    }
}