package org.firstinspires.ftc.teamcode.trajectorysequence.container;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

public class TrajectorySequenceContainer {
    private final PathSegment[] pathSegments;
    private TrajectorySequence trajectorySequence;
    public TrajectorySequenceContainer (PathSegment... pathSegments) {
        this.pathSegments = pathSegments;
    }

    public TrajectorySequence build(TrajectorySequenceBuilder trajectorySequenceBuilder) {
        for (PathSegment pathSegment : pathSegments) {
            if (pathSegment.getClass() == LineTo.class) {
                LineTo lineTo = (LineTo) pathSegment;
                trajectorySequenceBuilder = trajectorySequenceBuilder.lineTo(
                        new Vector2d(
                                lineTo.x,
                                lineTo.y
                        )
                ); // cast as a LineTo
            }

            if (pathSegment.getClass() == LineToConstantHeading.class) {
                LineToConstantHeading lineToConstantHeading = (LineToConstantHeading) pathSegment;
                trajectorySequenceBuilder = trajectorySequenceBuilder.lineToConstantHeading(
                        new Vector2d(
                                lineToConstantHeading.x,
                                lineToConstantHeading.y
                        )
                );
            }

            if (pathSegment.getClass() == LineToLinearHeading.class) {
                LineToLinearHeading lineToLinearHeading = (LineToLinearHeading) pathSegment;
                trajectorySequenceBuilder = trajectorySequenceBuilder.lineToLinearHeading(
                        new Pose2d(
                                lineToLinearHeading.x,
                                lineToLinearHeading.y,
                                Math.toRadians(lineToLinearHeading.heading)
                        )
                );
            }

            if (pathSegment.getClass() == LineToSplineHeading.class) {
                LineToSplineHeading lineToSplineHeading = (LineToSplineHeading) pathSegment;
                trajectorySequenceBuilder = trajectorySequenceBuilder.lineToSplineHeading(
                        new Pose2d(
                                lineToSplineHeading.x,
                                lineToSplineHeading.y,
                                Math.toRadians(lineToSplineHeading.heading)
                        )
                );
            }

            if (pathSegment.getClass() == SplineTo.class) {
                SplineTo splineTo = (SplineTo) pathSegment;
                trajectorySequenceBuilder = trajectorySequenceBuilder.splineTo(
                        new Vector2d(
                                splineTo.x,
                                splineTo.y
                        ),
                        Math.toRadians(splineTo.endHeading)
                );
            }

            if (pathSegment.getClass() == SplineToConstantHeading.class) {
                SplineToConstantHeading splineToConstantHeading = (SplineToConstantHeading) pathSegment;
                trajectorySequenceBuilder = trajectorySequenceBuilder.splineToConstantHeading(
                        new Vector2d(
                                splineToConstantHeading.x,
                                splineToConstantHeading.y
                        ),
                        Math.toRadians(splineToConstantHeading.endHeading)
                );
            }

            if (pathSegment.getClass() == SplineToLinearHeading.class) {
                SplineToLinearHeading splineToLinearHeading = (SplineToLinearHeading) pathSegment;
                trajectorySequenceBuilder = trajectorySequenceBuilder.splineToLinearHeading(
                        new Pose2d(
                                splineToLinearHeading.x,
                                splineToLinearHeading.y,
                                Math.toRadians(splineToLinearHeading.heading)
                        ),
                        Math.toRadians(splineToLinearHeading.endHeading)
                );
            }

            if (pathSegment.getClass() == SplineToSplineHeading.class) {
                SplineToSplineHeading splineToSplineHeading = (SplineToSplineHeading) pathSegment;
                trajectorySequenceBuilder = trajectorySequenceBuilder.splineToSplineHeading(
                        new Pose2d(
                                splineToSplineHeading.x,
                                splineToSplineHeading.y,
                                Math.toRadians(splineToSplineHeading.heading)
                        ),
                        Math.toRadians(splineToSplineHeading.endHeading)
                );
            }

            if (pathSegment.getClass() == Forward.class) {
                Forward forward = (Forward) pathSegment;
                trajectorySequenceBuilder = trajectorySequenceBuilder.forward(forward.distance);
            }

            if (pathSegment.getClass() == Back.class) {
                Back back = (Back) pathSegment;
                trajectorySequenceBuilder = trajectorySequenceBuilder.forward(back.distance);
            }

            if (pathSegment.getClass() == StrafeRight.class) {
                StrafeRight strafeRight = (StrafeRight) pathSegment;
                trajectorySequenceBuilder = trajectorySequenceBuilder.forward(strafeRight.distance);
            }

            if (pathSegment.getClass() == StrafeLeft.class) {
                StrafeLeft strafeLeft = (StrafeLeft) pathSegment;
                trajectorySequenceBuilder = trajectorySequenceBuilder.forward(strafeLeft.distance);
            }

            if (pathSegment.getClass() == SetReversed.class) {
                SetReversed setReversed = (SetReversed) pathSegment;
                trajectorySequenceBuilder = trajectorySequenceBuilder.setReversed(setReversed.reversed);
            }

            if (pathSegment.getClass() == Turn.class) {
                Turn turn = (Turn) pathSegment;
                trajectorySequenceBuilder = trajectorySequenceBuilder.turn(Math.toRadians(turn.angleDegrees));
            }

        }
        trajectorySequence = trajectorySequenceBuilder.build();
        return trajectorySequence;
    }

    public TrajectorySequence build(
            Pose2d startPose,
            Double startTangent,
            TrajectoryVelocityConstraint baseVelConstraint,
            TrajectoryAccelerationConstraint baseAccelConstraint,
            double baseTurnConstraintMaxAngVel,
            double baseTurnConstraintMaxAngAccel
    ) {
        return this.build(new TrajectorySequenceBuilder(
                startPose,
                startTangent,
                baseVelConstraint,
                baseAccelConstraint,
                baseTurnConstraintMaxAngVel,
                baseTurnConstraintMaxAngAccel
        ));
    }

    public TrajectorySequence build(
            Pose2d startPose,
            double baseVelConstraint,
            double baseAccelConstraint,
            double baseTurnConstraintMaxAngVel,
            double baseTurnConstraintMaxAngAccel
    ) {
        return this.build(new TrajectorySequenceBuilder(
                startPose,
                (v, pose2d, pose2d1, pose2d2) -> baseVelConstraint,
                (v, pose2d, pose2d1, pose2d2) -> baseAccelConstraint,
                baseTurnConstraintMaxAngVel,
                baseTurnConstraintMaxAngAccel
        ));
    }

    public TrajectorySequence build(
            Pose2d startPose,
            TrajectorySequenceConstraints constraints
    ) {
        return this.build(
                startPose,
                constraints.baseVelConstraint,
                constraints.baseAccelConstraint,
                constraints.baseTurnConstraintMaxAngVel,
                constraints.baseTurnConstraintMaxAngAccel
        );
    }

    public Pose2d end() {
        return trajectorySequence.end();
    }
}
