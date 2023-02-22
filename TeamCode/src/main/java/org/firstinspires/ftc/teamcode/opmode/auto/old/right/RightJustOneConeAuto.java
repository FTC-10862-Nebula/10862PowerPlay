/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.opmode.auto.old.right;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.commands.old.auto.JustONECone;
import org.firstinspires.ftc.teamcode.commands.drive.trajectory.StrafeLeftCommand;
import org.firstinspires.ftc.teamcode.commands.drive.trajectory.StrafeRightCommand;
import org.firstinspires.ftc.teamcode.subsystems.Pivot;
import org.firstinspires.ftc.teamcode.util.MatchOpMode;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.misc.TagVision;
import org.firstinspires.ftc.teamcode.subsystems.SensorColor;
import org.firstinspires.ftc.teamcode.subsystems.Slide;
import org.firstinspires.ftc.teamcode.subsystems.TurnServo;

@Disabled
@Autonomous
public class RightJustOneConeAuto extends MatchOpMode
{
    private int tagNum = 0;

    // Subsystems
    private Pivot pivot;
    private Claw claw;
    private Drivetrain drivetrain;
    private Slide slide;
    private TagVision tagVision;
    private TurnServo turnServo;
    private SensorColor sensorColor;


    @Override
    public void robotInit() {
        claw = new Claw(telemetry, hardwareMap);
        pivot = new Pivot( telemetry, hardwareMap);
        drivetrain = new Drivetrain(new MecanumDrive(hardwareMap, telemetry, false), telemetry, hardwareMap);
        drivetrain.init();
        slide = new Slide( telemetry, hardwareMap);
        turnServo = new TurnServo(telemetry, hardwareMap);
        sensorColor = new SensorColor(hardwareMap, telemetry);
        tagVision = new TagVision(hardwareMap, telemetry);
        sensorColor = new SensorColor(hardwareMap, telemetry);
//        tagVision.init(hardwareMap);
        while (!isStarted() && !isStopRequested())
        {
            tagVision.updateTagOfInterest();
            tagVision.tagToTelemetry();
            telemetry.update();
//            new InstantCommand(tagVision::getsend, tagVision);

        }
        this.matchStart();
    }

    public void matchStart() {
        tagNum = tagVision.getTag();

        SequentialCommandGroup autonGroup;
        switch (tagNum) {
            case 1: { //Left
                autonGroup = new SequentialCommandGroup(
                        new JustONECone(drivetrain, slide, pivot, claw, turnServo, sensorColor),
                        new StrafeLeftCommand(drivetrain, 10)
                );
                break;
            }
            case 2: { //Mid
                autonGroup = new SequentialCommandGroup(
                        new JustONECone(drivetrain, slide, pivot, claw, turnServo, sensorColor),
                        new StrafeRightCommand(drivetrain, 14)
                );
                break;
            }
            case 3: { //High
                autonGroup =new SequentialCommandGroup(
                        new JustONECone(drivetrain, slide, pivot, claw, turnServo, sensorColor),
                        new StrafeRightCommand(drivetrain, 34.5)
                );
                break;
            }
            default: {
                autonGroup = new SequentialCommandGroup(
                        new JustONECone(drivetrain, slide, pivot, claw, turnServo, sensorColor),
                        new StrafeRightCommand(drivetrain, 14)
                );
            }
        }
        schedule(autonGroup);
    }
}