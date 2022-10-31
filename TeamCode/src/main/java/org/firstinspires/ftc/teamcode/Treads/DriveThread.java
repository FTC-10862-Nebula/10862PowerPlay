package org.firstinspires.ftc.teamcode.Treads;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
public class DriveThread extends Thread
    {
//        public DriveThread()
//        {
//            this.setName("DriveThread");
//wheres the servo declaration
//
//            System.out.println("%s");
//            System.out.println(this.getName());
//        }
//
//        // called when tread.start is called. thread stays in loop to do what it does until exit is
//        // signaled by main code calling thread.interrupt.
//        @Override
//        public void run()
//        {
//            System.out.println("Starting thread %s");
//            System.out.println(this.getName());
//
//            try
//            {
//                while (!isInterrupted())
//                {
//                    // we record the Y values in the main class to make showing them in telemetry
//                    // easier.
//                    new DriveForwardCommand(drivetrain, 13);
////                    leftY = gamepad1.left_stick_y * -1;
////                    rightY = gamepad1.right_stick_y * -1;
////
////                    leftMotor.setPower(1);
////                    rightMotor.setPower(Range.clip(rightY, -1.0, 1.0));
//
//                    idle();
//                }
//            }
//            // interrupted means time to shutdown. note we can stop by detecting isInterrupted = true
//            // or by the interrupted exception thrown from the sleep function.
//            catch (InterruptedException e) {
//                Logging.log("%s interrupted", this.getName());
//            }
//            // an error occurred in the run loop.
//            catch (Exception e) {
//                e.printStackTrace(Logging.logPrintStream);
//            }
//
//            Logging.log("end of thread %s", this.getName());
//        }
    }