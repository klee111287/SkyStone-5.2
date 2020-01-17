/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@Disabled
@Autonomous(name="Charge 11171 Autonomous Loading Zone", group="Linear Opmode")

public class Charge11171Autonomous2 extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive1 = null;
    private DcMotor rightDrive1 = null;
    private DcMotor leftDrive2 = null;
    private DcMotor rightDrive2 = null;
    private DcMotor lift = null;
    private DcMotor leftIntake = null;
    private DcMotor rightIntake = null;
    private DcMotor arm = null;
    //declare servos
    private Servo spin = null;
    private Servo grab = null;
    private Servo foundationLeft = null;
    private Servo foundationRight = null;
    private Servo hold = null;

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 0.4 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 3.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 1;
    static final double     TURN_SPEED              = 1;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive1  = hardwareMap.get(DcMotor.class, "leftDrive1");
        rightDrive1 = hardwareMap.get(DcMotor.class, "rightDrive1");
        leftDrive2  = hardwareMap.get(DcMotor.class, "leftDrive2");
        rightDrive2 = hardwareMap.get(DcMotor.class, "rightDrive2");

        //grabber = hardwareMap.get(Servo.class, "grabber");


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive1.setDirection(DcMotor.Direction.FORWARD);
        rightDrive1.setDirection(DcMotor.Direction.REVERSE);
        leftDrive2.setDirection(DcMotor.Direction.FORWARD);
        rightDrive2.setDirection(DcMotor.Direction.REVERSE);
        //grabber.setDirection(Servo.Direction.FORWARD);
        leftDrive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDrive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDrive1.setTargetPosition(0);
        rightDrive1.setTargetPosition(0);
        leftDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDrive2.setTargetPosition(0);
        rightDrive2.setTargetPosition(0);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
            // Send calculated power to wheels


            // Wait for the game to start (driver presses PLAY)
            waitForStart();

            // Step through each leg of the path,
            // Note: Reverse movement is obtained by setting a negative distance (not speed)


            //go forwards
            encoderDrive(DRIVE_SPEED,  37,  37, 37, 37, 5);  // S1: Forward 48 Inches with 5 Sec timeout

            //grab foundation
            foundationLeft.setPosition(90);
            foundationRight.setPosition(-90);

            //go backwards into build site
            encoderDrive(DRIVE_SPEED, -37, -37, -37, -37, 5);

            //strafe to under bridge
            encoderDrive(DRIVE_SPEED, 124, -124, -124, 124, 10);

        telemetry.addData("Path", "Complete");
            telemetry.update();
        }

        /*
         *  Method to perform a relative move, based on encoder counts.
         *  Encoders are not reset as the move is based on the current position.
         *  Move will stop if any of three conditions occur:
         *  1) Move gets to the desired position
         *  2) Move runs out of time
         *  3) Driver stops the opmode running.
         */
        public void encoderDrive(double speed,
                                 double leftInches1,
                                 double rightInches1,
                                 double leftInches2,
                                 double rightInches2,
                                 double timeoutS) {
            int newLeft1Target;
            int newRight1Target;
            int newLeft2Target;
            int newRight2Target;

            // Ensure that the opmode is still active
            if (opModeIsActive()) {

                // Determine new target position, and pass to motor controller
                newLeft1Target = leftDrive1.getCurrentPosition() + (int)(leftInches1 * COUNTS_PER_INCH);
                newRight1Target = rightDrive1.getCurrentPosition() + (int)(rightInches1 * COUNTS_PER_INCH);
                newLeft2Target = leftDrive2.getCurrentPosition() + (int)(leftInches2 * COUNTS_PER_INCH);
                newRight2Target = rightDrive2.getCurrentPosition() + (int)(rightInches2 * COUNTS_PER_INCH);
                leftDrive1.setTargetPosition(newLeft1Target);
                rightDrive1.setTargetPosition(newRight1Target);
                leftDrive2.setTargetPosition(newLeft2Target);
                rightDrive2.setTargetPosition(newRight2Target);

                // Turn On RUN_TO_POSITION
                leftDrive1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightDrive1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftDrive2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightDrive2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                // reset the timeout time and start motion.
                runtime.reset();
                leftDrive1.setPower(Math.abs(speed));
                rightDrive1.setPower(Math.abs(speed));
                leftDrive2.setPower(Math.abs(speed));
                rightDrive2.setPower(Math.abs(speed));

                // keep looping while we are still active, and there is time left, and both motors are running.
                // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
                // its target position, the motion will stop.  This is "safer" in the event that the robot will
                // always end the motion as soon as possible.
                // However, if you require that BOTH motors have finished their moves before the robot continues
                // onto the next step, use (isBusy() || isBusy()) in the loop test.
                while (opModeIsActive() &&
                        (runtime.seconds() < timeoutS) &&
                        (leftDrive1.isBusy() && rightDrive1.isBusy() && rightDrive2.isBusy() && leftDrive2.isBusy())) {

                    // Display it for the driver.
                    telemetry.addData("Path1",  "Running to %7d :%7d", newLeft1Target,  newRight1Target, newLeft2Target,  newRight2Target);
                    telemetry.addData("Path2",  "Running at %7d :%7d",
                            leftDrive1.getCurrentPosition(),
                            rightDrive1.getCurrentPosition(),
                            leftDrive2.getCurrentPosition(),
                            rightDrive2.getCurrentPosition());
                    telemetry.update();
                }

                // Stop all motion;
                leftDrive1.setPower(0);
                rightDrive1.setPower(0);
                leftDrive2.setPower(0);
                rightDrive2.setPower(0);

                // Turn off RUN_TO_POSITION
                leftDrive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightDrive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                leftDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                //  sleep(250);   // optional pause after each move
            }
        }
    }