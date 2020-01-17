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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


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
//How the program will be named on the phone
@TeleOp(name="Charge 11171 TeleOp", group="Linear Opmode")

public class Charge11171_TeleOp extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor left1 = null;
    private DcMotor right1 = null;
    private DcMotor left2 = null;
    private DcMotor right2 = null;
    private DcMotor lift1 = null;
    private DcMotor lift2 = null;
    //declare servos
    private Servo foundationLeft = null;
    private Servo foundationRight = null;
    private Servo hand1 = null;
    private Servo hand2 = null;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //Hardware Maps all motors
        left1  = hardwareMap.get(DcMotor.class, "left1");
        right1 = hardwareMap.get(DcMotor.class, "right1");
        left2  = hardwareMap.get(DcMotor.class, "left2");
        right2 = hardwareMap.get(DcMotor.class, "right2");
        lift1 = hardwareMap.get(DcMotor.class, "lift1");
        lift2  = hardwareMap.get(DcMotor.class, "lift2");
        //Hardware maps all servos
        foundationLeft = hardwareMap.get(Servo.class, "foundationLeft");
        foundationRight = hardwareMap.get(Servo.class, "foundationRight");
        hand1 = hardwareMap.get(Servo.class, "hand1");
        hand2 = hardwareMap.get(Servo.class, "hand2");



        //Sets direction of all motors
        left1.setDirection(DcMotor.Direction.FORWARD);
        right1.setDirection(DcMotor.Direction.REVERSE);
        left2.setDirection(DcMotor.Direction.FORWARD);
        right2.setDirection(DcMotor.Direction.REVERSE);
        lift2.setDirection(DcMotor.Direction.FORWARD);
        lift1.setDirection(DcMotor.Direction.REVERSE);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            /*
            This part of the program is the part that controls what each button
            on the controller does. The joysticks will be configured to a
            tank drive setup.
             */

            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPower;
            double rightPower;

            //Sets the joysticks on the controller to drive the robot
            leftPower = gamepad1.left_stick_y;
            rightPower = gamepad1.right_stick_y;

            //Sets the power of each drive motor
            left1.setPower(leftPower);
            right1.setPower(rightPower);
            left2.setPower(leftPower);
            right2.setPower(rightPower);


            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            leftPower  = gamepad1.left_stick_y ;
            rightPower = gamepad1.right_stick_y ;

            //When the right bumper on controller 1 is pressed the robot will strafe to the right
            while (gamepad1.right_bumper)
            {
                left1.setPower(1);
                left2.setPower(-1);
                right1.setPower(-1);
                right2.setPower(1);
            }
            //When the left bumper on controller 1 is pressed the robot will strafe to the left
            while (gamepad1.left_bumper)
            {
                left1.setPower(-1);
                left2.setPower(1);
                right1.setPower(1);
                right2.setPower(-1);
            }
            //When the down button on the dpad is pressed on controller 2 the lift mechanism will go down
            if (gamepad2.dpad_down)
            {
                lift1.setPower(1);
                lift2.setPower(-1);
            }
            //When the up button on the dpad is pressed on controller 2 the lift mechanism will go up
            else if (gamepad2.dpad_up)
            {
                lift1.setPower(-1);
                lift2.setPower(-1);
            }
            //When there is nothing being pushed the mechanism will not do anything
            else
            {
                lift1.setPower(0);
                lift2.setPower(0);
            }
            //When the a button is pressed on controller 1, the foundation servos will drop
            if (gamepad1.a)
            {
                foundationLeft.setPosition(.9);
                foundationRight.setPosition(.3);
            }
            //When the b button is pressed on controller 1, the foundation servos will go back up
            else if (gamepad1.b)
            {
                foundationLeft.setPosition(0);
                foundationRight.setPosition(-.3);
            }
            //When the a button is pressed on controller 2, the grabber mechanism will grab the blocks
            if (gamepad2.a)
            {
               hand1.setPosition(1);
               hand2.setPosition(0);
            }
            //when the b button is pressed on controller 2, the grabber mechanism will release the blocks
            else if (gamepad2.b)
            {
                hand1.setPosition(0);
                hand2.setPosition(1);
            }
            //While the down button on the dpad is pressed on gamepad 1, the robot will slowly move backwards
            while (gamepad1.dpad_down)
            {
                left1.setPower(0.2);
                left2.setPower(0.2);
                right1.setPower(0.2);
                right2.setPower(0.2);

            }
            //While the up button on the dpad is pressed on gamepad 1, the robot will slowly move forward
            while (gamepad1.dpad_up)
            {
                left1.setPower(-0.2);
                left2.setPower(-0.2);
                right1.setPower(-0.2);
                right2.setPower(-0.2);

            }
            //While the right button on the dpad is pressed on gamepad 1, the robot will slowly strafe to the right
            while (gamepad1.dpad_right)
            {
                left1.setPower(0.2);
                left2.setPower(-0.2);
                right1.setPower(-0.2);
                right2.setPower(0.2);

            }
            //While the left button on the dpad is pressed on gamepad 1, the robot will slowly strafe to the left
            while (gamepad1.dpad_left)
            {
                left1.setPower(-0.2);
                left2.setPower(0.2);
                right1.setPower(0.2);
                right2.setPower(-0.2);

            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();
        }
    }
}
