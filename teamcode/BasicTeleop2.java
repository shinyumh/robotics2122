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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="BasicTeleop2", group="TeleOp")

public class BasicTeleop2 extends LinearOpMode {

    // declare opmode members
    HardwareMap21 robot = new HardwareMap21();
    private ElapsedTime runtime = new ElapsedTime();
    double speedAdjust = 10;

    // from https://www.reddit.com/r/FTC/comments/72oxmt/adding_strafing_for_mecanum_wheels/
    // TODO : fix strafing, wheels don't work anymore ;-;
    double drive;   // Power for forward and back motion
    double strafe;  // Power for left and right motion
    double rotate;  // Power for rotating the robot
    double FLeftPower;
    double BLeftPower;
    double FRightPower;
    double BRightPower;

    @Override
    public void runOpMode() {

        // update telemetry
        telemetry.addData("Status", "Initialized");
        robot.init(hardwareMap);
        telemetry.update();

        // set run mode
        robot.BleftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.FleftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.BrightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.FrightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.carousel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
/*
        drive = gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x;

        FLeftPower = drive + strafe + rotate;
        BLeftPower = drive - strafe + rotate;
        FRightPower = drive - strafe - rotate;
        BRightPower = drive + strafe - rotate;
*/
        // set direction of wheels
        robot.BleftDrive.setDirection(DcMotor.Direction.REVERSE);
        robot.FleftDrive.setDirection(DcMotor.Direction.FORWARD);
        robot.BrightDrive.setDirection(DcMotor.Direction.FORWARD);
        robot.FrightDrive.setDirection(DcMotor.Direction.REVERSE);
        robot.carousel.setDirection(DcMotor.Direction.FORWARD);

        // set servo positions
        robot.claw.setPosition(0);



        // wait for driver to press start
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // uses basic math to combine motions and is easier to drive straight.
            double drive = -gamepad1.left_stick_y;
            //double turn  =  gamepad1.right_stick_x;
            double leftPower    = Range.clip(drive, -1.0, 1.0) ;
            double rightPower   = Range.clip(drive, -1.0, 1.0) ;

            if (gamepad1.right_stick_x > 0.25){ // turning right
                robot.BleftDrive.setPower(0.75);
                robot.BrightDrive.setPower(-0.75);
                robot.FleftDrive.setPower(-0.75);
                robot.FrightDrive.setPower(0.75);
            } else if (gamepad1.right_stick_x < -0.25){ // turning left
                robot.BleftDrive.setPower(-0.75);
                robot.BrightDrive.setPower(0.75);
                robot.FleftDrive.setPower(0.75);
                robot.FrightDrive.setPower(-0.75);
            } else {
                robot.BleftDrive.setPower(leftPower);
                robot.BrightDrive.setPower(rightPower);
                robot.FleftDrive.setPower(leftPower);
                robot.FrightDrive.setPower(rightPower);
            }

            // Linear movement with joysticks

            /*
            robot.BleftDrive.setPower((gamepad1.left_stick_y +  gamepad1.left_stick_x - gamepad1.right_stick_x) * (-speedAdjust / 10));
            robot.BrightDrive.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x) * (-speedAdjust / 10));
            robot.FleftDrive.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x) * (-speedAdjust / 10));
            robot.FrightDrive.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x) * (-speedAdjust / 10));
*/

            // A and B button - spin carousel
            if (gamepad1.a) {
                robot.carousel.setPower(5); //2
            } else if (gamepad1.b){
                robot.carousel.setPower(-5);
            } else {
                robot.carousel.setPower(0);
            }

            // Right/Left bumpers - right & left strafe
            if (gamepad1.right_bumper) {
                robot.BleftDrive.setPower(-2);
                robot.BrightDrive.setPower(2);
                robot.FleftDrive.setPower(-2);
                robot.FrightDrive.setPower(2);
            } else if (gamepad1.left_bumper){
                robot.BleftDrive.setPower(2);
                robot.BrightDrive.setPower(-2);
                robot.FleftDrive.setPower(2);
                robot.FrightDrive.setPower(-2);
            } else {
                robot.BleftDrive.setPower(0);
                robot.BrightDrive.setPower(0);
                robot.FleftDrive.setPower(0);
                robot.FrightDrive.setPower(0);
            }

            // X and Y button - move servo for claw
            if (gamepad1.x) {
                robot.clasp.setPower(0.05);
            } else if (gamepad1.y) {
                robot.clasp.setPower(-0.05);
            } else {
                robot.clasp.setPower(0);
            }

            // X and Y button - move servo for claw
            if (gamepad1.left_trigger > 0.1) {
                robot.claw.setPosition(0.5);
            } else if (gamepad1.right_trigger > 0.1) {
                robot.claw.setPosition(-0.5);
            } else {
                robot.claw.setPosition(0);
            }



            // show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}
