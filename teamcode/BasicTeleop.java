
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

@TeleOp(name="BasicTeleop", group="TeleOp")

public class BasicTeleop extends LinearOpMode {

    // declare opmode members
    HardwareMap21 robot = new HardwareMap21();
    private ElapsedTime runtime = new ElapsedTime();
    double speedAdjust = 5;
    // Setup a variable for each drive wheel to save power level for telemetry

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
        robot.clasp.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // set direction of wheels
        // TODO: fix strafing movement
        robot.BleftDrive.setDirection(DcMotor.Direction.REVERSE);
        robot.FleftDrive.setDirection(DcMotor.Direction.REVERSE);
        robot.BrightDrive.setDirection(DcMotor.Direction.FORWARD);
        robot.FrightDrive.setDirection(DcMotor.Direction.FORWARD);
        robot.carousel.setDirection(DcMotor.Direction.REVERSE);
        robot.clasp.setDirection(DcMotor.Direction.REVERSE);

        // set servo positions
        robot.claw.setPosition(0);

        // wait for driver to press start
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPower;
            double rightPower;

            /*
            // POV Mode uses left stick to go forward, and right stick to turn.
            // uses basic math to combine motions and is easier to drive straight.
            double drive = -gamepad1.left_stick_y;
            double turn  =  gamepad1.right_stick_x;
            leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
            rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;
            robot.BleftDrive.setPower(leftPower);
            robot.BrightDrive.setPower(rightPower);
            robot.FleftDrive.setPower(leftPower);
            robot.FrightDrive.setPower(rightPower);
            */

            // for mecanum wheels

            robot.BleftDrive.setPower((gamepad1.left_stick_y +  gamepad1.left_stick_x - gamepad1.right_stick_x) * (-speedAdjust / 10));
            robot.BrightDrive.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x) * (-speedAdjust / 10));
            robot.FleftDrive.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x) * (-speedAdjust / 10));
            robot.FrightDrive.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x) * (-speedAdjust / 10));

            // A and B button - move motor for clasp
            if (gamepad1.a){
                robot.clasp.setPower(0.5);
            } else if (gamepad1.b){
                robot.clasp.setPower(-0.5);
            } else {
                robot.clasp.setPower(0);
            }


            // X and Y button - move servo for clasp
            if (gamepad1.x) {
                robot.claw.setPosition(-1);
            } else if (gamepad1.y){
                robot.claw.setPosition(-1);
            } else {
                robot.claw.setPosition(0);
            }

            //right trigger for carousel positive power
            if (gamepad1.right_trigger > 0.5){
                robot.carousel.setPower(2);
            } else if (gamepad1.left_trigger > 0.5){
                robot.carousel.setPower(-2);
            } else {
                robot.carousel.setPower(0);
            }

            // show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            //telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();
        }
    }
}
