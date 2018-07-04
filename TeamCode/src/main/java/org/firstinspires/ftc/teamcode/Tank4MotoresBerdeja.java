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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * This file provides basic Telop driving for a Pushbot robot.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 *
 * This particular OpMode executes a basic Tank Drive Teleop for a PushBot
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Tank 4 llantas Berdeja", group="Teleop")
//@Disabled
public class Tank4MotoresBerdeja extends OpMode {

    /* Declare OpMode members. */
    HardwareChassisBerdeja robot       = new HardwareChassisBerdeja(); // use the class created to define a Pushbot's hardware
                                                         // could also use HardwarePushbotMatrix class.

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double drive;
        double turn;
        double left;
        double right;
        double max;
        double angle = 0.0;

        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        drive = -gamepad1.left_stick_y;
        turn  =  gamepad1.right_stick_x;

        // Combine drive and turn for blended motion.
        left  = drive + turn;
        right = drive - turn;

        // Normalize the values so neither exceed +/- 1.0
        max = Math.max(Math.abs(left), Math.abs(right));
        if (max > 1.0)
        {
            left /= max;
            right /= max;
        }

        if(gamepad1.a) {
            robot.recogedorLeft.setPower(1);
            robot.recogedorRight.setPower(1);
        } else if (gamepad1.b){
            robot.recogedorLeft.setPower(-1);
            robot.recogedorRight.setPower(-1);
        } else {
            robot.recogedorLeft.setPower(0);
            robot.recogedorRight.setPower(0);
        }


        if (gamepad1.y){
            robot.puerta.setPosition(1);
        } else if (gamepad1.x){
            robot.puerta.setPosition(0);
        }

        if (gamepad1.dpad_up) {
            robot.elevador.setPower(1);

        } else if (gamepad1.dpad_down){
            robot.elevador.setPower(-1);

        } else {
            robot.elevador.setPower(0);
        }


            //La parte cool del morro este, Santi
        if (gamepad2.a){
            robot.grip.setPosition(0);
        } else if (gamepad2.b) {
            robot.grip.setPosition(1);
        }

        if (gamepad2.dpad_up){
            robot.lift.setPosition(1);
        } else if (gamepad2.dpad_down) {
            robot.lift.setPosition(0);
        }

        if (gamepad2.x){
            robot.eolico.setPower(1);
        } else  if (gamepad2.y) {
            robot.eolico.setPower(-1);
        } else {
            robot.eolico.setPower(0);
        }
        // bye bye Santi.

        // Output the safe vales to the motor drives.
        robot.frontLeftDrive.setPower(left);
        robot.backLeftDrive.setPower(left);
        robot.frontRightDrive.setPower(right);
        robot.backRightDrive.setPower(right);

        // Send telemetry message to signify robot running;

        telemetry.addData("left",  "%.2f", left);
        telemetry.addData("right", "%.2f", right);


        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
