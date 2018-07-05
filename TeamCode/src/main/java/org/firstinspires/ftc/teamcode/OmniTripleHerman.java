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
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="RoboPro", group="Iterative Opmode")

public class OmniTripleHerman extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor centreDrive = null;
    private DcMotor elevadorDrive = null;
    private CRServo cajasDrive = null;
    public DcMotor recogedorLeft  = null;
    public DcMotor recogedorRight  = null;
    public DcMotor elevador = null;
    public Servo grip = null;
    public Servo lift = null;
    public DcMotor eolico = null;
    public Servo puerta = null;


    //Code to run ONCE when the driver hits INIT
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        leftDrive  = hardwareMap.get(DcMotor.class, "leftMotor");
        rightDrive = hardwareMap.get(DcMotor.class, "rightMotor");
        centreDrive = hardwareMap.get(DcMotor.class, "centreMotor");
        elevadorDrive = hardwareMap.get(DcMotor.class, "motor");
        cajasDrive = hardwareMap.get(CRServo.class, "servoCajas");
        recogedorRight = hardwareMap.get(DcMotor.class, "RR");
        recogedorLeft  = hardwareMap.get(DcMotor.class, "RL");
        elevador = hardwareMap.get(DcMotor.class,"EL");
        lift = hardwareMap.get(Servo.class,"LF");
        grip = hardwareMap.get(Servo.class,"GR");
        eolico = hardwareMap.get(DcMotor.class, "EO");
        puerta = hardwareMap.get(Servo.class,"PU");


        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        centreDrive.setDirection(DcMotor.Direction.FORWARD);
        recogedorLeft.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        recogedorRight.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        elevador.setDirection(DcMotor.Direction.FORWARD);
        eolico.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
    }

    //Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
    @Override
    public void init_loop() {
    }

    //Code to run ONCE when the driver hits PLAY
    @Override
    public void start() {
        runtime.reset();
    }

    //Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    double cajasPower = 0;
    boolean presd = false;
    boolean presd1 = false;

    @Override
    public void loop() {
        double leftPower;
        double rightPower;
        double centrePower;
        double elevadorPower;

        // POV Mode uses left stick to go forward, and right stick to turn.
        double drive = -gamepad1.left_stick_y;
        double turn  =  gamepad1.left_stick_x;
        leftPower    = Range.clip(drive + turn, -1.0, 1.0);
        rightPower   = Range.clip(drive - turn, -1.0, 1.0);
        centrePower = gamepad1.right_stick_x;

        // Tank Mode uses one stick to control each wheel.
        /*leftPower = -gamepad1.left_stick_y;
        rightPower = -gamepad1.left_stick_x ;*/

        // Control power of wheels.
        if (gamepad1.right_trigger>0) {
          leftPower = leftPower * 0.75;
          rightPower = rightPower * 0.75;
          centrePower = centrePower * 0.75;
        } else if(gamepad1.left_trigger>0){
          leftPower = leftPower * 0.5 + leftPower * 0.5*(1-gamepad1.left_trigger);
          rightPower = rightPower * 0.5 + rightPower * 0.5*(1-gamepad1.left_trigger);
          centrePower = centrePower * 0.5 + centrePower * 0.5*(1-gamepad1.left_trigger);
        }

        if(gamepad1.a) {
            recogedorLeft.setPower(1);
            recogedorRight.setPower(1);
        } else if (gamepad1.b){
            recogedorLeft.setPower(-1);
            recogedorRight.setPower(-1);
        } else {
            recogedorLeft.setPower(0);
            recogedorRight.setPower(0);
        }


        if (gamepad1.y){
            puerta.setPosition(1);
        } else if (gamepad1.x){
            puerta.setPosition(0);
        }

        if (gamepad1.dpad_up) {
            elevador.setPower(1);

        } else if (gamepad1.dpad_down){
            elevador.setPower(-1);

        } else {
            elevador.setPower(0);
        }


        //La parte cool del morro este, Santi
        if (gamepad2.a){
            grip.setPosition(0);
        } else if (gamepad2.b) {
            grip.setPosition(1);
        }

        if (gamepad2.dpad_up){
            lift.setPosition(1);
        } else if (gamepad2.dpad_down) {
            lift.setPosition(0);
        }

        if (gamepad2.x){
            eolico.setPower(1);
        } else  if (gamepad2.y) {
            eolico.setPower(-1);
        } else {
            eolico.setPower(0);
        }
        // bye bye Santi.


        // Send calculated power to wheels
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
        centreDrive.setPower(centrePower);

        cajasDrive.setPower(cajasPower);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%f), right (%f)", leftPower, rightPower);
    }

    //Code to run ONCE after the driver hits STOP
    @Override
    public void stop() {
    }

}
