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

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class HardwareChassisHerman
{
    /* Public OpMode members. */
    public DcMotor frontLeftDrive   = null;
    public DcMotor backLeftDrive   = null;
    public DcMotor frontRightDrive  = null;
    public DcMotor backRightDrive  = null;
    public DcMotor eolico  = null;
    public DcMotor recogedor  = null;
    public CRServo LiftL1 = null;
    public CRServo LiftL2 = null;
    public CRServo LiftL3 = null;
    public CRServo LiftL4 = null;
    public CRServo LiftL5 = null;
    public CRServo LiftL6 = null;
    public Servo eject = null;
    public DcMotor Elevador = null;


    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareChassisHerman(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        frontLeftDrive  = hwMap.get(DcMotor.class, "FL");
        backLeftDrive  = hwMap.get(DcMotor.class, "BL");
        frontRightDrive  = hwMap.get(DcMotor.class, "FR");
        backRightDrive  = hwMap.get(DcMotor.class, "BR");
        eolico = hwMap.get(DcMotor.class, "EO");
        recogedor = hwMap.get(DcMotor.class, "RE");
        LiftL1 = hwMap.get(CRServo.class, "L1");
        LiftL2 = hwMap.get(CRServo.class, "L2");
        LiftL3 = hwMap.get(CRServo.class, "L3");
        LiftL4 = hwMap.get(CRServo.class, "L4");
        LiftL5 = hwMap.get(CRServo.class, "L5");
        LiftL6 = hwMap.get(CRServo.class, "L6");
        Elevador = hwMap.get(DcMotor.class, "EL");
        eject = hwMap.get(Servo.class, "EJ");


        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        eolico.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        recogedor.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        Elevador.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power
        frontLeftDrive.setPower(0);
        backLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backRightDrive.setPower(0);
        eolico.setPower(0);
        recogedor.setPower(0);
        LiftL1.setPower(0);
        LiftL2.setPower(0);
        LiftL3.setPower(0);
        LiftL4.setPower(0);
        LiftL5.setPower(0);
        LiftL6.setPower(0);
        Elevador.setPower(0);
        eject.setPosition(0);


        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        eolico.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        recogedor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Elevador.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }
    public void normalize(double[] wheelSpeeds)
    {
        double maxMagnitude = Math.abs(wheelSpeeds[0]);
        for (int i = 1; i < wheelSpeeds.length; i++)
        {
            double magnitude = Math.abs(wheelSpeeds[i]);

            if (magnitude > maxMagnitude)
            {
                maxMagnitude = magnitude;
            }
        }

        if (maxMagnitude > 1.0)
        {
            for (int i = 0; i < wheelSpeeds.length; i++)
            {
                wheelSpeeds[i] /= maxMagnitude;
            }
        }
    }
 }

