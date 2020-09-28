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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


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

@Autonomous(name="Autonom_BasicFunctions_Test_Odometry")

public class Autonom_BasicFunctions_Test_Odometry extends LinearOpMode {

    // Declare OpMode members.
    DcMotor TLM = null;
    DcMotor TRM = null;
    DcMotor BLM = null;
    DcMotor BRM = null;






    BNO055IMU imu;
    Orientation angles;


    @Override
    public void runOpMode() {


         TRM = hardwareMap.dcMotor.get("Tright_drive");
         BRM = hardwareMap.dcMotor.get("Bright_drive");
        TLM = hardwareMap.dcMotor.get("Tleft_drive");
        BLM = hardwareMap.dcMotor.get("Bleft_drive");

        BLM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        TLM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BRM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        TRM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //am regasit printe elementele robotului conectate le hub cele 4 motoare


        BLM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        TLM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BRM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        TRM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //servomotorul
        TRM.setDirection(DcMotorSimple.Direction.REVERSE);
        BRM.setDirection(DcMotorSimple.Direction.REVERSE);

        BLM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        TLM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BRM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        TRM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        // setup
        waitForStart();

           /*Forward(450,0.3);
            TLM.setPower(0);
            BLM.setPower(0);
            TRM.setPower(0);
            BRM.setPower(0);
            sleep(500);
            BLM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            TLM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BRM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            TRM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            telemetry.addData("reached turnAbsolute",".");
            telemetry.update();
            turnAbsolute(90,0.15);
            TLM.setPower(0);
            BLM.setPower(0);
            TRM.setPower(0);
            BRM.setPower(0);
            sleep(500);
            BLM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            TLM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BRM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            TRM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            telemetry.addData("reached Forward 1",".");
            Forward(250,0.3);
            TLM.setPower(0);
            BLM.setPower(0);
            TRM.setPower(0);
            BRM.setPower(0);
            sleep(500);
            BLM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            TLM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BRM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            TRM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Forward(-250,0.3);
            TLM.setPower(0);
            BLM.setPower(0);
            TRM.setPower(0);
            BRM.setPower(0);
            sleep(500);
            BLM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            TLM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BRM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            TRM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        angles=imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            turnAbsolute(-90,0.15);
            TLM.setPower(0);
            BLM.setPower(0);
            TRM.setPower(0);
            BRM.setPower(0);
            sleep(500);
            BLM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            TLM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BRM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            TRM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Forward(450,0.3);
            TLM.setPower(0);
            BLM.setPower(0);
            TRM.setPower(0);
            BRM.setPower(0);
            sleep(500);
            BLM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            TLM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BRM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            TRM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


            //telemetrie

            */

        BLM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        TLM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BRM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        TRM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BLM.setTargetPosition(-1000);
        BRM.setTargetPosition(-1000);
        TRM.setTargetPosition(-1000);
        BLM.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        BRM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        TRM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BLM.setPower(0.5);
        TLM.setPower(0.5);
        BRM.setPower(0.5);
        TRM.setPower(0.5);
        int max=0;
        while(BLM.isBusy() &&BRM.isBusy() &&TRM.isBusy() && opModeIsActive())
        {

            telemetry.addData("val. motorului ", BLM.getCurrentPosition());
            telemetry.addData("in loop", BLM.getPower());
            telemetry.update();
            TLM.setPower(BLM.getPower());


        }
        telemetry.addData("inafara loopului", BLM.getCurrentPosition());
        telemetry.update();
        BLM.setPower(0);
        TLM.setPower(0);
        BLM.setPower(0);
        TRM.setPower(0);
        BRM.setPower(0);

    }

}