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
import android.app.Activity;
import android.graphics.Color;
import android.view.View;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;


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

@Autonomous(name="Autonom_BasicFunctions+Color_Range")

public class Autonom_BasicFunctions_ColorAndRange extends LinearOpMode {

    //Motoare
    DcMotor TLM = null;
    DcMotor TRM = null;
    DcMotor BLM = null;
    DcMotor BRM = null;
    //variabile semi-globale
    float target=0;



    //variabile pt imu
    BNO055IMU imu;
    Orientation angles;
    //variabile pt color/range
    ColorSensor sensorColor;
    DistanceSensor sensorDistance;


    @Override
    public void runOpMode()  {
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


            //  IMU
            BNO055IMU.Parameters parameters=new BNO055IMU.Parameters();
            parameters.angleUnit= BNO055IMU.AngleUnit.DEGREES;
            parameters.calibrationDataFile="BNO055IMUCalibration.json";
            imu = hardwareMap.get(BNO055IMU.class, "imu");
            imu.initialize(parameters);
            angles=imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            //  COLOR_RANGE
        sensorColor = hardwareMap.get(ColorSensor.class, "sensor_color_distance");

        // get a reference to the distance sensor that shares the same name.
        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_color_distance");

        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F, 0F, 0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        // sometimes it helps to multiply the raw RGB values with a scale factor
        // to amplify/attentuate the measured values.
        final double SCALE_FACTOR = 255;

        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);



        waitForStart();

        Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                (int) (sensorColor.green() * SCALE_FACTOR),
                (int) (sensorColor.blue() * SCALE_FACTOR),
                hsvValues);
        Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                (int) (sensorColor.green() * SCALE_FACTOR),
                (int) (sensorColor.blue() * SCALE_FACTOR),
                hsvValues);



        // send the info back to driver station using telemetry function.
        telemetry.addData("Distance (cm)",
                String.format(Locale.US, "%.02f", sensorDistance.getDistance(DistanceUnit.CM)));
        telemetry.addData("Alpha", sensorColor.alpha());
        telemetry.addData("Red  ", sensorColor.red());
        telemetry.addData("Green", sensorColor.green());
        telemetry.addData("Blue ", sensorColor.blue());
        telemetry.addData("Hue", hsvValues[0]);


           /*Forward(450,0.3);
            TLM.setPower(0);
            BLM.setPower(0);
            TRM.setPower(0);
            BRM.setPower(0);
            sleep(500);
            BLM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            TLM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BRM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            TRM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);*/

            ForwardColorRange(450,0.3);
      /*  int target=450;
        double pwr=0.3;
        TLM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BLM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        TRM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BRM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        TLM.setPower(-pwr);
        BLM.setPower(-pwr);
        TRM.setPower(-pwr);
        BRM.setPower(-pwr);

        double r,g,b,hue;
        r=sensorColor.red();
        g=sensorColor.green();
        b=sensorColor.blue();
        hue=sensorColor.argb();

        // sensorDistance.getDistance(DistanceUnit.CM)<=7
        while (r<b && g<b )
        {
            r=sensorColor.red();
            g=sensorColor.green();
            b=sensorColor.blue();
            telemetry.addData("Status:", "Moving to pos");
            telemetry.update();
            telemetry.addData("Red  ", sensorColor.red());
            telemetry.addData("Green", sensorColor.green());
            telemetry.addData("Blue ", sensorColor.blue());
            telemetry.addData("Hue", hsvValues[0]);
            telemetry.update();

            if (r>b&& g>b)
                break;

        }//sau <=2 depinde de senzor

*/

        TLM.setPower(0);
        BLM.setPower(0);
        TRM.setPower(0);
        BRM.setPower(0);
        sleep(200);

        telemetry.addData("Status:", "At pos");
        telemetry.update();
        /* We've reached position once that loop above this ends. Stop motors from moving. */
        TLM.setPower(0);
        BLM.setPower(0);
        TRM.setPower(0);
        BRM.setPower(0);

            sleep(500);
             TLM.setPower(0);
             BLM.setPower(0);
             TRM.setPower(0);
             BRM.setPower(0);
            BLM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            TLM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BRM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            TRM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            //telemetrie


    }
    public void turnAbsolute(float target,double pwr)

    {
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
        TRM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);BNO055IMU.Parameters parameters=new BNO055IMU.Parameters();
        parameters.angleUnit= BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile="BNO055IMUCalibration.json";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        angles=imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

         float zAccumulated= angles.firstAngle;
         telemetry.addData("mmc 5 plzzz",Math.abs(zAccumulated-target));
         telemetry.update();

        while (Math.abs(zAccumulated - target) >= 5) {

            angles=imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            zAccumulated=  angles.firstAngle;


            if (zAccumulated >= target) {
                TLM.setPower(-pwr);
                BLM.setPower(-pwr);
                TRM.setPower(pwr);
                BRM.setPower(pwr);
                telemetry.addData("status:","rotating");

                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    telemetry.addData("Heading: ", angles.firstAngle);
                    telemetry.addData("Roll: ", angles.secondAngle);
                    telemetry.addData("Pitch", angles.thirdAngle);
                    telemetry.update();
            }


            if (zAccumulated <target) {
                TLM.setPower(pwr);
                BLM.setPower(pwr);
                TRM.setPower(-pwr);
                BRM.setPower(-pwr);
                telemetry.addData("status:","rotating");

                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                telemetry.addData("Heading: ", angles.firstAngle);
                telemetry.addData("Roll: ", angles.secondAngle);
                telemetry.addData("Pitch", angles.thirdAngle);
                telemetry.update();
            }

            telemetry.update();



        }
        telemetry.update();
        TLM.setPower(0);
        BLM.setPower(0);
        TRM.setPower(0);
        BRM.setPower(0);
        while (opModeIsActive() && !isStopRequested() && TLM.isBusy() && BLM.isBusy() && TRM.isBusy() && BRM.isBusy()) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("Heading: ", angles.firstAngle);
            telemetry.addData("Roll: ", angles.secondAngle);
            telemetry.addData("Pitch", angles.thirdAngle);
            telemetry.update();
        }




    }
    public void Forward(int target,double pwr) {


        TLM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BLM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        TRM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BRM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        TLM.setTargetPosition(-target);
        BLM.setTargetPosition(-target);
        TRM.setTargetPosition(-target);
        BRM.setTargetPosition(-target);

        TLM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BLM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        TRM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BRM.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        TLM.setPower(pwr);
        BLM.setPower(pwr);
        TRM.setPower(pwr);
        BRM.setPower(pwr);
        while(TLM.isBusy() && BLM.isBusy() && opModeIsActive()&& TRM.isBusy() && opModeIsActive()&& BRM.isBusy() && opModeIsActive())
        {
            telemetry.addData("Status:", "Moving to pos");
            telemetry.update();
        }
        telemetry.addData("Status:", "At pos");
        telemetry.update();
        /* We've reached position once that loop above this ends. Stop motors from moving. */
        TLM.setPower(0);
        BLM.setPower(0);
        TRM.setPower(0);
        BRM.setPower(0);







    }
    public void ForwardColorRange(int target,double pwr) {
        int ok=1;
        int nuvede=0;
        float hsvValues[] = {0F, 0F, 0F};
        final float values[] = hsvValues;
        final double SCALE_FACTOR = 255;
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);
        Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                (int) (sensorColor.green() * SCALE_FACTOR),
                (int) (sensorColor.blue() * SCALE_FACTOR),
                hsvValues);

        /* We've reached position once that loop above this ends. Stop motors from moving. */



        TLM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BLM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        TRM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BRM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        TLM.setPower(-pwr);
        BLM.setPower(-pwr);
        TRM.setPower(-pwr);
        BRM.setPower(-pwr);

            double r,g,b,hue;
            r=sensorColor.red();
            g=sensorColor.green();
            b=sensorColor.blue();
            hue=sensorColor.argb();

       // sensorDistance.getDistance(DistanceUnit.CM)<=7
                while (r<b || g<b )
                {   r=sensorColor.red();
                    g=sensorColor.green();
                    b=sensorColor.blue();

                    telemetry.addData("Status:", "Moving to pos");
                    telemetry.update();
                    telemetry.addData("Red  ", sensorColor.red());
                    telemetry.addData("Green", sensorColor.green());
                    telemetry.addData("Blue ", sensorColor.blue());
                    telemetry.addData("Hue", hsvValues[0]);
                    telemetry.update();

                }//sau <=2 depinde de senzor



                TLM.setPower(0);
                BLM.setPower(0);
                TRM.setPower(0);
                BRM.setPower(0);
                sleep(200);

                telemetry.addData("Status:", "At pos");
                telemetry.update();
                /* We've reached position once that loop above this ends. Stop motors from moving. */
                TLM.setPower(0);
                BLM.setPower(0);
                TRM.setPower(0);
                BRM.setPower(0);



    }

}