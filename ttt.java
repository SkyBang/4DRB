package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import java.util.Locale;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/*
 * This is an example LinearOpMode that shows how to use
 * the REV Robotics Color-Distance Sensor.
 *
 * It assumes the sensor is configured with the name "sensor_color_distance".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 */
@Autonomous(name = "ttt", group = "ttt")// Comment this out to add to the opmode list
public class ttt extends LinearOpMode {
    private DcMotor ColectorE = null;
    private DcMotor ColectorV = null;
    private Servo LegoStanga1 = null;
    private  Servo LegoStanga2 = null;
    private Servo LegoDreapta1 = null;
    private  Servo LegoDreapta2 = null;
    public DcMotor MotorDF = null;
    public DcMotor MotorSF = null;
    public DcMotor MotorDJ = null;
    public DcMotor MotorSJ = null;
    double ok=0,k=0,nuvede=0;
    BNO055IMU imu;
    Orientation angles;
    /**
     * Note that the REV Robotics Color-Distance incorporates two sensors into one device.
     * It has an IR proximity sensor which is used to calculate distance and an RGB color sensor.
     *
     * There will be some variation in the values measured depending on whether you are using a
     * V3 color sensor versus the older V2 and V1 sensors, as the V3 is based around a different chip.
     *
     * For V1/V2, the light/distance sensor saturates at around 2" (5cm).  This means that targets that are 2"
     * or closer will display the same value for distAance/light detected.
     *
     * For V3, the distance sensor as configured can handle distances between 0.25" (~0.6cm) and 6" (~15cm).
     * Any target closer than 0.25" will dislay as 0.25" and any target farther than 6" will display as 6".
     *
     * Note that the distance sensor function of both chips is built around an IR proximity sensor, which is
     * sensitive to ambient light and the reflectivity of the surface against which you are measuring. If
     * very accurate distance is required you should consider calibrating the raw optical values read from the
     * chip to your exact situation.
     *
     * Although you configure a single REV Robotics Color-Distance sensor in your configuration file,
     * you can treat the sensor as two separate sensors that share the same name in your op mode.
     *
     * In this example, we represent the detected color by a hue, saturation, and value color
     * model (see https://en.wikipedia.org/wiki/HSL_and_HSV).  We change the background
     * color of the screen to match the detected color.
     *
     * In this example, we  also use the distance sensor to display the distance
     * to the target object.
     *
     */
    ColorSensor sensorColorD;
    ColorSensor sensorColorS;
    ///DistanceSensor sensorDistance;

    @Override
    public void runOpMode() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        //am configurat gyroul integrat si i-am definit unitatea de masura


        MotorDF = hardwareMap.dcMotor.get("Tright_drive");
        MotorDJ = hardwareMap.dcMotor.get("Bright_drive");
        MotorSF = hardwareMap.dcMotor.get("Tleft_drive");
        MotorSJ = hardwareMap.dcMotor.get("Bleft_drive");
        ColectorE = hardwareMap.get(DcMotor.class, "ColectorE");
        ColectorV = hardwareMap.get(DcMotor.class, "ColectorV");

        MotorSJ.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorSF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorDJ.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorDF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //am regasit printe elementele robotului conectate le hub cele 4 motoare


        MotorSF.setDirection(DcMotorSimple.Direction.REVERSE);
        MotorSJ.setDirection(DcMotorSimple.Direction.REVERSE);
        //am declarat ca motoarele de pe partea stanga se vor misca in sens invers

        MotorSF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorSJ.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorDF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorDJ.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        MotorSF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorSJ.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorDF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorDJ.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // get a reference to the color sensor.
        sensorColorD = hardwareMap.get(ColorSensor.class, "SensorColorD");
        sensorColorS = hardwareMap.get(ColorSensor.class, "SensorColorS");
        ColectorE = hardwareMap.get(DcMotor.class, "ColectorE");
        ColectorV = hardwareMap.get(DcMotor.class, "ColectorV");
        LegoStanga1 = hardwareMap.servo.get("LegoStanga1");
        LegoStanga2 = hardwareMap.servo.get("LegoStanga2");
        LegoDreapta1 = hardwareMap.servo.get("LegoDreapta1");
        LegoDreapta2 = hardwareMap.servo.get("LegoDreapta2");

        // get a reference to the distance sensor that shares the same name.
        ///sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_color_distance");

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
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        //am identificat gyroul integrat printre componentele robotului


        // wait for the start button to be pressed.
        waitForStart();

        // loop and read the RGB and distance data.
        // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        while (opModeIsActive()) {
            // convert the RGB values to HSV values.
            // multiply by the SCALE_FACTOR.
            // then cast it back to int (SCALE_FACTOR is a double)
            Color.RGBToHSV((int) (sensorColorD.red() * SCALE_FACTOR),
                    (int) (sensorColorD.green() * SCALE_FACTOR),
                    (int) (sensorColorD.blue() * SCALE_FACTOR),
                    hsvValues);
            Color.RGBToHSV((int) (sensorColorS.red() * SCALE_FACTOR),
                    (int) (sensorColorS.green() * SCALE_FACTOR),
                    (int) (sensorColorS.blue() * SCALE_FACTOR),
                    hsvValues);

            // send the info back to driver station using telemetry function.
            // telemetry.addData("Distance (cm)",
            //             String.format(Locale.US, "%.02f", sensorDistance.getDistance(DistanceUnit.CM)));
            //telemetry.addData("Alpha", sensorColorD.alpha());























            Diagonala(0.3, -2910);
            while (opModeIsActive() && !isStopRequested() && MotorSF.isBusy() && MotorSJ.isBusy() && MotorDF.isBusy() && MotorDJ.isBusy()) {
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                telemetry.addData("Heading: ", angles.firstAngle);
                telemetry.addData("Roll: ", angles.secondAngle);
                telemetry.addData("Pitch", angles.thirdAngle);
                telemetry.update();
            }
            MotorSF.setPower(0);
            MotorSJ.setPower(0);
            MotorDF.setPower(0);
            MotorDJ.setPower(0);
            sleep(500);
            LegoStanga1.setPosition(-0.8); //se lasa
            sleep(500);
            LegoStanga2.setPosition(-0.9); //se lasa
            sleep(500);
            Fata(0.3, -200);
            while (opModeIsActive() && !isStopRequested() && MotorSF.isBusy() && MotorSJ.isBusy() && MotorDF.isBusy() && MotorDJ.isBusy()) {
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                telemetry.addData("Heading: ", angles.firstAngle);
                telemetry.addData("Roll: ", angles.secondAngle);
                telemetry.addData("Pitch", angles.thirdAngle);
                telemetry.update();
            }
            MotorSF.setPower(0);
            MotorSJ.setPower(0);
            MotorDF.setPower(0);
            MotorDJ.setPower(0);
            sleep(500);
            telemetry.addData("Red  ", sensorColorD.red());
            telemetry.addData("Green", sensorColorD.green());
            telemetry.addData("Blue ", sensorColorD.blue());
            telemetry.addData("Red  ", sensorColorS.red());
            telemetry.addData("Green", sensorColorS.green());
            telemetry.addData("Blue ", sensorColorS.blue());
            telemetry.addData("Hue", hsvValues[0]);

            double r,g,b;
            r=sensorColorD.red();
            g=sensorColorD.green();
            b=sensorColorD.blue();
            if((r*g)/(b*b)<=3)//sau <=2 depinde de senzor
            { LegoStanga1.setPosition(0.9);//agata caramida
                sleep(1000);
                //LegoStanga2.setPosition(0.5); //se ridica
                //sleep(2000);
                ok=1;
            }
            if(ok==1) {
                nuvede=1;
                Fata(0.3, 530);
                while (opModeIsActive() && !isStopRequested() && MotorSF.isBusy() && MotorSJ.isBusy() && MotorDF.isBusy() && MotorDJ.isBusy()) {
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    telemetry.addData("Heading: ", angles.firstAngle);
                    telemetry.addData("Roll: ", angles.secondAngle);
                    telemetry.addData("Pitch", angles.thirdAngle);
                    telemetry.update();
                }
                MotorSF.setPower(0);
                MotorSJ.setPower(0);
                MotorDF.setPower(0);
                MotorDJ.setPower(0);
                sleep(200);

                Laterala(0.8, 5400);
                while (opModeIsActive() && !isStopRequested() && MotorSF.isBusy() && MotorSJ.isBusy() && MotorDF.isBusy() && MotorDJ.isBusy()) {
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    telemetry.addData("Heading: ", angles.firstAngle);
                    telemetry.addData("Roll: ", angles.secondAngle);
                    telemetry.addData("Pitch", angles.thirdAngle);
                    telemetry.update();
                }
                MotorSF.setPower(0);
                MotorSJ.setPower(0);
                MotorDF.setPower(0);
                MotorDJ.setPower(0);
                sleep(200);

                LegoStanga1.setPosition(-0.7);
                sleep(200);
                LegoStanga2.setPosition(0.5);//bratul mare care se ridica!!!!!;
                sleep(500);
                LegoStanga1.setPosition(0.7);
                sleep(200);

                Laterala(0.5, -3605);
                while (opModeIsActive() && !isStopRequested() && MotorSF.isBusy() && MotorSJ.isBusy() && MotorDF.isBusy() && MotorDJ.isBusy()) {
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    telemetry.addData("Heading: ", angles.firstAngle);
                    telemetry.addData("Roll: ", angles.secondAngle);
                    telemetry.addData("Pitch", angles.thirdAngle);
                    telemetry.update();
                }
                MotorSF.setPower(0);
                MotorSJ.setPower(0);
                MotorDF.setPower(0);
                MotorDJ.setPower(0);
                sleep(500);

                LegoStanga1.setPosition(-0.9); //se lasa
                sleep(200);
                LegoStanga2.setPosition(-0.9); //se lasa
                sleep(200);

                Fata(0.3, -480);
                while (opModeIsActive() && !isStopRequested() && MotorSF.isBusy() && MotorSJ.isBusy() && MotorDF.isBusy() && MotorDJ.isBusy()) {
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    telemetry.addData("Heading: ", angles.firstAngle);
                    telemetry.addData("Roll: ", angles.secondAngle);
                    telemetry.addData("Pitch", angles.thirdAngle);
                    telemetry.update();
                }
                MotorSF.setPower(0);
                MotorSJ.setPower(0);
                MotorDF.setPower(0);
                MotorDJ.setPower(0);
                sleep(200);
                LegoStanga1.setPosition(0.9); //se lasa
                sleep(500);
                Fata(0.5, 550);
                while (opModeIsActive() && !isStopRequested() && MotorSF.isBusy() && MotorSJ.isBusy() && MotorDF.isBusy() && MotorDJ.isBusy()) {
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    telemetry.addData("Heading: ", angles.firstAngle);
                    telemetry.addData("Roll: ", angles.secondAngle);
                    telemetry.addData("Pitch", angles.thirdAngle);
                    telemetry.update();
                }
                MotorSF.setPower(0);
                MotorSJ.setPower(0);
                MotorDF.setPower(0);
                MotorDJ.setPower(0);
                sleep(200);

                Laterala(0.8, 3600);
                while (opModeIsActive() && !isStopRequested() && MotorSF.isBusy() && MotorSJ.isBusy() && MotorDF.isBusy() && MotorDJ.isBusy()) {
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    telemetry.addData("Heading: ", angles.firstAngle);
                    telemetry.addData("Roll: ", angles.secondAngle);
                    telemetry.addData("Pitch", angles.thirdAngle);
                    telemetry.update();
                }
                MotorSF.setPower(0);
                MotorSJ.setPower(0);
                MotorDF.setPower(0);
                MotorDJ.setPower(0);
                sleep(200);
                LegoStanga1.setPosition(-0.9);
                sleep(200);
                LegoStanga2.setPosition(0.7);
                sleep(200);
                LegoStanga1.setPosition(0.5);
                sleep(200);
                Diagonala(0.3, -300);
                while (opModeIsActive() && !isStopRequested() && MotorSF.isBusy() && MotorSJ.isBusy() && MotorDF.isBusy() && MotorDJ.isBusy()) {
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    telemetry.addData("Heading: ", angles.firstAngle);
                    telemetry.addData("Roll: ", angles.secondAngle);
                    telemetry.addData("Pitch", angles.thirdAngle);
                    telemetry.update();
                }
                MotorSF.setPower(0);
                MotorSJ.setPower(0);
                MotorDF.setPower(0);
                MotorDJ.setPower(0);
                sleep(100);
                Laterala(1, -1000);
                while (opModeIsActive() && !isStopRequested() && MotorSF.isBusy() && MotorSJ.isBusy() && MotorDF.isBusy() && MotorDJ.isBusy()) {
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    telemetry.addData("Heading: ", angles.firstAngle);
                    telemetry.addData("Roll: ", angles.secondAngle);
                    telemetry.addData("Pitch", angles.thirdAngle);
                    telemetry.update();
                }
                break;
            }























            Fata(0.3, 50);
            while (opModeIsActive() && !isStopRequested() && MotorSF.isBusy() && MotorSJ.isBusy() && MotorDF.isBusy() && MotorDJ.isBusy()) {
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                telemetry.addData("Heading: ", angles.firstAngle);
                telemetry.addData("Roll: ", angles.secondAngle);
                telemetry.addData("Pitch", angles.thirdAngle);
                telemetry.update();
            }
            Laterala(0.3, 560);
            while (opModeIsActive() && !isStopRequested() && MotorSF.isBusy() && MotorSJ.isBusy() && MotorDF.isBusy() && MotorDJ.isBusy()) {
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                telemetry.addData("Heading: ", angles.firstAngle);
                telemetry.addData("Roll: ", angles.secondAngle);
                telemetry.addData("Pitch", angles.thirdAngle);
                telemetry.update();
            }
            telemetry.addData("Red  ", sensorColorD.red());
            telemetry.addData("Green", sensorColorD.green());
            telemetry.addData("Blue ", sensorColorD.blue());
            telemetry.addData("Red  ", sensorColorS.red());
            telemetry.addData("Green", sensorColorS.green());
            telemetry.addData("Blue ", sensorColorS.blue());
            telemetry.addData("Hue", hsvValues[0]);

            double r1,g1,b1;
            r1=sensorColorD.red();
            g1=sensorColorD.green();
            b1=sensorColorD.blue();
            if((r1*g1)/(b1*b1)<=3)//sau <=2 depinde de senzor
            { LegoStanga1.setPosition(0.9);//agata caramida
                sleep(1000);
                //LegoStanga2.setPosition(0.5); //se ridica
                //sleep(2000);
                k=1;

            }














            if(k==1)
            {nuvede=1;
                Fata(0.3, 350);
                while (opModeIsActive() && !isStopRequested() && MotorSF.isBusy() && MotorSJ.isBusy() && MotorDF.isBusy() && MotorDJ.isBusy()) {
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    telemetry.addData("Heading: ", angles.firstAngle);
                    telemetry.addData("Roll: ", angles.secondAngle);
                    telemetry.addData("Pitch", angles.thirdAngle);
                    telemetry.update();
                }
                Laterala(0.8, 4610);
                while (opModeIsActive() && !isStopRequested() && MotorSF.isBusy() && MotorSJ.isBusy() && MotorDF.isBusy() && MotorDJ.isBusy()) {
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    telemetry.addData("Heading: ", angles.firstAngle);
                    telemetry.addData("Roll: ", angles.secondAngle);
                    telemetry.addData("Pitch", angles.thirdAngle);
                    telemetry.update();
                }
                MotorSF.setPower(0);
                MotorSJ.setPower(0);
                MotorDF.setPower(0);
                MotorDJ.setPower(0);
                sleep(100);
                LegoStanga1.setPosition(-0.9);
                sleep(100);
                LegoStanga2.setPosition(0.7);
                sleep(100);
                LegoStanga1.setPosition(0.8);
                sleep(100);

                Laterala(0.5, -3000);
                while (opModeIsActive() && !isStopRequested() && MotorSF.isBusy() && MotorSJ.isBusy() && MotorDF.isBusy() && MotorDJ.isBusy()) {
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    telemetry.addData("Heading: ", angles.firstAngle);
                    telemetry.addData("Roll: ", angles.secondAngle);
                    telemetry.addData("Pitch", angles.thirdAngle);
                    telemetry.update();
                }
                MotorSF.setPower(0);
                MotorSJ.setPower(0);
                MotorDF.setPower(0);
                MotorDJ.setPower(0);
                sleep(100);

                LegoStanga1.setPosition(-0.9); //se lasa
                sleep(300);
                LegoStanga2.setPosition(-0.9); //se lasa
                sleep(300);

                Fata(0.3, -450);
                while (opModeIsActive() && !isStopRequested() && MotorSF.isBusy() && MotorSJ.isBusy() && MotorDF.isBusy() && MotorDJ.isBusy()) {
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    telemetry.addData("Heading: ", angles.firstAngle);
                    telemetry.addData("Roll: ", angles.secondAngle);
                    telemetry.addData("Pitch", angles.thirdAngle);
                    telemetry.update();
                }
                MotorSF.setPower(0);
                MotorSJ.setPower(0);
                MotorDF.setPower(0);
                MotorDJ.setPower(0);
                sleep(100);
                LegoStanga1.setPosition(0.9); //se lasa
                sleep(300);
                Fata(0.5, 550);
                while (opModeIsActive() && !isStopRequested() && MotorSF.isBusy() && MotorSJ.isBusy() && MotorDF.isBusy() && MotorDJ.isBusy()) {
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    telemetry.addData("Heading: ", angles.firstAngle);
                    telemetry.addData("Roll: ", angles.secondAngle);
                    telemetry.addData("Pitch", angles.thirdAngle);
                    telemetry.update();
                }
                MotorSF.setPower(0);
                MotorSJ.setPower(0);
                MotorDF.setPower(0);
                MotorDJ.setPower(0);
                sleep(100);

                Laterala(0.8, 3600);
                while (opModeIsActive() && !isStopRequested() && MotorSF.isBusy() && MotorSJ.isBusy() && MotorDF.isBusy() && MotorDJ.isBusy()) {
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    telemetry.addData("Heading: ", angles.firstAngle);
                    telemetry.addData("Roll: ", angles.secondAngle);
                    telemetry.addData("Pitch", angles.thirdAngle);
                    telemetry.update();
                }
                MotorSF.setPower(0);
                MotorSJ.setPower(0);
                MotorDF.setPower(0);
                MotorDJ.setPower(0);
                sleep(200);
                LegoStanga1.setPosition(-0.9);
                sleep(200);
                LegoStanga2.setPosition(0.7);
                sleep(200);
                LegoStanga1.setPosition(0.8);
                sleep(200);
                sleep(1000);
                Diagonala(0.3, -300);
                while (opModeIsActive() && !isStopRequested() && MotorSF.isBusy() && MotorSJ.isBusy() && MotorDF.isBusy() && MotorDJ.isBusy()) {
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    telemetry.addData("Heading: ", angles.firstAngle);
                    telemetry.addData("Roll: ", angles.secondAngle);
                    telemetry.addData("Pitch", angles.thirdAngle);
                    telemetry.update();
                }
                MotorSF.setPower(0);
                MotorSJ.setPower(0);
                MotorDF.setPower(0);
                MotorDJ.setPower(0);
                sleep(100);
                Laterala(1, -1500);
                while (opModeIsActive() && !isStopRequested() && MotorSF.isBusy() && MotorSJ.isBusy() && MotorDF.isBusy() && MotorDJ.isBusy()) {
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    telemetry.addData("Heading: ", angles.firstAngle);
                    telemetry.addData("Roll: ", angles.secondAngle);
                    telemetry.addData("Pitch", angles.thirdAngle);
                    telemetry.update();
                }
                break;
            }

















            if(nuvede==0) {
                Laterala(0.3, 500);
                while (opModeIsActive() && !isStopRequested() && MotorSF.isBusy() && MotorSJ.isBusy() && MotorDF.isBusy() && MotorDJ.isBusy()) {
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    telemetry.addData("Heading: ", angles.firstAngle);
                    telemetry.addData("Roll: ", angles.secondAngle);
                    telemetry.addData("Pitch", angles.thirdAngle);
                    telemetry.update();
                }
                LegoStanga1.setPosition(0.9);//agata caramida
                sleep(500);

                Fata(0.3, 350);
                while (opModeIsActive() && !isStopRequested() && MotorSF.isBusy() && MotorSJ.isBusy() && MotorDF.isBusy() && MotorDJ.isBusy()) {
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    telemetry.addData("Heading: ", angles.firstAngle);
                    telemetry.addData("Roll: ", angles.secondAngle);
                    telemetry.addData("Pitch", angles.thirdAngle);
                    telemetry.update();
                }
                Laterala(0.8, 4610);
                while (opModeIsActive() && !isStopRequested() && MotorSF.isBusy() && MotorSJ.isBusy() && MotorDF.isBusy() && MotorDJ.isBusy()) {
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    telemetry.addData("Heading: ", angles.firstAngle);
                    telemetry.addData("Roll: ", angles.secondAngle);
                    telemetry.addData("Pitch", angles.thirdAngle);
                    telemetry.update();
                }
                MotorSF.setPower(0);
                MotorSJ.setPower(0);
                MotorDF.setPower(0);
                MotorDJ.setPower(0);
                sleep(200);
                LegoStanga1.setPosition(-0.9);
                sleep(200);
                LegoStanga2.setPosition(0.7);
                sleep(200);
                LegoStanga1.setPosition(0.8);
                sleep(200);

                Laterala(0.5, -2900);
                while (opModeIsActive() && !isStopRequested() && MotorSF.isBusy() && MotorSJ.isBusy() && MotorDF.isBusy() && MotorDJ.isBusy()) {
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    telemetry.addData("Heading: ", angles.firstAngle);
                    telemetry.addData("Roll: ", angles.secondAngle);
                    telemetry.addData("Pitch", angles.thirdAngle);
                    telemetry.update();
                }
                MotorSF.setPower(0);
                MotorSJ.setPower(0);
                MotorDF.setPower(0);
                MotorDJ.setPower(0);
                sleep(200);

                LegoStanga1.setPosition(-0.9); //se lasa
                sleep(200);
                LegoStanga2.setPosition(-0.9); //se lasa
                sleep(200);

                Fata(0.3, -450);
                while (opModeIsActive() && !isStopRequested() && MotorSF.isBusy() && MotorSJ.isBusy() && MotorDF.isBusy() && MotorDJ.isBusy()) {
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    telemetry.addData("Heading: ", angles.firstAngle);
                    telemetry.addData("Roll: ", angles.secondAngle);
                    telemetry.addData("Pitch", angles.thirdAngle);
                    telemetry.update();
                }
                MotorSF.setPower(0);
                MotorSJ.setPower(0);
                MotorDF.setPower(0);
                MotorDJ.setPower(0);
                sleep(200);
                LegoStanga1.setPosition(0.9); //se lasa
                sleep(200);
                Fata(0.5, 650);
                while (opModeIsActive() && !isStopRequested() && MotorSF.isBusy() && MotorSJ.isBusy() && MotorDF.isBusy() && MotorDJ.isBusy()) {
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    telemetry.addData("Heading: ", angles.firstAngle);
                    telemetry.addData("Roll: ", angles.secondAngle);
                    telemetry.addData("Pitch", angles.thirdAngle);
                    telemetry.update();
                }
                MotorSF.setPower(0);
                MotorSJ.setPower(0);
                MotorDF.setPower(0);
                MotorDJ.setPower(0);
                sleep(100);

                Laterala(0.8, 3000);
                while (opModeIsActive() && !isStopRequested() && MotorSF.isBusy() && MotorSJ.isBusy() && MotorDF.isBusy() && MotorDJ.isBusy()) {
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    telemetry.addData("Heading: ", angles.firstAngle);
                    telemetry.addData("Roll: ", angles.secondAngle);
                    telemetry.addData("Pitch", angles.thirdAngle);
                    telemetry.update();
                }
                MotorSF.setPower(0);
                MotorSJ.setPower(0);
                MotorDF.setPower(0);
                MotorDJ.setPower(0);
                sleep(100);
                LegoStanga1.setPosition(-0.9);
                sleep(100);
                LegoStanga2.setPosition(0.7);
                sleep(100);
                LegoStanga1.setPosition(0.8);
                sleep(100);
                Laterala(1, -1300);
                while (opModeIsActive() && !isStopRequested() && MotorSF.isBusy() && MotorSJ.isBusy() && MotorDF.isBusy() && MotorDJ.isBusy()) {
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    telemetry.addData("Heading: ", angles.firstAngle);
                    telemetry.addData("Roll: ", angles.secondAngle);
                    telemetry.addData("Pitch", angles.thirdAngle);
                    telemetry.update();
                }

            }









            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
                }
            });

            telemetry.update();
        }

        // Set the panel back to the default color
        relativeLayout.post(new Runnable() {
            public void run() {
                relativeLayout.setBackgroundColor(Color.WHITE);
            }
        });
    }
    public void Fata(double p,int t){
        MotorSF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorSJ.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorDF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorDJ.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        MotorSF.setTargetPosition(-t);
        MotorSJ.setTargetPosition(-t);
        MotorDF.setTargetPosition(-t);
        MotorDJ.setTargetPosition(-t);


        MotorSF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MotorSJ.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MotorDF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MotorDJ.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        MotorSF.setPower(p);
        MotorSJ.setPower(p);
        MotorDF.setPower(p);
        MotorDJ.setPower(p);
        if((t-MotorDF.getCurrentPosition()<t-50) &&(t-MotorDJ.getCurrentPosition()<t-50) && (t-MotorSF.getCurrentPosition()<t-50)&&(t-MotorSJ.getCurrentPosition()<t-50))
        {p=0;
            MotorDF.setPower(0);
            MotorDF.setPower(0);
            MotorDF.setPower(0);
            MotorDF.setPower(0);
        }
    }
    public void Diagonala(double p,int t){
        MotorSF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorSJ.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorDF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorDJ.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        MotorSF.setTargetPosition(-t);
        MotorSJ.setTargetPosition(-t);
        MotorDF.setTargetPosition(-t);
        MotorDJ.setTargetPosition(-t);


        MotorSF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MotorSJ.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MotorDF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MotorDJ.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        MotorSF.setPower(0);
        MotorSJ.setPower(p);
        MotorDF.setPower(p);
        MotorDJ.setPower(0);
    }
    public void Laterala(double p,int t){
        MotorSF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorSJ.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorDF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorDJ.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        MotorSF.setTargetPosition(t);
        MotorSJ.setTargetPosition(-t);
        MotorDF.setTargetPosition(-t);
        MotorDJ.setTargetPosition(t);


        MotorSF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MotorSJ.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MotorDF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MotorDJ.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        MotorSF.setPower(p);
        MotorSJ.setPower(p);
        MotorDF.setPower(p);
        MotorDJ.setPower(p);
        if(t>0) {
            if ((t - MotorDF.getCurrentPosition() > (t - 50)) && (t - MotorDJ.getCurrentPosition() < (t - 50)) && (t - MotorSF.getCurrentPosition() < (t - 50)) && (t - MotorSJ.getCurrentPosition() > (t - 50))) {
                p = 0;
                MotorDF.setPower(0);
                MotorDF.setPower(0);
                MotorDF.setPower(0);
                MotorDF.setPower(0);
            }
        }
        if(t<0) {
            if ((t - MotorDF.getCurrentPosition() < (t - 50)) && (t - MotorDJ.getCurrentPosition() > (t - 50)) && (t - MotorSF.getCurrentPosition() > (t - 50)) && (t - MotorSJ.getCurrentPosition() < (t - 50))) {
                p = 0;
                MotorDF.setPower(0);
                MotorDF.setPower(0);
                MotorDF.setPower(0);
                MotorDF.setPower(0);
            }
        }
    }
}