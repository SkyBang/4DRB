package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import java.util.Locale;

@Autonomous(name= "ValceaAlbastruCaramidaGyro",group = "Sensor")
public class ValceaAlbastruCaramidaGyro extends LinearOpMode {

    public DcMotor MotorDF = null;
    public DcMotor MotorSF = null;
    public DcMotor MotorDJ = null;
    public DcMotor MotorSJ = null;
    private DcMotor ColectorE = null;
    private DcMotor ColectorV = null;


    DigitalChannel digitalTouch;  // Hardware Device Object

    BNO055IMU imu;
    Orientation angles;
    // State used for updating telemetry
    Acceleration gravity;

    @Override
    public void runOpMode() {
        digitalTouch = hardwareMap.get(DigitalChannel.class, "sensor_digital");

        // set the digital channel to input.
        digitalTouch.setMode(DigitalChannel.Mode.INPUT);

        //am configurat gyroul integrat si i-am definit unitatea de masura

        MotorDF = hardwareMap.dcMotor.get("Tright_drive");
        MotorDJ = hardwareMap.dcMotor.get("Bright_drive");
        MotorSF = hardwareMap.dcMotor.get("Tleft_drive");
        MotorSJ = hardwareMap.dcMotor.get("Bleft_drive");
        double TurnPower=0.3;
        float zAccumulated;
        int target;
        int t;
        int ok=0;

        ColectorE = hardwareMap.get(DcMotor.class, "ColectorE");
        ColectorV = hardwareMap.get(DcMotor.class, "ColectorV");

        MotorSJ.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorSF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorDJ.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorDF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //am regasit printe elementele robotului conectate le hub cele 4 motoare


        MotorSF.setDirection(DcMotorSimple.Direction.REVERSE);
        MotorSJ.setDirection(DcMotorSimple.Direction.REVERSE);
        //am declarat ca motoarele de pe partea stanga se vormisca in sens invers

        MotorSF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorSJ.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorDF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorDJ.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        MotorSF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorSJ.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorDF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorDJ.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Set up our telemetry dashboard
        composeTelemetry();

        // Wait until we're told to go
        waitForStart();
        sleep(2000);

        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        // Loop and update the dashboard
        //se reseteaza valorile encoderelor pentru cele doua motoare de stanga la 0

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        //am identificat gyroul integrat printre componentele robotului


        telemetry.addData("Mode", "Waiting");
        telemetry.update();
        //am actuaizat datele din telemetrie

        waitForStart();
        telemetry.addData("Mode", "running");
        telemetry.update();
        ColectorE.setPower(0.3);
        ColectorV.setPower(0.3);
        Laterala(0.5,2560);
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

        Fata(0.3, 500);
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

        if(digitalTouch.getState() != true)
        {ColectorE.setPower(0);
            ColectorV.setPower(0);
        }

        ColectorE.setPower(0);
        ColectorV.setPower(0);
        //a fost -1150
        Laterala(0.5,-850);
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


        Fata(0.5, -2600);
        while (opModeIsActive() && !isStopRequested() && MotorSF.isBusy() && MotorSJ.isBusy() && MotorDF.isBusy() && MotorDJ.isBusy()) {
            angles = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("Heading: ", angles.firstAngle);

            telemetry.addData("Roll: ", angles.secondAngle);
            telemetry.addData("Pitch", angles.thirdAngle);
            telemetry.update();
        }
        MotorSF.setPower(0);
        MotorSJ.setPower(0);
        MotorDF.setPower(0);
        MotorDJ.setPower(0);
        target=55;
        while((target-angles.firstAngle)>3.75)
        {
            MotorSF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            MotorSJ.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            MotorDF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            MotorDJ.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


            MotorDF.setPower(-TurnPower);
            MotorDJ.setPower(-TurnPower);
            MotorSF.setPower(TurnPower);
            MotorSJ.setPower(TurnPower);
            telemetry.addData(" zacc:",angles.firstAngle);
            telemetry.addData(" target", target);
            telemetry.update();
        }
        sleep(500);
        ColectorE.setPower(-0.7);
        ColectorV.setPower(-0.7);
        sleep(500);
        t=33;
        while((t-angles.firstAngle)<3.75)
        {
            MotorSF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            MotorSJ.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            MotorDF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            MotorDJ.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


            MotorDF.setPower(TurnPower);
            MotorDJ.setPower(TurnPower);
            MotorSF.setPower(-TurnPower);
            MotorSJ.setPower(-TurnPower);
            telemetry.addData("zacc2:",angles.firstAngle);
            telemetry.addData("t2", t);
            telemetry.update();
        }
        sleep(200);

        Fata(0.7, 700);
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

    }

    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity  = imu.getGravity();
        }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override public String value() {
                        return gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel*gravity.xAccel
                                        + gravity.yAccel*gravity.yAccel
                                        + gravity.zAccel*gravity.zAccel));
                    }
                });
        //double varx=(double).formatAngle(angles.angleUnit, angles.secondAngle);

    }

    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    public void Laterala(double p,int t){
        MotorSF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorSJ.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorDF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorDJ.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        MotorSF.setTargetPosition(t);
        MotorSJ.setTargetPosition(-t);//-
        MotorDF.setTargetPosition(-t);//-
        MotorDJ.setTargetPosition(t);


        MotorSF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MotorSJ.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MotorDF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MotorDJ.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        MotorSF.setPower(p);
        MotorSJ.setPower(p);
        MotorDF.setPower(p);
        MotorDJ.setPower(p);
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
    }

    public void Rotire(double p,int t){
        MotorSF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorSJ.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorDF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorDJ.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        MotorSF.setTargetPosition(t);
        MotorSJ.setTargetPosition(t);
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
    }
}