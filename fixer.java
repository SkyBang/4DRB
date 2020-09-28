package org.firstinspires.ftc.teamcode;


import android.database.CursorIndexOutOfBoundsException;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@Autonomous(name= "fixer")
public class fixer extends LinearOpMode {

    public DcMotor MotorDF = null;
    public DcMotor MotorSF = null;
    public DcMotor MotorDJ = null;
    public DcMotor MotorSJ = null;
    public Servo Servomotor1 = null;
    public Servo Servomotor2 = null;
    public Servo ServomotorE = null;
    public Servo ServomotorV = null;
    BNO055IMU imu;
    Orientation angles;

    @Override
    public void runOpMode() {

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        //am configurat gyroul integrat si i-am definit unitatea de masura


        MotorDF = hardwareMap.dcMotor.get("Tright_drive");
        MotorDJ = hardwareMap.dcMotor.get("Bright_drive");
        MotorSF = hardwareMap.dcMotor.get("Tleft_drive");
        MotorSJ = hardwareMap.dcMotor.get("Bleft_drive");

        MotorSJ.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorSF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorDJ.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorDF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //am regasit printe elementele robotului conectate le hub cele 4 motoare

        Servomotor1 = hardwareMap.servo.get("Servo1");
        Servomotor2 = hardwareMap.servo.get("Servo2");
        ServomotorE = hardwareMap.servo.get("Servo_spateE");
        ServomotorV = hardwareMap.servo.get("Servo_spateV");
        //servomotorul
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
      /*  MotorSF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorSJ.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorDF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorDJ.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);*/

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

       /* MotorSF.setPower(0.3);
        MotorSJ.setPower(0.3);
        MotorDF.setPower(0.3);
        MotorDJ.setPower(0.3);

        sleep(1500);
        MotorSF.setPower(0);
        MotorSJ.setPower(0);
        MotorDF.setPower(0);
        MotorDJ.setPower(0);*/
       Test_Encoder(0.3,1500);
        while(MotorDJ.isBusy() && opModeIsActive()) {
            //Loop body can be empty
            telemetry.addData("encoder_df",MotorSF.getCurrentPosition());
            telemetry.addData("encoder_df",MotorSJ.getCurrentPosition());
            telemetry.addData("encoder_df",MotorDF.getCurrentPosition());
            telemetry.addData("encoder_df",MotorDJ.getCurrentPosition());
            telemetry.update();
        }

    }
    public void Test_Encoder (double p,int t){
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



    }
}
