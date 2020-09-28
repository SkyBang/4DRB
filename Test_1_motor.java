package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@Autonomous(name= "Test 1 Motor")
public class Test_1_motor extends LinearOpMode {

    public DcMotor MotorDF = null;

    BNO055IMU imu;
    Orientation angles;

    @Override
    public void runOpMode() {

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        //am configurat gyroul integrat si i-am definit unitatea de masura


        MotorDF = hardwareMap.dcMotor.get("Tright_drive");



        MotorDF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //am regasit printe elementele robotului conectate le hub cele 4 motoare

        MotorDF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        MotorDF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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
        MotorDF_fata(0.3,450);
       sleep(500);
        MotorDF.setPower(0);

        sleep(500);


    }

    public void MotorDF_fata(double p,int t) {

        MotorDF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        MotorDF.setTargetPosition(-t);


        MotorDF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MotorDF.setPower(p);


    }
}