package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="TeleOpNou", group="Linear Opmode")

public class  TeleOpNou extends LinearOpMode {
    private int speedAdjust = 8;
    boolean isPulling = false, clamped = false, prevX = false, prevLeft = false, prevRight = false, prevB = false, rotated = false, prevA = false;
    double speed, robotAngle, realign = 0;
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor TleftDrive = null;
    private DcMotor BleftDrive = null;
    private DcMotor TrightDrive = null;
    private DcMotor BrightDrive = null;
    private Servo LegoStanga1 = null;
    private  Servo LegoStanga2 = null;
    private Servo LegoDreapta1 = null;
    private  Servo LegoDreapta2 = null;
    private  Servo Grip = null;
    private DcMotor ColectorE = null;
    private DcMotor ColectorV = null;
    private DcMotor CulisataOrizontala=null;
    private DcMotor CulisantaVerticala=null;
    DigitalChannel  TouchDreapta;
    DigitalChannel  TouchStanga;
    double ok=0;
    double  power   = 1;
    double FranaS,FranaD;
    public Servo ServomotorE = null;
    public Servo ServomotorV = null;
    double k=0;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        TleftDrive  = hardwareMap.get(DcMotor.class, "Tleft_drive");
        TrightDrive = hardwareMap.get(DcMotor.class, "Tright_drive");
        BleftDrive  = hardwareMap.get(DcMotor.class, "Bleft_drive");
        BrightDrive = hardwareMap.get(DcMotor.class, "Bright_drive");
        ColectorE = hardwareMap.get(DcMotor.class, "ColectorE");
        ColectorV = hardwareMap.get(DcMotor.class, "ColectorV");
        CulisataOrizontala= hardwareMap.get(DcMotor.class, "CulisantaOrizontala");
        CulisantaVerticala= hardwareMap.get(DcMotor.class, "CulisantaVerticala");
        ServomotorE = hardwareMap.servo.get("Servo_SpateE");
        ServomotorV = hardwareMap.servo.get("Servo_SpateV");
        LegoStanga1 = hardwareMap.servo.get("LegoStanga1");
        LegoStanga2 = hardwareMap.servo.get("LegoStanga2");
        LegoDreapta1 = hardwareMap.servo.get("LegoDreapta1");
        LegoDreapta2 = hardwareMap.servo.get("LegoDreapta2");
        Grip=hardwareMap.servo.get("Grip");

        TleftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        TrightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BrightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BleftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ColectorE.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ColectorV.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);




        TleftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        BleftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        TouchStanga = hardwareMap.get(DigitalChannel.class, "TouchStanga");
        TouchDreapta = hardwareMap.get(DigitalChannel.class, "TouchDreapta");
        // set the digital channel to input.
        TouchStanga.setMode(DigitalChannel.Mode.INPUT);
        TouchDreapta.setMode(DigitalChannel.Mode.INPUT);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if(ok==0)
            {TleftDrive.setPower(0);
                BleftDrive.setPower(0);
                TrightDrive.setPower(0);
                BrightDrive.setPower(0);
                ok=1;
            }
            telemetry.update();
            ColectorE.setPower(0.7);
            ColectorV.setPower(0.7);
            while (gamepad1.dpad_up) {
                TleftDrive.setPower(-power);
                BleftDrive.setPower(-power);
                TrightDrive.setPower(-power);
                BrightDrive.setPower(-power);

                CurbaFata(power, FranaD, FranaS);

                if (gamepad1.start)
                    power = 0.5;
                if (!gamepad1.dpad_up) {
                    TleftDrive.setPower(0);
                    BleftDrive.setPower(0);
                    TrightDrive.setPower(0);
                    BrightDrive.setPower(0);
                    power = 1;
                    break;
                }
            }
            telemetry.update();
            while (gamepad1.dpad_down) {
                TleftDrive.setPower(power);
                BleftDrive.setPower(power);
                TrightDrive.setPower(power);
                BrightDrive.setPower(power);

                CurbaSpate(power, FranaD, FranaS);

                if (gamepad1.start)
                    power = 1;
                if (!gamepad1.dpad_down) {
                    TleftDrive.setPower(0);
                    BleftDrive.setPower(0);
                    TrightDrive.setPower(0);
                    BrightDrive.setPower(0);
                    power = 1;
                    break;
                }
            }
            telemetry.update();

            while (gamepad1.dpad_left) {
                TleftDrive.setPower(power);
                BleftDrive.setPower(-power);
                TrightDrive.setPower(-power);
                BrightDrive.setPower(power);
                if (gamepad1.start)
                    power = 0.3;
                if (!gamepad1.start)
                    power = 1;
                if (!gamepad1.dpad_left) {
                    TleftDrive.setPower(0);
                    BleftDrive.setPower(0);
                    TrightDrive.setPower(0);
                    BrightDrive.setPower(0);
                    power = 1;
                    break;
                }
            }
            telemetry.update();

            while (gamepad1.dpad_right) {
                TleftDrive.setPower(-power);
                BleftDrive.setPower(power);
                TrightDrive.setPower(power);
                BrightDrive.setPower(-power);
                if (gamepad1.start)
                    power = 0.3;
                if (!gamepad1.start)
                    power = 1;
                if (!gamepad1.dpad_right) {
                    TleftDrive.setPower(0);
                    BleftDrive.setPower(0);
                    TrightDrive.setPower(0);
                    BrightDrive.setPower(0);
                    power = 1;
                    break;
                }
            }
            telemetry.update();
            while (gamepad1.y) {
                TleftDrive.setPower(1);
                BrightDrive.setPower(1);
                if (!(gamepad1.y)) {
                    TleftDrive.setPower(0);
                    BrightDrive.setPower(0);
                    break;
                }
            }
            while (gamepad1.a) {
                BleftDrive.setPower(1);
                TrightDrive.setPower(1);
                if (!(gamepad1.a)) {
                    BleftDrive.setPower(0);
                    TrightDrive.setPower(0);
                    break;
                }
            }
            telemetry.update();
            while (gamepad1.x) {
                TleftDrive.setPower(-1);
                BrightDrive.setPower(-1);
                if (!(gamepad1.x)) {
                    TleftDrive.setPower(0);
                    BrightDrive.setPower(0);
                    break;
                }
            }
            while (gamepad1.b) {
                BleftDrive.setPower(-1);
                TrightDrive.setPower(-1);
                if (!(gamepad1.b)) {
                    BleftDrive.setPower(0);
                    TrightDrive.setPower(0);
                    break;
                }
            }
            while (gamepad1.left_bumper) {
                TleftDrive.setPower(1);
                BleftDrive.setPower(1);
                TrightDrive.setPower(-1);
                BrightDrive.setPower(-1);
                if (!gamepad1.left_bumper) {
                    TleftDrive.setPower(0);
                    BleftDrive.setPower(0);
                    TrightDrive.setPower(0);
                    BrightDrive.setPower(0);
                    break;
                }
            }
            while (gamepad1.right_bumper) {
                TleftDrive.setPower(-1);
                BleftDrive.setPower(-1);
                TrightDrive.setPower(1);
                BrightDrive.setPower(1);
                if (!gamepad1.right_bumper) {
                    TleftDrive.setPower(0);
                    BleftDrive.setPower(0);
                    TrightDrive.setPower(0);
                    BrightDrive.setPower(0);
                    break;
                }
            }
            while(gamepad2.right_bumper)
            {
                CulisataOrizontala.setPower(0.8);
                if(!gamepad2.right_bumper)
                {
                    CulisataOrizontala.setPower(0);
                    break;
                }
            }
            while(gamepad2.left_bumper)
            {
                CulisataOrizontala.setPower(-0.8);
                if(!gamepad2.left_bumper)
                {
                    CulisataOrizontala.setPower(0);
                    break;
                }
            }
            if(gamepad2.dpad_left)
            {
                Grip.setPosition(-0.3);
            }
            if(gamepad2.dpad_right)
            {
                Grip.setPosition(0.3);
            }
            while(gamepad2.dpad_up)
            {
                if (TouchDreapta.getState() == true) {
                    telemetry.addData("TouchDreapta", "Is Not Pressed");
                    CulisantaVerticala.setPower(-1);
                }
                else
                    telemetry.addData("TouchDreapta", "Is Pressed");
                CulisantaVerticala.setPower(0);
                telemetry.update();
            }
            while(gamepad2.dpad_down)
            {
                if (TouchStanga.getState() == true) {
                    telemetry.addData("TouchStanga", "Is Not Pressed");
                    CulisantaVerticala.setPower(0.8);
                }
                else
                    telemetry.addData("TouchStanga", "Is Pressed");
                CulisantaVerticala.setPower(0);
                telemetry.update();
            }
            //controls speed adjusting
            if(gamepad1.dpad_left && !prevLeft) speedAdjust--;
            if(gamepad1.dpad_right && !prevRight) speedAdjust++;
            prevLeft = gamepad1.dpad_left;
            prevRight = gamepad1.dpad_right;

            //sets clip that attaches to the foundation

            prevX = gamepad1.x;


            //uses the hypotenuse of left joystick and right joystick to calculate the speed of the robot
            speed = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);

            //finds the angle the robot is moving at
            //robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) + (angles.firstAngle + Math.PI + angles2.firstAngle + Math.PI) / 2 - Math.PI / 4 + realign;
            robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            //finds the percent of power to each wheel and multiplies it by the speed
            TleftDrive.setPower(speed * Math.sin(robotAngle) - gamepad1.right_stick_x);
            TrightDrive.setPower(speed * Math.cos(robotAngle) + gamepad1.right_stick_x);
            BleftDrive.setPower(speed * Math.cos(robotAngle) - gamepad1.right_stick_x);
            BrightDrive.setPower(speed * Math.sin(robotAngle) + gamepad1.right_stick_x);
            telemetry.update();

        }
    }

    public void CurbaFata(double power,double FranaD,double FranaS){
        FranaD=gamepad1.right_trigger;
        FranaS=gamepad1.left_trigger;

        if (gamepad1.left_trigger != 0) {
            TleftDrive.setPower(-(power - FranaS));
            BleftDrive.setPower(-(power - FranaS));
        }
        if(gamepad1.right_trigger!=0){
            TrightDrive.setPower(-(power-FranaD));
            BrightDrive.setPower(-(power-FranaD));
        }

    }
    public void CurbaSpate(double power,double FranaD,double FranaS){
        FranaD=gamepad1.right_trigger;
        FranaS=gamepad1.left_trigger;

        if (gamepad1.left_trigger != 0) {
            TleftDrive.setPower(power - FranaS);
            BleftDrive.setPower(power - FranaS);
        }
        if(gamepad1.right_trigger!=0){
            TrightDrive.setPower(power-FranaD);
            BrightDrive.setPower(power-FranaD);
        }

    }
}