// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

/*^install vendor library when ctre does funny things: https://maven.ctr-electronics.com/release/com/ctre/phoenix/Phoenix-frc2022-latest.json
if doesn't work, restart vs code
if again doesn't work, uninstall ctre library from vendors, then restart vs code
if again doesn't work, panic*/
//rev library link: https://software-metadata.revrobotics.com/REVLib.json
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  CANSparkMax l1 = new CANSparkMax(RobotMap.L1CANID, MotorType.kBrushless);
  CANSparkMax l2 = new CANSparkMax(RobotMap.L2CANID, MotorType.kBrushless);
  //CANSparkMax l3 = new CANSparkMax(RobotMap.L3CANID, MotorType.kBrushless);
  CANSparkMax r1 = new CANSparkMax(RobotMap.R1CANID, MotorType.kBrushless);
  CANSparkMax r2 = new CANSparkMax(RobotMap.R2CANID, MotorType.kBrushless);
  //CANSparkMax r3 = new CANSparkMax(RobotMap.R3CANID, MotorType.kBrushless);

  MotorControllerGroup l = new MotorControllerGroup(l1, l2);
  MotorControllerGroup r = new MotorControllerGroup(r1, r2);

  DifferentialDrive drive = new DifferentialDrive(l,r);

  XboxController driver = new XboxController(0);
  XboxController operator = new XboxController(1);

  //again, bad names; change once get function
  TalonSRX shootIntake = new TalonSRX(RobotMap.SHOOTINTAKEID);
  TalonSRX indexer = new TalonSRX(RobotMap.INDEXID);
  TalonSRX intake = new TalonSRX(RobotMap.INTAKEID);
  TalonSRX drIntake = new TalonSRX(RobotMap.DRINTAKEID);

  //climb motors
  TalonSRX climbA = new TalonSRX(RobotMap.CLIMBAID);
  TalonSRX climbB = new TalonSRX(RobotMap.CLIMBBID);

  
  //S shooter
  CANSparkMax shooter = new CANSparkMax(RobotMap.SHOOTID, MotorType.kBrushless);
  

  //leds
  Spark led = new Spark(RobotMap.BLINKINPORT);

  //auto 
  Timer timer = new Timer();

  //gyro
  ADXRS450_Gyro gyro = new ADXRS450_Gyro();
  double i = 0;
  double d = 0;
  double p = Math.pow(0.5, 9);
  PIDController PID = new PIDController(p, i, d);

  //pneumatics
  DoubleSolenoid solfor = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, RobotMap.SOLFORCHANNEL1, RobotMap.SOLFORCHANNEL2);
  Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    gyro.calibrate();
    gyro.reset();
    startCompressor();
  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {}

  @Override
  // what is this vscode theme
  public void autonomousPeriodic() {
    if(timer.get()<RobotMap.AUTOTIME1){
      drive.arcadeDrive(RobotMap.AUTODRIVESPEED1, RobotMap.AUTODRIVETURN1);
    }
    if(timer.get()<RobotMap.AUTOTIME2 && timer.get()>RobotMap.AUTOTIME1){
      
    }
    if(timer.get()<RobotMap.AUTOTIME3 && timer.get()>RobotMap.AUTOTIME2){
      
    }
    if(timer.get()>RobotMap.AUTOTIME3){

    }
  }

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    speedButtons();
    leds();
    //^important
    deployRetractIntake();
    intake();
    gearShifting();
    deployClimber();
    shootIntake();
    indexer();
    shoot();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  public void shootIntake(){
    if(operator.getRawButton(1)){
      shootIntake.set(ControlMode.PercentOutput, -0.5);

    }
    if(operator.getRawButton(4)){
      shootIntake.set(ControlMode.PercentOutput, 0.5);

    }
    if(!operator.getRawButton(1) && !operator.getRawButton(4)){
      shootIntake.set(ControlMode.PercentOutput, 0.0);
    }
  }
  public void indexer(){
    if(operator.getRawButton(2)){
      indexer.set(ControlMode.PercentOutput, 0.5);

    }
    if(!operator.getRawButton(2)){
      indexer.set(ControlMode.PercentOutput, 0.0);
    }
  }
  public void shoot(){
    if(operator.getRawButton(3)){
      shooter.set(0.65);
    }
    if(!operator.getRawButton(3)){
      shooter.set(0.0);
    }
  }

  public void deployClimber(){
    if (timer.get()>=120){
      if(operator.getRawButton(RobotMap.CLIMBERBUTTON)){
        //change magnitude once test
        climbA.set(ControlMode.PercentOutput, 0.4);
        climbB.set(ControlMode.PercentOutput, 0.4);
      }
      if(!operator.getRawButton(RobotMap.CLIMBERBUTTON)){
        climbA.set(ControlMode.PercentOutput, 0);
        climbB.set(ControlMode.PercentOutput, 0);
      }
    }
  }

  public void intake(){
    if(operator.getRawButton(RobotMap.INTAKEBUTTON)){
      intake.set(ControlMode.PercentOutput, .8);
    }
    if(!operator.getRawButton(RobotMap.INTAKEBUTTON)){
      intake.set(ControlMode.PercentOutput, 0);
    }
  }

  public void deployRetractIntake(){
    if(operator.getRawAxis(RobotMap.DEPLOYINTAKEAXIS)>0){
      drIntake.set(ControlMode.PercentOutput, operator.getRawAxis(RobotMap.DEPLOYINTAKEAXIS));
    }
    if(operator.getRawAxis(RobotMap.DEPLOYINTAKEAXIS)==0){
      drIntake.set(ControlMode.PercentOutput, 0);
    }
    if(operator.getRawAxis(RobotMap.RETRACTINTAKEAXIS)>0){
      drIntake.set(ControlMode.PercentOutput, -operator.getRawAxis(RobotMap.RETRACTINTAKEAXIS));
    }
  }

  public void leds(){
    if(driver.getPOV() == 0){
      //up red
      led.set(0.61); 
  }
  if(driver.getPOV() == 180){
    //down blue
    led.set(0.87);
  }
  if(driver.getPOV() == 90){
    //right "twinkles ocean palette"
    led.set(-0.51);
  }
  if(driver.getPOV() == 270){
    //left "twinkles lava palette"
    led.set(-0.49);
  }
  }

  public void speedButtons(){
    //verified; WORK
    //slow button for xbox controller
   if(driver.getRawButton(3)){
    drive.arcadeDrive(-driver.getRawAxis(0) * 0.2, -driver.getRawAxis(3) * 0.2);
    if(driver.getRawAxis(2) > 0){
    drive.arcadeDrive(-driver.getRawAxis(0) * 0.2, driver.getRawAxis(2) * 0.2);
  }
 }

 //fast button for xbox controller
 else if(driver.getRawButton(1)){
  drive.arcadeDrive(-driver.getRawAxis(0), -driver.getRawAxis(3));
    if(driver.getRawAxis(2) > 0){
      drive.arcadeDrive(-driver.getRawAxis(0), driver.getRawAxis(2));
  }
 }
 

 //default condition for neither buttons active
 else if(!driver.getRawButton(3) || !driver.getRawButton(1)){
  drive.arcadeDrive(-driver.getRawAxis(0) * 0.8, -driver.getRawAxis(3) * 0.8);
  if(driver.getRawAxis(2) > 0){
    drive.arcadeDrive(-driver.getRawAxis(0) * 0.8, driver.getRawAxis(2) * 0.8);
  }
 }  
  }

  //gyro method
  public void turnTo(double targetAngle, double targetSpeed){
    //angle = 0-2^16
    PID.setSetpoint(targetAngle);
  PID.setTolerance(3, 0.1);
  double turn = PID.calculate(gyro.getAngle());
  if(PID.getPositionError()>3){
    drive.tankDrive(turn, -turn);
  }else{
    drive.arcadeDrive(targetSpeed, turn);
  }
  } 

  public void gearShifting(){
    solfor.set(Value.kOff);
    if(driver.getRawButton(RobotMap.SOLFORONBUTTON)){
      solfor.set(Value.kForward);
    } else if(driver.getRawButton(RobotMap.SOLFOROFFBUTTON)){
      solfor.set(Value.kReverse);
    }
  }

  public void startCompressor(){
    compressor.enableDigital();
  }

  /*public void s_shooter(){
    if(operator.getRawButton(RobotMap.INTAKEBUTTON)){
      sorting.set(ControlMode.PercentOutput, 0.1);
      indexer.set(ControlMode.PercentOutput, -0.1);
    }

    if(operator.getRawButton(RobotMap.SHOOTERBUTTON)){
      fullSpeedShooter.set(1);
    }

    if(operator.getRawButton(RobotMap.OPERATORINDEXERBUTTON)){
      indexer.set(ControlMode.PercentOutput, 0.1);
    }

    if(!operator.getRawButton(RobotMap.INTAKEBUTTON) && !operator.getRawButton(RobotMap.SHOOTERBUTTON) && !operator.getRawButton(RobotMap.OPERATORINDEXERBUTTON)){
      sorting.set(ControlMode.PercentOutput, 0.0);
      indexer.set(ControlMode.PercentOutput, 0.0);
      fullSpeedShooter.set(0.0);
    }
  }*/

  public void reverse_intake(){}

  public void henryGyroTest(){
    
  }
}

// strong acids

// perchloric acid hclo3
// hydrochloric acid hcl
// hydrobromic acid hbr
// hydroiodic acid hi
// nitric acid hno3
// sulfuric acid h2so4  

// strong bases

// lithium hydroxide lioh
// sodium hydroxide naoh
// calcium hydroxide caoh
// barium hydroxide baoh
// potassium hydroxide poh
// strontium hydroxide sroh

//reviewing chem rn 
/*good for you henry
(henry doesn't know how to use comments)*/