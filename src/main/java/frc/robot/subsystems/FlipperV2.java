package frc.robot.subsystems;


import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Constants.FlipperConstants;



public class FlipperV2 extends SubsystemBase {

    //declarations
    CANSparkMax flipper; //flipper represents SparkMax
    AbsoluteEncoder flipperEnc; //flipper encoder type
    Shoulder shoulder;
    private PID flipperControl;

    //dynamic variables
    double flipperPos = 0;
    double flipperSetPos = 0;



    public FlipperV2(Shoulder m_shoulder){

        shoulder = m_shoulder; //shoulder to shoulder????


        //create flipper controller
        flipper = new CANSparkMax(FlipperConstants.flipperCanId, MotorType.kBrushless); 
        flipper.restoreFactoryDefaults(); //reset
        flipperEnc = flipper.getAbsoluteEncoder(Type.kDutyCycle); //configure encoder type
        
        DriverDisplay.flipperPos.setDouble(getFlipperPos());

        //software limit 
        //flipper.setSoftLimit(SoftLimitDirection.kForward, 90);
        //flipper.setSoftLimit(SoftLimitDirection.kReverse, 0);
        flipper.setSmartCurrentLimit(40, 40); //current limit
        flipper.setIdleMode(IdleMode.kBrake);//control type
        DriverDisplay.flipperSetpoint.setDouble(flipperSetPos); //current pos

        flipper.burnFlash();//flash settings on to controller
        DriverDisplay.appliedOutput.setDouble(flipper.getAppliedOutput()); //output
        DriverDisplay.busVoltage.setDouble(flipper.getBusVoltage()); //output bus voltage

        //Creat PID controller algorithm
        flipperControl = new PID(FlipperConstants.kFlipperP, FlipperConstants.kFlipperI, FlipperConstants.kFlipperD, 0, FlipperConstants.kMaxVelocity, FlipperConstants.kMaxAcceleration);
        flipperControl.setNewPoint(0); //home flipper
    }

   //Simplified functions ready too use

   //get flipper's current position
   public double getFlipperPos() {
    return flipperEnc.getPosition()*360; //manualy converting rotations to degrees and save it
   }
   //get current set position
  public double getFlipperSetpoint() {
    return flipperSetPos;
  }

  //why? we already have velocity limit on PID controller
  public void setFlipperSpeed(double speed) {
    flipper.set(speed);
  }

  //updates flipper new set position
  public void moveFlipperToPos(double degrees) {
    flipperSetPos = degrees;

  }




  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    flipperControl.setNewPoint(flipperSetPos); //set new position
    flipperControl.calculatePID(); //run PID periodicaly, and calculate output
    setFlipperSpeed(flipperControl.getPID()); //set motor speed (basicaly voltage but indirect) based on pid output
    flipperPos = getFlipperPos(); //update encoder position, save into flipperpos 

    DriverDisplay.flipperPos.setDouble(getFlipperPos());
    DriverDisplay.busVoltage.setDouble(flipper.getBusVoltage()); //output bus voltage
    DriverDisplay.appliedOutput.setDouble(flipper.getAppliedOutput()); //output   




    if (Constants.CODEMODE == Constants.MODES.TEST) {
      DriverDisplay.flipperSetpoint.setDouble(flipperSetPos);
      double tempSetpoint = DriverDisplay.flipperSetpoint.getDouble (flipperSetPos);
      if (flipperSetPos != tempSetpoint) {
        moveFlipperToPos(tempSetpoint);
      }
    }
  }
}
