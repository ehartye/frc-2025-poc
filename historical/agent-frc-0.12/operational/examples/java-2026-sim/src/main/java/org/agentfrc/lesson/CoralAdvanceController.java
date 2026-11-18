package org.agentfrc.lesson;
import edu.wpi.first.wpilibj.*;import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
/** Offline finite-tube advance experiment; beam-triggered HOLD is not scoring or retention proof. */
public final class CoralAdvanceController implements AutoCloseable {
 public enum Phase{FEED,HOLD,FAULT}private final Encoder encoder;private final DigitalInput beam;private final PWMSparkMax motor;private Phase phase=Phase.FEED;private double previous,rate,time;private boolean initialized;
 public CoralAdvanceController(){if(RobotBase.isReal())throw new IllegalStateException("Offline only");encoder=new Encoder(0,1);encoder.setDistancePerPulse(2*Math.PI/2048);beam=new DigitalInput(2);motor=new PWMSparkMax(0);}
 public void periodic(boolean enabled){if(!enabled||RobotController.getBatteryVoltage()<6)phase=Phase.FAULT;if(phase==Phase.FAULT||phase==Phase.HOLD){motor.set(0);return;}double angle=encoder.getDistance();rate=(angle-previous)/.02;previous=angle;time+=.02;boolean blocked=!beam.get();if(!initialized){initialized=true;if(blocked)phase=Phase.FAULT;}if(!enabled||RobotController.getBatteryVoltage()<6||time>3||!Double.isFinite(rate)||rate>1)phase=Phase.FAULT;if(phase!=Phase.FAULT&&blocked)phase=Phase.HOLD;motor.set(phase==Phase.FEED?-.12:0);}
 public Phase phase(){return phase;}public double rate(){return rate;}
 @Override public void close(){motor.set(0);motor.close();encoder.close();beam.close();}
}

