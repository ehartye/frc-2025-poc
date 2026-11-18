package org.agentfrc.lesson;
import edu.wpi.first.wpilibj.simulation.*;
/** Axis-X and Y0 are external constraints. Hollow tube intersects a vertical ray through its upper/lower walls. */
public final class CoralContactRig implements AutoCloseable {
 public final CoralAdvanceController controller;private final CoralContactPlant plant;private final EncoderSim encoder;private final DIOSim beam;private final PWMSim pwm;private final double length,scale;private final int beamFault;private CoralContactPlant.Sample last;
 public CoralContactRig(CoralContactPlant.Config c,double scale){this(c,scale,0);} public CoralContactRig(CoralContactPlant.Config c,double scale,int beamFault){if(beamFault<0||beamFault>2)throw new IllegalArgumentException("Beam fault domain");this.beamFault=beamFault;if(scale!=1&&scale!=-1)throw new IllegalArgumentException("Declared scale fault");this.scale=scale;length=c.length();plant=new CoralContactPlant(c,-.160,.05715,.15);controller=new CoralAdvanceController();encoder=EncoderSim.createForChannel(0);beam=new DIOSim(2);pwm=new PWMSim(0);last=plant.sample(0);inject();}
 private void inject(){encoder.setCount((int)Math.round(last.angle()/(2*Math.PI/2048)*scale));beam.setValue(beamFault==1?false:beamFault==2?true:Math.abs(last.x()-.100)>length/2);RoboRioSim.setVInVoltage(last.bus());}
 public CoralContactPlant.Sample step(boolean enabled,int substeps){if(substeps!=200&&substeps!=400)throw new IllegalArgumentException("Refinement domain");controller.periodic(enabled);for(int i=0;i<substeps;i++)last=plant.step(pwm.getSpeed(),.02/substeps);inject();return last;}
 @Override public void close(){controller.close();}
}

