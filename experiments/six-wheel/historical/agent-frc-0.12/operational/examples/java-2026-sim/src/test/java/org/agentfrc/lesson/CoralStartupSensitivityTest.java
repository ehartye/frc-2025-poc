package org.agentfrc.lesson;
import static org.junit.jupiter.api.Assertions.*;
import org.junit.jupiter.api.*;
class CoralStartupSensitivityTest {
 record Result(String outcome,double elapsed,double x,double speed,double residual){}
 static Result run(double mass,double mu,int pitch,double spin,double volts,double dt){
  var p=new PoweredGuidePlant(new PoweredGuidePlant.Config(mass,.0254,.004,.15,Math.toRadians(pitch),15,mu,.08,volts));
  for(int k=0;k<Math.round(spin/dt);k++)p.step(dt);p.feed(.2);var r=p.sample();
  for(int k=0;k<Math.round(3/dt);k++){r=p.step(dt);assertEquals(0,r.energyResidual(),2e-5);if(r.exited())return new Result("EXIT",r.time()-spin,r.x(),r.speed(),r.energyResidual());if(r.x()<0&&r.speed()<0)return new Result("ROLLBACK",r.time()-spin,r.x(),r.speed(),r.energyResidual());}
  return new Result("TIMEOUT",r.time()-spin,r.x(),r.speed(),r.energyResidual());
 }
 @Test void sourceMassBoundaryGrid(){int count=0,exits=0,rollbacks=0;for(double mass:new double[]{1.1*.45359237,1.8*.45359237})for(double mu:new double[]{.05,.2,.8})for(int pitch:new int[]{0,15,30})for(double spin:new double[]{0,.5,2})for(double volts:new double[]{9,12}){
  var a=run(mass,mu,pitch,spin,volts,.0001);var b=run(mass,mu,pitch,spin,volts,.00005);assertEquals(a.outcome(),b.outcome());assertEquals(a.elapsed(),b.elapsed(),.0002);assertEquals(a.speed(),b.speed(),.002);if(b.outcome().equals("EXIT"))exits++;if(b.outcome().equals("ROLLBACK"))rollbacks++;count++;
  System.out.println("CORAL_SENSITIVITY massKg="+mass+" mu="+mu+" pitchDegrees="+pitch+" spinupS="+spin+" volts="+volts+" "+b);
 }assertEquals(108,count);assertTrue(exits>0);assertTrue(rollbacks>0);}
 @Test void boundsAndFrictionAreSeparate(){assertEquals(.498951607,1.1*.45359237,1e-12);assertEquals(.816466266,1.8*.45359237,1e-12);for(double mass:new double[]{1.1*.45359237,1.8*.45359237}){assertTrue(2*.05*15<mass*9.80665*Math.sin(Math.PI/6));assertTrue(2*.8*15>mass*9.80665*Math.sin(Math.PI/6));}}
 @AfterAll static void binding() throws Exception {
  String token=System.getenv("FRC_CORAL_RUN_ID"),output=System.getenv("FRC_CORAL_RUNTIME");if(token==null&&output==null)return;assertNotNull(token);assertNotNull(output);var p=new java.util.Properties();p.setProperty("runId",token);
  for(Class<?> c:new Class<?>[]{PoweredGuidePlant.class,PoweredGuidePlant.Config.class,PoweredGuidePlant.Sample.class,SharedSupply.class,SharedSupply.Result.class,CoralStartupSensitivityTest.class,Result.class,edu.wpi.first.math.system.plant.DCMotor.class}){String n=c.getName().substring(c.getName().lastIndexOf('.')+1);p.setProperty(n+"Sha256",java.util.HexFormat.of().formatHex(java.security.MessageDigest.getInstance("SHA-256").digest(c.getResourceAsStream(n+".class").readAllBytes())));}
  try(var stream=java.nio.file.Files.newOutputStream(java.nio.file.Path.of(output),java.nio.file.StandardOpenOption.CREATE_NEW)){p.store(stream,"Fresh CORAL mass sensitivity");}System.out.println("CORAL_RUN_ID="+token);
 }
}
