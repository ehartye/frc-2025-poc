package org.reefscape.controls;

/** Wrap the frozen station program on its existing 20 ms desktop service. */
public final class GuardedAcquisition {
    private final PortAcquisition ports = new PortAcquisition(true);
    private final IdlerWatch idlers = new IdlerWatch();
    private String veto = "";
    public record Output(String state, double beltMps, String reason, boolean positiveDriveSeen) {}
    public void observe(boolean left, IdlerWatch.Reading reading) { idlers.sample(left,reading); }

    public Output update(long ms,long sampleMs,boolean validPorts,boolean portCalibration,boolean docked,boolean enabled,boolean entry,boolean seat,boolean calibrated) {
        if (!veto.isEmpty()) return new Output("FAULT",0,veto,false);
        var p=ports.update(ms,sampleMs,validPorts,portCalibration,docked,enabled,entry,seat);
        if (!p.active()) return new Output(p.state().name(),0,p.reason(),false);
        boolean probing=p.state()==PortAcquisition.State.PROBE;
        boolean settling=p.state()==PortAcquisition.State.DWELL;
        // Once the port controller settles, encoder motion is no longer demanded.
        // On the transition, it must already have been observed on both idlers.
        var g=idlers.evaluate(ms*1000,calibrated,probing,settling);
        if (!g.allow()) { veto=g.reason(); return new Output("FAULT",0,veto,false); }
        return new Output(p.state().name(),p.beltMps(),p.reason(),g.motionSeen());
    }
}

