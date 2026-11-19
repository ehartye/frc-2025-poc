package org.reefscape.controls;

/** Exercise packet-level failure modes that do not depend on a station pipe trajectory. */
public final class WatchBoundaryTest {
    private static void require(boolean condition, String label) {
        if (!condition) throw new AssertionError(label);
    }
    private static void pair(IdlerWatch watch, long at, int left, int right) {
        watch.sample(true, new IdlerWatch.Reading(at, at, left, true));
        watch.sample(false, new IdlerWatch.Reading(at, at, right, true));
    }
    public static void main(String[] arguments) {
        IdlerWatch noSamples = new IdlerWatch();
        require(!noSamples.evaluate(0, true, true, false).allow(), "no converter frames");

        IdlerWatch noPriorCommand = new IdlerWatch();
        for (long at=0; at<=20_000; at+=5_000) pair(noPriorCommand, at, 47, 47);
        require(!noPriorCommand.evaluate(20_000, true, false, true).allow(), "unproven dwell");

        IdlerWatch earlyDwell = new IdlerWatch();
        for (long at=0; at<=20_000; at+=5_000) pair(earlyDwell, at, 47, 47);
        require(earlyDwell.evaluate(20_000, true, true, false).allow(), "bounded startup");
        for (long at=25_000; at<=40_000; at+=5_000) pair(earlyDwell, at, 47, 47);
        require(!earlyDwell.evaluate(40_000, true, false, true).allow(), "early dual without travel");
        for (long at=45_000; at<=100_000; at+=5_000) pair(earlyDwell, at, 47, 47);
        require(!earlyDwell.evaluate(100_000, true, true, false).allow(), "latched fault");

        IdlerWatch discontinuous = new IdlerWatch();
        for (long at=0; at<=20_000; at+=5_000) pair(discontinuous, at, 400, 400);
        pair(discontinuous, 25_000, 1225, 1225);
        require(!discontinuous.evaluate(25_000, true, true, false).allow(), "half-turn ambiguity");

        IdlerWatch futureDelivery = new IdlerWatch();
        for (long at=0; at<=20_000; at+=5_000) pair(futureDelivery, at, 400, 400);
        futureDelivery.sample(true, new IdlerWatch.Reading(25_000, 29_000, 401, true));
        require(!futureDelivery.evaluate(25_000, true, true, false).allow(), "future arrival");
    }
}
