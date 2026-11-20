import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Arrays;

/** Read back the existing spatial solver's instantaneous wrench, excluding gravity. */
public class ForceWitness {
    public static void main(String[] args) throws Exception {
        SpatialContact.main(Arrays.copyOfRange(args, 0, 5));
        for (String record : Files.readAllLines(Path.of(args[5]))) {
            String[] values = record.split(",");
            SpatialContact tube = new SpatialContact(values);
            tube.contacts(Integer.parseInt(values[19]));
            double[] wrench = {tube.fx, tube.fy, tube.fz + SpatialContact.MASS * SpatialContact.G,
                    tube.tx, tube.ty, tube.tz, Math.max(0, tube.floorHi - tube.floorLo),
                    Math.max(0, tube.lipHi - tube.lipLo), tube.compression};
            for (double value : wrench) System.out.print(value + " ");
            System.out.println();
        }
    }
}
