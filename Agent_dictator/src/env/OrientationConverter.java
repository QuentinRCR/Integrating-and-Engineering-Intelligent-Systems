import cartago.Artifact;
import cartago.OPERATION;

/**
 *      Artifact that implements the auction.
 */
public class OrientationConverter extends Artifact {
    @OPERATION
    public void init()  {
        // observable properties
        defineObsProperty("converted_value",0);
    }

    @OPERATION
    public void convert(double z, double w) {
        getObsProperty("converted_value").updateValue(Math.atan2(2*w*z, 1-2*z*z));
    }
}
