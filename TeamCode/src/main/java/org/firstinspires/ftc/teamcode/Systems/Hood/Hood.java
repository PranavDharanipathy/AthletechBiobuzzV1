package org.firstinspires.ftc.teamcode.Systems.Hood;

import org.firstinspires.ftc.teamcode.Constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.Utility.InterpolationData;
import org.firstinspires.ftc.teamcode.Utility.MathUtil;

import java.util.List;

public class Hood {

    private HoodAngler hoodAngler;

    public HoodAngler accessHoodAngler() {
        return hoodAngler;
    }

    public Hood(HoodAngler hoodAngler) {
        this.hoodAngler = hoodAngler;
    }

    public void update(double distanceToGoal) {

        double hoodPosition = getHoodPositionFromInterpolation(
                distanceToGoal,
                ShooterConstants.HOOD_DISTANCES,
                ShooterConstants.HOOD_POSITIONS
        );

        hoodAngler.setSafePosition(hoodPosition);

    }

    public void tuningUpdate(double hoodPosition) {

        hoodAngler.setSafePosition(hoodPosition);

    }

    private double getHoodPositionFromInterpolation(double distanceToGoal, List<Double> distancesList, List<Double> positionsList) {

        if (distanceToGoal < distancesList.get(0)) {
            return positionsList.get(0);
        }
        else if (distanceToGoal > distancesList.get(distancesList.size() - 1)) {
            return positionsList.get(positionsList.size() - 1);
        }

        double[] distances = distancesList.stream().mapToDouble(Double::doubleValue).toArray();

        double[] bounds = MathUtil.findBoundingValues(distances, distanceToGoal);

        double distance0 = bounds[0];
        double distance1 = bounds[1];

        double hoodPosition0 = positionsList.get(distancesList.indexOf(distance0));
        double hoodPosition1 = positionsList.get(distancesList.indexOf(distance1));

        return MathUtil.interpolateLinear(

                distanceToGoal,

                new InterpolationData(
                        new double[] {distance0, hoodPosition0},
                        new double[] {distance1, hoodPosition1}
                )
        );

    }
}
