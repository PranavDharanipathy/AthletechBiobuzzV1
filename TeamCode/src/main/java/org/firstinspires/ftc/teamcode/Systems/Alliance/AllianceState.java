package org.firstinspires.ftc.teamcode.Systems.Alliance;

public class AllianceState {

    private Alliance alliance;

    public AllianceState(Alliance initialAlliance) {
        this.alliance = initialAlliance;
    }

    public void setAlliance(Alliance alliance) {
        this.alliance = alliance;
    }

    public Alliance getAlliance() {
        return alliance;
    }

    public boolean isRedAlliance() {
        return alliance.isRedAlliance();
    }

    public boolean isBlueAlliance() {
        return alliance.isBlueAlliance();
    }
}
