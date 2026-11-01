package org.firstinspires.ftc.teamcode.shooter.core;

/**
 * Where to shoot: the alliance's upward-facing CELL (manual §9.6). The HIVE tips after enough balls land, and
 * then the OTHER cell faces up, so the aim point jumps by roughly 2 × cellOffset. The code tracks which cell
 * is up: drivers or vision call {@link #notifyTip()}, or set it explicitly.
 *
 * <p>Field frame (our convention, inches): origin at the field centre (HIVE pivot), +y toward the side
 * opposite the audience, +x toward the audience's right. All values are ❓ until checked against the official
 * field CAD and measured on a real field.
 */
public final class HiveTarget {

    public enum Alliance { RED, BLUE }
    public enum UpCell { AUDIENCE_SIDE, FAR_SIDE }

    public static final class Config {
        /** HIVE centre lines are 25.5 in apart (manual Fig. 9-10). Red assumed at −x ❓. */
        public double redHiveX = -12.75;
        public double blueHiveX = 12.75;
        /** Horizontal offset of the upward cell's opening centre from the pivot along y (in) ❓ (~15.8 from Fig. 9-10). */
        public double cellOffsetY = 15.8;
        /** Opening centre height: (53.5 + 65.6) / 2 (manual Fig. 9-10). */
        public double openingCenterZ = 59.5;
        /** Half the opening width (20 in wide): used for the lateral aim tolerance. */
        public double openingHalfWidth = 10.0;
        /** Fine-tune offsets from on-field testing (in). */
        public double trimX = 0, trimY = 0, trimZ = 0;
    }

    private final Config cfg;
    private Alliance alliance;
    private UpCell upCell;

    public HiveTarget(Config cfg, Alliance alliance, UpCell initial) {
        this.cfg = cfg;
        this.alliance = alliance;
        this.upCell = initial;
    }

    public void setAlliance(Alliance a) { alliance = a; }
    public void setUpCell(UpCell c) { upCell = c; }
    public UpCell upCell() { return upCell; }

    /** The HIVE tipped: the other cell now faces up. */
    public void notifyTip() {
        upCell = upCell == UpCell.AUDIENCE_SIDE ? UpCell.FAR_SIDE : UpCell.AUDIENCE_SIDE;
    }

    public double x() { return (alliance == Alliance.RED ? cfg.redHiveX : cfg.blueHiveX) + cfg.trimX; }
    public double y() { return (upCell == UpCell.FAR_SIDE ? cfg.cellOffsetY : -cfg.cellOffsetY) + cfg.trimY; }
    public double z() { return cfg.openingCenterZ + cfg.trimZ; }
    public double halfWidth() { return cfg.openingHalfWidth; }
    public Config config() { return cfg; }
}
