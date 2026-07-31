from __future__ import annotations

from pathlib import Path


def test_wheel_alignment_uses_degree_type_and_explicit_radian_conversion() -> None:
    root = Path(__file__).resolve().parents[1]
    record = (
        root
        / "BobLib"
        / "Records"
        / "VehicleRecord"
        / "Chassis"
        / "Suspension"
        / "Templates"
        / "Tire"
        / "Templates"
        / "PartialWheelRecord.mo"
    ).read_text(encoding="utf-8")

    assert "parameter NonSI.Angle_deg staticAlpha" in record
    assert "parameter NonSI.Angle_deg staticGamma" in record
    assert "parameter SI.Angle staticAlpha" not in record
    assert "parameter SI.Angle staticGamma" not in record

    consumers = (
        root / "BobLib" / "Chassis" / "Chassis_DW.mo",
        root / "BobLib" / "Chassis" / "Chassis_DWBCStabar_DWBCStabar.mo",
        root
        / "BobLib"
        / "Experiments"
        / "Standards"
        / "Templates"
        / "FourPost"
        / "BaseFourPostSim.mo",
    )

    for path in consumers:
        source = path.read_text(encoding="utf-8")
        assert "staticAlpha*pi/180" not in source
        assert "staticGamma*pi/180" not in source
        assert "Conversions.from_deg" in source
