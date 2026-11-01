within BobLibTest.Experiments.Standards;

model TestKnCSteerable

  extends BobLib.Experiments.Standards.KnCSteerable;

equation
  assert(abs(axle.leftCP.r_0[3] - cpLeft[3]) < 1e-7 and
    abs(axle.rightCP.r_0[3] - cpLeft[3]) < 1e-7,
    "Steering sweep contact patches must remain on the fixture plane");
  for i in 1:7 loop

    when i <= angleCount and time >= 119*(i - 1) + 3 then
      assert(abs(axle.rackAndPinion.rackAxis.s -
        steeringAngles[i]*axle.pAxle.steering.cFactor/(2*Modelica.Constants.pi)) < 1e-7,
        "Steering sweep did not reach the scheduled rack displacement");
    end when;
  end for;
end TestKnCSteerable;
