within BobLibTest.Experiments.Standards.Examples;

model TestDirectSuspensionFourPost

  extends BobLib.Experiments.Standards.Examples.DirectSuspensionFourPost;

equation
  when time >= 3 then
    assert(abs(frChassisActuator.jackingOutput + 3190.563348682308) < 0.02,
      "Direct front suspension braking jacking changed at the prescribed reference pose");
    assert(abs(rrChassisActuator.jackingOutput + 2773.845760161949) < 0.02,
      "Direct rear suspension braking jacking changed at the prescribed reference pose");
  end when;
  annotation(Documentation(info = "<html><p>Coverage fixture for <code>BobLib.Experiments.Standards.Examples.DirectSuspensionFourPost</code>.</p></html>"));
end TestDirectSuspensionFourPost;
