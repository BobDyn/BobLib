within BobLib.Brakes;
model BasicVCUBrakes "VCU-commanded friction brakes with explicit optional spatial supports"

  extends BobLib.Brakes.Internal.BrakeActuation(final usePedal = false);
end BasicVCUBrakes;