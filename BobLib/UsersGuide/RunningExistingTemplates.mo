within BobLib.UsersGuide;
class RunningExistingTemplates "Run and configure the standard experiments"
  extends Modelica.Icons.Information;
  annotation(preferredView = "info", Documentation(info = "<html>
<h4>Choose an entry point</h4>
<ul>
<li><a href='modelica://BobLib.Experiments.Standards.VehicleSim'>VehicleSim</a>: calibrated maneuver simulation with StandardDriver and QSS initialization.</li>
<li><a href='modelica://BobLib.Experiments.Standards.VehicleFMI'>VehicleFMI</a>: steering-angle, accelerator-pedal, and brake-pedal inputs. Inverter enable is internal; automatic speed control is disabled by default.</li>
<li><a href='modelica://BobLib.Experiments.Standards.FourPostSim'>FourPostSim</a>: suspension kinematics and compliance on the four-post rig. Its standard configuration sets stabilizer-bar rates to zero; enable them explicitly when required.</li>
</ul>
<h4>Configure the architecture</h4>
<p>Use the chassis Front axle and Rear axle model selections independently. The double-wishbone implementation supports DoubleWishboneDirect, DoubleWishboneBellcrank, and DoubleWishboneBellcrankStabar at either end. Supply a compatible VehicleRecord and geometry for the selected models.</p>
<p><a href='modelica://BobLib.Experiments.Standards.Examples.MixedSuspensionVehicle'>MixedSuspensionVehicle</a> shows a bellcrank front axle and direct rear axle. <a href='modelica://BobLib.Experiments.Standards.Examples.DirectSuspensionFourPost'>DirectSuspensionFourPost</a> shows direct actuation on the rig. These examples demonstrate configuration; the full front/rear combination matrix is tested in BobLibTest.Experiments.Standards.Configurations.</p>
<p>VehicleSim uses a solved QSS pose from its vehicle record. A changed geometry, loading, or initial speed requires a matching initialization solution; selecting a different axle alone does not recalibrate that pose. The mixed vehicle example extends the maneuver base without imposing the default vehicle's solved suspension angles.</p>
<p><a href='modelica://BobLib.Experiments.Standards.Examples.RoadAndAtmosphereVehicle'>RoadAndAtmosphereVehicle</a> replaces VI road friction and atmospheric wind. <a href='modelica://BobLib.Experiments.Standards.Examples.SpatialBrakingVehicle'>SpatialBrakingVehicle</a> connects explicit driver-command sources to the FMI plant and enables spatial mount reactions and selectable brake locations. <a href='modelica://BobLib.Experiments.Standards.Examples.PhysicalPedalVehicle'>PhysicalPedalVehicle</a> demonstrates the physical VI pedal path.</p>
<h4>Shared implementations</h4>
<p><a href='modelica://BobLib.Experiments.Standards.BaseClasses.PartialElectricVehicle'>PartialElectricVehicle</a> owns the shared plant, environment, bus wiring, and measurements. PartialVehicleSim adds maneuver control and termination monitors; PartialVehicleFMI adds external driver inputs. Components remain visible in the inherited diagram.</p>
<p><a href='modelica://BobLib.Experiments.Standards.BaseClasses.PartialFourPostRig'>PartialFourPostRig</a> owns architecture-neutral fixtures and actuation. PartialDoubleWishboneFourPost supplies wishbone records, tire configuration, contact positions, and kinematics measurements. Additional axle families can specialize the neutral rig; MacPherson physics is not yet supplied.</p>
<h4>Parameters and results</h4>
<p>Complete calibrations remain under Records.VehicleDefn. VehicleSim exposes maneuver outputs and monitors; VehicleFMI exposes driver inputs and vehicle measurements; FourPostSim exposes frKnC and rrKnC. FourPostSim supports outboard, halfshaft-inboard, and differential-input brake reactions. Its prescribed-wheel loading uses ideal static differential sharing, not a dynamic LSD prediction.</p>
</html>"));
end RunningExistingTemplates;
