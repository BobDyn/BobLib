within BobLibVehicleInterfaces.Experiments;
package Standards

  annotation(
    Documentation(info = "<html>
<p>
Package <code>BobLibVehicleInterfaces.Experiments.Standards</code> contains standard runnable experiments.
</p>
<p>
The models here are the primary entry points for full-vehicle and four-post simulations assembled from one-level public subsystem models.
</p>
<p>
Architecture-specific subsystem redeclare stacks live in <code>Architectures</code>.
The public benchmark models extend those architecture models so new powertrain
layouts can be added without duplicating the runnable experiment entrypoints.
</p>
</html>"));
end Standards;
