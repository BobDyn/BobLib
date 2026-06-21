within BobLibVehicleInterfaces.Aero;
package Internal

  "Internal aerodynamic helper functions"
  extends BobLibVehicleInterfaces.Icons.BobLibInternalPackageBackground;

  annotation(
    Documentation(info = "<html>
<p>
Package <code>BobLibVehicleInterfaces.Aero.Internal</code> contains helper calculations used by aerodynamic models.
</p>
<p>
These helpers stay below the public aero boundary so lookup and interpolation details do not appear as subsystem insertion points.
</p>
</html>"));
end Internal;
