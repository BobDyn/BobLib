within BobLibVehicleInterfaces.Chassis;
partial model Chassis_LockRrSteer

  "Detailed BobLib chassis with rear steer locked"
  extends BobLibVehicleInterfaces.Chassis.ChassisBase(
    redeclare BobLibVehicleInterfaces.Chassis.Internal.DetailedChassis_LockRrSteer detailedChassis);
  annotation(
    Documentation(info = "<html>
<p>
Partial model <code>Chassis_LockRrSteer</code> binds the public chassis adapter to a detailed chassis with rear steering locked.
</p>
<p>
It is a reusable specialization for vehicle experiments that want front steering input while preserving the VehicleInterfaces chassis contract.
</p>
</html>"));
end Chassis_LockRrSteer;
