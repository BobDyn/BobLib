within BobLibTest.Experiments.Standards.Configurations;

model RigidFrameVehicle "Frame selection must not retain compliance-only modifiers"

  extends BobLib.Experiments.Standards.VehicleSim(
    chassis(redeclare model FrameModel = BobLib.Chassis.Body.FrameRigid));
end RigidFrameVehicle;
