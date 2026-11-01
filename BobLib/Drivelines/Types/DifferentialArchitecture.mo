within BobLib.Drivelines.Types;
type DifferentialArchitecture = enumeration(

  Open "Equal output torques, independent output speeds",
  ClutchLSD "Preloaded plate differential with drive/coast locking",
  TorqueBiasing "Helical-style differential with a bounded torque ratio",
  Locked "Spool with equal output speeds");
