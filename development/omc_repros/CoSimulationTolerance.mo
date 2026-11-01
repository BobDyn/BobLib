model CoSimulationTolerance "The requested FMI tolerance must reach the CS integrator"
  Real x(start=1, fixed=true);
  output Real solution=x;
equation
  der(x)=-x;
end CoSimulationTolerance;
