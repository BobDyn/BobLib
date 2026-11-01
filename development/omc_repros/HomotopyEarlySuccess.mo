model HomotopyEarlySuccess "A solved initial guess must survive the next nonlinear solve"
  model Surface
    outer VehicleInterfaces.Roads.Interfaces.Base road;
    input Real r[3];
    parameter Real s_start=2, w_start=3;
    Real s(start=s_start), w(start=w_start);
    output Real position[3];
    output Real gap;
    output Real normal[3];
    output Real heading[3];
  protected
    Real lateral[3], delta[3];
  equation
    position=road.position(s,w);
    normal=road.normal(s,w);
    heading=road.headingDirection(s,w);
    0=delta*heading;
    0=delta*lateral;
    lateral=cross(normal,heading);
    delta=r-position;
    gap=delta*normal;
    assert(abs(normal*normal-1)<1e-6,"Nonlinear solution lost the unit normal");
  end Surface;
  inner VehicleInterfaces.Roads.FlatRoad road(animation=false);
  Real x(start=2,fixed=true);
  Surface surface(r=road.position(x,3)-0.01*road.normal(x,3));
  output Real gap=surface.gap;
  output Real normalZ=surface.normal[3];
equation
  der(x)=1;
end HomotopyEarlySuccess;
