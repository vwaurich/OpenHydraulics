within OpenHydraulics.DevelopmentTests;
model TeleCylinderTest

  extends OpenHydraulics.Interfaces.PartialFluidCircuit(redeclare
      OpenHydraulics.Fluids.GenericOilSimple oil);

  Modelica.Mechanics.Translational.Components.Fixed fixed
    annotation (Placement(transformation(extent={{-22,50},{-2,70}})));
  Components.Cylinders.TeleCylinder2Stage teleCylinder1_1(
    initType=Types.RevoluteInit.Position,
    pistonMass=0.1,
    s_init1=0.5,
    s_init2=0,
    q_nom=1e-4) annotation (Placement(transformation(extent={{24,40},{64,80}})));
  OpenHydraulics.Basic.FluidPower2MechRotConst pump(Dconst=5e-4)
                                                         annotation (Placement(transformation(extent={{-24,-46},
            {-4,-26}})));
  OpenHydraulics.Components.Volumes.CircuitTank circuitTank
    annotation (Placement(transformation(extent={{30,-90},{10,-70}})));
  Modelica.Blocks.Sources.Ramp ramp(height=100, duration=50)
    annotation (Placement(transformation(extent={{-74,-46},{-54,-26}})));
  OpenHydraulics.Components.Valves.ReliefValve reliefValve
    annotation (Placement(transformation(
        origin={10,-46},
        extent={{-10,-10},{10,10}},
        rotation=270)));
  Modelica.Mechanics.Rotational.Sources.Position position(
                                                  f_crit=1, useSupport=false)
    annotation (Placement(transformation(extent={{-46,-46},{-26,-26}})));
  Components.Lines.NJunction j1(n_ports=3)  annotation (Placement(
        transformation(extent={{0,-26},{20,-6}})));
  Components.Lines.NJunction j2(n_ports=3)
    annotation (Placement(transformation(extent={{30,-66},{50,-46}})));
  Modelica.Mechanics.Translational.Components.Mass mass(m=100)
    annotation (Placement(transformation(extent={{116,50},{136,70}})));
  Modelica.Mechanics.Translational.Sources.Force force annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={168,50})));
  Modelica.Blocks.Sources.RealExpression realExpression(y=-600*time)
    annotation (Placement(transformation(extent={{162,8},{182,28}})));
  Components.Sensors.PressureSensor pressureSensor(pHigh=10000000)
    annotation (Placement(transformation(extent={{-32,12},{-12,32}})));
initial equation
  //reliefValve.port_a.p = 101325;

equation
  connect(fixed.flange, teleCylinder1_1.flange_a)
    annotation (Line(points={{-12,60},{24,60}}, color={0,127,0}));
  connect(pump.port_a,circuitTank. port_b) annotation (Line(points={{-14,-46},
          {-14,-80},{10,-80}}, color={255,0,0}));
  connect(ramp.y, position.phi_ref)
    annotation (Line(points={{-53,-36},{-48,-36}}, color={0,0,127}));
  connect(position.flange,   pump.flange_a)
    annotation (Line(points={{-26,-36},{-24,-36}}, color={0,0,0}));
  connect(pump.port_b, j1.port[1]) annotation (Line(points={{-14,-26},{-14,-16},
          {10,-16},{10,-16.3333}},      color={255,0,0}));
  connect(reliefValve.port_a, j1.port[2]) annotation (Line(points={{10,-36},{
          10,-26.025},{10,-16},{10,-16}}, color={255,0,0}));
  connect(circuitTank.port_a, j2.port[1]) annotation (Line(points={{30,-80},{40,
          -80},{40,-56.3333}},    color={255,0,0}));
  connect(reliefValve.port_b, j2.port[2])
    annotation (Line(points={{10,-56},{25,-56},{25,-56},{40,-56}}, color={255,
          0,0}));
  connect(teleCylinder1_1.port_a, j1.port[3])
    annotation (Line(points={{28,44},{10,44},{10,-15.6667}}, color={255,0,0}));
  connect(teleCylinder1_1.flange_b, mass.flange_a)
    annotation (Line(points={{64,60},{116,60}}, color={0,127,0}));
  connect(force.flange, mass.flange_b) annotation (Line(points={{158,50},{142,
          50},{142,60},{136,60}}, color={0,127,0}));
  connect(realExpression.y, force.f) annotation (Line(points={{183,18},{198,18},
          {198,50},{180,50}}, color={0,0,127}));
  connect(teleCylinder1_1.port_a, pressureSensor.port_a) annotation (Line(
        points={{28,44},{10,44},{10,12},{-22,12}}, color={255,0,0}));
annotation (
    experiment(
      StopTime=100,
      Tolerance=1e-08,
      __Dymola_Algorithm="Cvode"));
end TeleCylinderTest;
