within OpenHydraulics.DevelopmentTests;
model Prop43ValveTestSimple

  extends OpenHydraulics.Interfaces.PartialFluidCircuit(redeclare
      OpenHydraulics.Fluids.GenericOilSimple oil);

  OpenHydraulics.Components.Valves.ReliefValve reliefValve
    annotation (Placement(transformation(
        origin={-30,-16},
        extent={{-10,-10},{10,10}},
        rotation=270)));
  OpenHydraulics.Basic.OpenTank tank          annotation (Placement(transformation(extent={{-70,-60},
            {-50,-40}})));
  OpenHydraulics.Components.Cylinders.DoubleActingCylinder doubleActingCylinder(
    boreDiameter=0.12,
    strokeLength=1,
    closedLength=1,
    rodDiameter=0.05,
    pistonMass=0.3,
    s_init=0.5,
    initType=Types.RevoluteInit.PositionVelocityAcceleration) annotation (Placement(transformation(extent={{22,76},
            {42,96}})));

  Modelica.Mechanics.Translational.Components.Fixed fixed
    annotation (Placement(transformation(extent={{-10,76},{10,96}})));
  Modelica.Mechanics.Translational.Components.Mass slidingMass(m=1)
    annotation (Placement(transformation(extent={{60,76},{80,96}})));
  Modelica.Blocks.Sources.Sine sine(
    amplitude=1,
    f=0.2,
    offset=0,
    startTime=1)
    annotation (Placement(transformation(extent={{-34,-80},{-14,-60}})));
  Components.Valves.DirectionalValves.V4_3CC
                                           throttleValve(q_nom=0.001, dp_nom=
        10000000)
    annotation (Placement(transformation(extent={{20,-10},{40,10}})));
  OpenHydraulics.Basic.ConstVolumeSource source(q=0.001)
    annotation (Placement(transformation(extent={{-70,-20},{-50,0}})));
  Components.Lines.NJunction j1(            n_ports=3)
    annotation (Placement(transformation(extent={{-40,0},{-20,20}})));
  Components.Lines.NJunction j2(            n_ports=3)
    annotation (Placement(transformation(extent={{-40,-50},{-20,-30}})));
  Components.Sensors.PressureSensor pressureSensor
    annotation (Placement(transformation(extent={{-10,30},{10,50}})));
  Components.Sensors.PressureSensor pressureSensor1
    annotation (Placement(transformation(extent={{50,30},{70,50}})));
  Components.Sensors.PressureSensor pressureSensor2
    annotation (Placement(transformation(extent={{-10,0},{10,20}})));
  Components.Sensors.PressureSensor pressureSensor3
    annotation (Placement(transformation(extent={{50,-8},{70,12}})));
  Components.Lines.Line line(L=2, D=0.05) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={24,56})));
  Components.Lines.Line line1(L=2, D=0.05) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={40,56})));
equation
  connect(doubleActingCylinder.flange_a,fixed.flange)    annotation (Line(
        points={{22,86},{0,86}},        color={0,127,0}));
  connect(doubleActingCylinder.flange_b, slidingMass.flange_a)
    annotation (Line(points={{42,86},{60,86}}, color={0,127,0}));
  connect(throttleValve.control,sine. y) annotation (Line(points={{41,0},{52,0},
          {52,-70},{-13,-70}},
                           color={0,0,127}));
  connect(reliefValve.port_a, j1.port[1]) annotation (Line(points={{-30,-6},{
          -30,1.625},{-30,9.66667},{-30,9.66667}}, color={255,0,0}));
  connect(source.port, j1.port[2]) annotation (Line(points={{-60,0},{-60,10},
          {-30,10}}, color={255,0,0}));
  connect(tank.port, j2.port[1]) annotation (Line(points={{-60,-40},{-30,-40},{
          -30,-40.3333}},  color={255,0,0}));
  connect(reliefValve.port_b, j2.port[2]) annotation (Line(points={{-30,-26},
          {-30,-33.025},{-30,-40},{-30,-40}}, color={255,0,0}));
  connect(throttleValve.portP, j1.port[3]) annotation (Line(points={{26,-8},{
          -12,-8},{-12,10.3333},{-30,10.3333}}, color={255,0,0}));
  connect(throttleValve.portT, j2.port[3]) annotation (Line(points={{34,-8},{38,
          -8},{38,-39.6667},{-30,-39.6667}}, color={255,0,0}));
  connect(throttleValve.portP, pressureSensor2.port_a)
    annotation (Line(points={{26,-8},{0,-8},{0,0}}, color={255,0,0}));
  connect(throttleValve.portT, pressureSensor3.port_a) annotation (Line(points=
          {{34,-8},{38,-8},{38,-14},{60,-14},{60,-8}}, color={255,0,0}));
  connect(line.port_b, doubleActingCylinder.port_a)
    annotation (Line(points={{24,66},{24,78}}, color={255,0,0}));
  connect(line1.port_b, doubleActingCylinder.port_b)
    annotation (Line(points={{40,66},{40,78}}, color={255,0,0}));
  connect(throttleValve.portA, line.port_a)
    annotation (Line(points={{26,8},{24,8},{24,46}}, color={255,0,0}));
  connect(line1.port_a, throttleValve.portB)
    annotation (Line(points={{40,46},{40,8},{34,8}}, color={255,0,0}));
  connect(pressureSensor1.port_a, throttleValve.portB)
    annotation (Line(points={{60,30},{40,30},{40,8},{34,8}}, color={255,0,0}));
  connect(pressureSensor.port_a, line.port_a)
    annotation (Line(points={{0,30},{24,30},{24,46}}, color={255,0,0}));
  annotation (
    experiment(StopTime=10, __Dymola_Algorithm="Dassl"));
end Prop43ValveTestSimple;
