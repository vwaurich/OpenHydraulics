within OpenHydraulics.DevelopmentTests;
model Prop43Valve_TeleCylinder

  extends OpenHydraulics.Interfaces.PartialFluidCircuit(redeclare
      OpenHydraulics.Fluids.GenericOilSimple oil);

  OpenHydraulics.Components.Valves.ReliefValve reliefValve
    annotation (Placement(transformation(
        origin={-30,-16},
        extent={{-10,-10},{10,10}},
        rotation=270)));
  OpenHydraulics.Basic.OpenTank tank          annotation (Placement(transformation(extent={{-70,-60},
            {-50,-40}})));

  Modelica.Mechanics.Translational.Components.Fixed fixed
    annotation (Placement(transformation(extent={{-10,76},{10,96}})));
  Modelica.Mechanics.Translational.Components.Mass slidingMass(m=100)
    annotation (Placement(transformation(extent={{68,76},{88,96}})));
  Modelica.Blocks.Sources.Ramp ramp(
    height=0.3,
    duration=1,
    offset=0,
    startTime=0)
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
  Modelica.Mechanics.Translational.Sources.Force force annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={128,80})));
  Modelica.Blocks.Sources.RealExpression realExpression(y=-100*time)
    annotation (Placement(transformation(extent={{122,38},{142,58}})));
  Components.Cylinders.TeleCylinder2Stage3 teleCylinder1_1(
    damping=0,
    stopStiffness=1e6,
    stopDamping=1e6,
    useCushionRod=false,
    initType=Types.RevoluteInit.Position,
    pistonMass=0.1,
    s_init1=0,
    s_init2=0,
    q_nom=1e-4) annotation (Placement(transformation(extent={{22,72},{50,100}})));
equation
  connect(throttleValve.control,ramp. y) annotation (Line(points={{42.8,0},{52,
          0},{52,-70},{-13,-70}},
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
  connect(throttleValve.portA, line.port_a)
    annotation (Line(points={{26,8},{24,8},{24,46}}, color={255,0,0}));
  connect(line1.port_a, throttleValve.portB)
    annotation (Line(points={{40,46},{40,8},{34,8}}, color={255,0,0}));
  connect(pressureSensor1.port_a, throttleValve.portB)
    annotation (Line(points={{60,30},{40,30},{40,8},{34,8}}, color={255,0,0}));
  connect(pressureSensor.port_a, line.port_a)
    annotation (Line(points={{0,30},{24,30},{24,46}}, color={255,0,0}));
  connect(realExpression.y,force. f) annotation (Line(points={{143,48},{158,48},
          {158,80},{140,80}}, color={0,0,127}));
  connect(force.flange, slidingMass.flange_b) annotation (Line(points={{118,80},
          {86,80},{86,86},{88,86}}, color={0,127,0}));
  connect(fixed.flange, teleCylinder1_1.flange_a)
    annotation (Line(points={{0,86},{22,86}}, color={0,127,0}));
  connect(slidingMass.flange_a, teleCylinder1_1.flange_b)
    annotation (Line(points={{68,86},{50,86}}, color={0,127,0}));
  connect(line.port_b, teleCylinder1_1.port_a)
    annotation (Line(points={{24,66},{24.8,66},{24.8,74.8}}, color={255,0,0}));
  annotation (
    experiment(StopTime=20, __Dymola_Algorithm="Dassl"));
end Prop43Valve_TeleCylinder;
