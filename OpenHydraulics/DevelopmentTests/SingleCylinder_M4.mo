within OpenHydraulics.DevelopmentTests;
model SingleCylinder_M4

  extends OpenHydraulics.Interfaces.PartialFluidCircuit(redeclare
      OpenHydraulics.Fluids.GenericOilSimple oil);

  OpenHydraulics.Components.Valves.ReliefValve reliefValve
    annotation (Placement(transformation(
        origin={-30,-16},
        extent={{-10,-10},{10,10}},
        rotation=270)));
  OpenHydraulics.Basic.OpenTank tank          annotation (Placement(transformation(extent={{-80,-60},
            {-60,-40}})));

  Modelica.Mechanics.Translational.Components.Fixed fixed
    annotation (Placement(transformation(extent={{-36,104},{-16,124}})));
  Modelica.Mechanics.Translational.Components.Mass slidingMass(m=100)
    annotation (Placement(transformation(extent={{76,104},{96,124}})));
  Modelica.Blocks.Sources.Ramp ramp(
    height=1,
    duration=5,
    offset=1.1,
    startTime=1)
    annotation (Placement(transformation(extent={{120,-40},{140,-20}})));
  Components.Valves.DirectionalValves.V4_3CCLSHydAntiCavitation
                                           throttleValve(q_nom=0.001, dp_nom=
        10000000)
    annotation (Placement(transformation(extent={{14,6},{50,42}})));
  OpenHydraulics.Basic.ConstVolumeSource source(q=0.001)
    annotation (Placement(transformation(extent={{-80,-20},{-60,0}})));
  Components.Lines.NJunction j1(            n_ports=3)
    annotation (Placement(transformation(extent={{-40,0},{-20,20}})));
  Components.Lines.NJunction j2(            n_ports=3)
    annotation (Placement(transformation(extent={{-40,-50},{-20,-30}})));
  Components.Sensors.PressureSensor pressureSensor
    annotation (Placement(transformation(extent={{-10,76},{10,96}})));
  Components.Sensors.PressureSensor pressureSensor2
    annotation (Placement(transformation(extent={{-28,32},{-8,52}})));
  Components.Sensors.PressureSensor pressureSensor3
    annotation (Placement(transformation(extent={{-22,-32},{-2,-12}})));
  Components.Lines.Line line(L=2, D=0.05) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={24,86})));
  Modelica.Mechanics.Translational.Sources.Force force annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={120,114})));
  Modelica.Blocks.Sources.RealExpression realExpression(y=-300*time)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
        rotation=180,
        origin={150,114})));
  Components.Cylinders.TeleCylinder2Stage3 teleCylinder1_1(
    damping=1e2,
    useCushionRod=true,
    initType=Types.RevoluteInit.Position,
    pistonMass=0.1,
    s_init1=0,
    s_init2=0,
    q_nom=1e-4) annotation (Placement(transformation(extent={{20,100},{54,128}})));
  Modelica.Blocks.Continuous.LimPID
                                 PID(
    controllerType=Modelica.Blocks.Types.SimpleController.PID,
    k=5,
    Ti=0.5,
    Td=1,
    yMax=1,
    yMin=-1) annotation (Placement(transformation(extent={{10,-10},{-10,10}},
        rotation=0,
        origin={168,38})));
  Modelica.Blocks.Interfaces.RealInput u annotation (Placement(transformation(
        extent={{-20,-20},{20,20}},
        rotation=180,
        origin={372,-88})));
  Modelica.Mechanics.Translational.Sensors.PositionSensor positionSensor
    annotation (Placement(transformation(extent={{154,66},{174,86}})));
equation
  connect(reliefValve.port_a, j1.port[1]) annotation (Line(points={{-30,-6},{
          -30,1.625},{-30,9.66667},{-30,9.66667}}, color={255,0,0}));
  connect(source.port, j1.port[2]) annotation (Line(points={{-70,0},{-70,10},{
          -30,10}},  color={255,0,0}));
  connect(tank.port, j2.port[1]) annotation (Line(points={{-70,-40},{-70,-36},{
          -46,-36},{-46,-40.3333},{-30,-40.3333}},
                           color={255,0,0}));
  connect(reliefValve.port_b, j2.port[2]) annotation (Line(points={{-30,-26},
          {-30,-33.025},{-30,-40},{-30,-40}}, color={255,0,0}));
  connect(throttleValve.portP, j1.port[3]) annotation (Line(points={{24.8,9.6},
          {24.8,10},{-16,10},{-16,10.3333},{-30,10.3333}},
                                                color={255,0,0}));
  connect(throttleValve.portT, j2.port[3]) annotation (Line(points={{39.2,9.6},
          {39.2,-40},{-16,-40},{-16,-39.6667},{-30,-39.6667}},
                                             color={255,0,0}));
  connect(throttleValve.portT, pressureSensor3.port_a) annotation (Line(points={{39.2,
          9.6},{39.2,-40},{-12,-40},{-12,-32}},        color={255,0,0}));
  connect(throttleValve.portA, line.port_a)
    annotation (Line(points={{24.8,38.4},{24.8,72},{24,72},{24,76}},
                                                     color={255,0,0}));
  connect(pressureSensor.port_a, line.port_a)
    annotation (Line(points={{0,76},{0,72},{24,72},{24,76}},
                                                      color={255,0,0}));
  connect(realExpression.y,force. f) annotation (Line(points={{139,114},{132,
          114}},              color={0,0,127}));
  connect(force.flange, slidingMass.flange_b) annotation (Line(points={{110,114},
          {96,114}},                color={0,127,0}));
  connect(fixed.flange, teleCylinder1_1.flange_a)
    annotation (Line(points={{-26,114},{20,114}}, color={0,127,0}));
  connect(slidingMass.flange_a, teleCylinder1_1.flange_b)
    annotation (Line(points={{76,114},{54,114}}, color={0,127,0}));
  connect(line.port_b, teleCylinder1_1.port_a) annotation (Line(points={{24,96},
          {24,102.8},{23.4,102.8}}, color={255,0,0}));
  connect(pressureSensor2.port_a, j1.port[3]) annotation (Line(points={{-18,32},
          {-20,32},{-20,22},{-30,22},{-30,10.3333}}, color={255,0,0}));
  connect(positionSensor.flange, slidingMass.flange_a) annotation (Line(points=
          {{154,76},{70,76},{70,114},{76,114}}, color={0,127,0}));
  connect(positionSensor.s, PID.u_m) annotation (Line(points={{175,76},{222,76},
          {222,12},{168,12},{168,26}}, color={0,0,127}));
  connect(ramp.y, PID.u_s) annotation (Line(points={{141,-30},{206,-30},{206,38},
          {180,38}}, color={0,0,127}));
  connect(PID.y, throttleValve.control) annotation (Line(points={{157,38},{64,
          38},{64,24},{51.8,24}}, color={0,0,127}));
  annotation (
    experiment(StopTime=10, __Dymola_Algorithm="Dassl"),
    Diagram(coordinateSystem(extent={{-100,-100},{180,140}}), graphics={Text(
          extent={{46,54},{118,34}},
          textColor={28,108,200},
          textString="Left and Right position is switched here")}),
    Icon(coordinateSystem(extent={{-100,-100},{180,140}})));
end SingleCylinder_M4;
