within OpenHydraulics.DevelopmentTests;
model Prop43ValveTestSimpleDeadZone_PID_Block

  extends OpenHydraulics.Interfaces.PartialFluidCircuit(redeclare
      OpenHydraulics.Fluids.GenericOilSimple oil);

  OpenHydraulics.Components.Valves.ReliefValve reliefValve
    annotation (Placement(transformation(
        origin={-30,-16},
        extent={{-10,-10},{10,10}},
        rotation=270)));
  OpenHydraulics.Components.Cylinders.DoubleActingCylinder doubleActingCylinder(
    boreDiameter=0.12,
    strokeLength=5,
    closedLength=1,
    rodDiameter=0.05,
    pistonMass=0.3,
    s_init=2.10064,
    initType=Types.RevoluteInit.PositionVelocityAcceleration) annotation (Placement(transformation(extent={{22,76},
            {42,96}})));

  Modelica.Mechanics.Translational.Components.Mass slidingMass(m=1)
    annotation (Placement(transformation(extent={{60,76},{80,96}})));
  Modelica.Blocks.Sources.Sine sine(
    amplitude=1,
    f=0.2,
    offset=0,
    startTime=1)
    annotation (Placement(transformation(extent={{-34,-80},{-14,-60}})));
  Components.Valves.DirectionalValves.V4_3CC
                                           throttleValve(
    q_nom=0.01,
    dp_nom=10000000,
    P2A(table=[0.0,0; 1,1]),
    B2T(table=[0.0,0; 1,1]),
    P2B(table=[-1,1; -0.0,0]),
    A2T(table=[-1,1; -0.0,0]))
    annotation (Placement(transformation(extent={{20,-10},{40,10}})));
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
    annotation (Placement(transformation(extent={{50,-36},{70,-16}})));
  Components.Lines.Line line(L=2, D=0.05) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={24,56})));
  Components.Lines.Line line1(L=2, D=0.05) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={40,56})));
  Modelica.Blocks.Continuous.LimPID
                                 PID(
    controllerType=Modelica.Blocks.Types.SimpleController.P,
    k=30,
    Ti=0.01,
    Td=1,
    yMax=1,
    yMin=-1) annotation (Placement(transformation(extent={{10,-10},{-10,10}},
        rotation=0,
        origin={90,0})));
  Modelica.Mechanics.Translational.Sensors.PositionSensor positionSensor
    annotation (Placement(transformation(extent={{60,120},{80,140}})));
  Components.MotorsPumps.ConstantDisplacementPump pump annotation (Dialog,
      Placement(transformation(extent={{-130,-34},{-110,-14}})));
  Modelica.Mechanics.Rotational.Sources.ConstantSpeed constantSpeed(w_fixed=100,
      useSupport=false)
    annotation (Placement(transformation(extent={{-168,-34},{-148,-14}})));
  Components.Volumes.CircuitTank                circuitTank
    annotation (Placement(transformation(extent={{-80,-50},{-100,-30}})));
  Modelica.Blocks.Interfaces.RealInput u annotation (Placement(transformation(
        extent={{-20,-20},{20,20}},
        rotation=180,
        origin={210,0})));
  Modelica.Mechanics.Translational.Interfaces.Flange_a flange_a
    annotation (Placement(transformation(extent={{-110,36},{-90,56}})));
  Modelica.Mechanics.Translational.Interfaces.Flange_b flange_b
    annotation (Placement(transformation(extent={{92,38},{112,58}})));
equation
  connect(doubleActingCylinder.flange_b, slidingMass.flange_a)
    annotation (Line(points={{42,86},{60,86}}, color={0,127,0}));
  connect(reliefValve.port_a, j1.port[1]) annotation (Line(points={{-30,-6},{-30,
          1.625},{-30,9.66667},{-30,9.66667}},     color={255,0,0}));
  connect(reliefValve.port_b, j2.port[2]) annotation (Line(points={{-30,-26},
          {-30,-33.025},{-30,-40},{-30,-40}}, color={255,0,0}));
  connect(throttleValve.portP, j1.port[3]) annotation (Line(points={{26,-8},{
          -12,-8},{-12,10.3333},{-30,10.3333}},
                                            color={255,0,0}));
  connect(throttleValve.portT, j2.port[3]) annotation (Line(points={{34,-8},{38,
          -8},{38,-39.6667},{-30,-39.6667}}, color={255,0,0}));
  connect(throttleValve.portP, pressureSensor2.port_a)
    annotation (Line(points={{26,-8},{0,-8},{0,0}}, color={255,0,0}));
  connect(throttleValve.portT, pressureSensor3.port_a) annotation (Line(points={
          {34,-8},{38,-8},{38,-40},{60,-40},{60,-36}}, color={255,0,0}));
  connect(throttleValve.portA, line.port_a)
    annotation (Line(points={{26,8},{24,8},{24,46}}, color={255,0,0}));
  connect(line1.port_a, throttleValve.portB)
    annotation (Line(points={{40,46},{40,8},{34,8}}, color={255,0,0}));
  connect(pressureSensor1.port_a, throttleValve.portB)
    annotation (Line(points={{60,30},{40,30},{40,8},{34,8}}, color={255,0,0}));
  connect(pressureSensor.port_a, line.port_a)
    annotation (Line(points={{0,30},{24,30},{24,46}}, color={255,0,0}));
  connect(throttleValve.control, PID.y)
    annotation (Line(points={{41,0},{79,0}},      color={0,0,127}));
  connect(positionSensor.flange, slidingMass.flange_a) annotation (Line(points={
          {60,130},{56,130},{56,86},{60,86}}, color={0,127,0}));
  connect(positionSensor.s, PID.u_m) annotation (Line(points={{81,130},{172,130},
          {172,-24},{90,-24},{90,-12}}, color={0,0,127}));
  connect(line.port_b, doubleActingCylinder.port_a)
    annotation (Line(points={{24,66},{24,78}}, color={255,0,0}));
  connect(line1.port_b, doubleActingCylinder.port_b)
    annotation (Line(points={{40,66},{40,78}}, color={255,0,0}));
  connect(pump.portP, j1.port[1]) annotation (Line(points={{-120,-14},{-120,8},
          {-48,8},{-48,9.66667},{-30,9.66667}}, color={255,0,0}));
  connect(pump.flange_a, constantSpeed.flange)
    annotation (Line(points={{-130,-24},{-148,-24}}, color={0,0,0}));
  connect(circuitTank.port_a, j2.port[1]) annotation (Line(points={{-80,-40},{
          -78,-40.3333},{-30,-40.3333}}, color={255,0,0}));
  connect(circuitTank.port_b, pump.portT) annotation (Line(points={{-100,-40},{
          -120,-40},{-120,-34}}, color={255,0,0}));
  connect(u, PID.u_s)
    annotation (Line(points={{210,0},{102,0}}, color={0,0,127}));
  connect(doubleActingCylinder.flange_a, flange_a) annotation (Line(points={{22,
          86},{-50,86},{-50,46},{-100,46}}, color={0,127,0}));
  connect(slidingMass.flange_b, flange_b)
    annotation (Line(points={{80,86},{102,86},{102,48}}, color={0,127,0}));
  annotation (
    experiment(StopTime=20, __Dymola_Algorithm="Dassl"));
end Prop43ValveTestSimpleDeadZone_PID_Block;
