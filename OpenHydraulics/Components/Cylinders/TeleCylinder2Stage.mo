within OpenHydraulics.Components.Cylinders;
model TeleCylinder2Stage
  import Modelica.Constants.pi;

  // the parameters
  parameter SI.Length boreDiameter1 = 0.08 "Bore diameter"
    annotation (Dialog(tab="Sizing",group="Dimensions"));

    parameter SI.Length boreDiameter2 = 0.05 "Bore diameter"
    annotation (Dialog(tab="Sizing",group="Dimensions"));

    parameter SI.Length boreDiameter3 = 0.02 "Bore diameter"
    annotation (Dialog(tab="Sizing",group="Dimensions"));

  parameter SI.Length stage1StrokeLength = 1.0 "Stroke length of the cylinder"
    annotation (Dialog(tab="Sizing",group="Dimensions"));
      parameter SI.Length stage2StrokeLength = 1.0 "Stroke length of the cylinder"
    annotation (Dialog(tab="Sizing",group="Dimensions"));

  parameter SI.Length closedLength = 1.0
    "Total length of cylinder fully retracted"
    annotation (Dialog(tab="Sizing",group="Dimensions"));
  parameter SI.VolumeFlowRate q_nom = 0.01 "Nominal flow rate for in/outlet"
    annotation (Dialog(tab="Sizing",group="Hydraulics"));
  parameter SI.Pressure dp_nom = 1e4 "Nominal pressure drop for q_nom"
    annotation (Dialog(tab="Sizing",group="Hydraulics"));
  parameter SI.AbsolutePressure maxPressure = 3e7 "Maximum rated pressure"
    annotation (Dialog(tab="Sizing",group="Hydraulics"));

  // dynamics parameters
  parameter SI.Mass pistonMass = 0 "Mass of the piston and rod"
    annotation (Dialog(tab="Dynamics"));
  parameter SI.TranslationalDampingConstant damping(
    final min=0) = 1e4 "damping between piston and cylinder"
    annotation (Dialog(tab="Dynamics"));

  parameter SI.Distance endOfTravelDistance = 0.01
    "Maximum distance beyond the end of travel point"
    annotation (Dialog(tab="Dynamics",group="End-of-travel"));
  parameter Real stopStiffness(
    final unit="N/m",
    final min=0) = 1e9 "stiffness at impact"
    annotation(Dialog(tab="Dynamics",group="End-of-travel"));
  parameter Real stopDamping(
    final unit="N.s/m",
    final min=-1000) = 1e12 "damping at impact"
    annotation(Dialog(tab="Dynamics",group="End-of-travel"));

  // cushion parameters
  parameter Boolean useCushionHead = true
    "false = constant restriction with q_nom & dp_nom"
    annotation(Evaluate=true, Dialog(tab="Cushions",group="Head Cushion"));
  parameter Real cushionTableHead[:, :]=[0,0.001;0.001,0.001;0.029,0.01;0.03,1]
    "Cushion flow rate (1st col = s_rel; 2nd col = fraction of q_nom)"
    annotation(Dialog(tab="Cushions",group="Head Cushion",enable=useCushionHead));
  parameter Modelica.Blocks.Types.Smoothness smoothnessHead=
    Modelica.Blocks.Types.Smoothness.LinearSegments
    "smoothness of table interpolation"
    annotation(Dialog(tab="Cushions",group="Head Cushion",enable=useCushionHead));
  parameter Boolean useCushionRod = true
    "false = constant restriction with q_nom & dp_nom"
    annotation(Evaluate=true, Dialog(tab="Cushions",group="Rod Cushion"));
  parameter Real cushionTableRod[:, :]=[0,0.001;0.001,0.001;0.029,0.01;0.03,1]
    "Cushion flow rate (1st col = s_rel; 2nd col = fraction of q_nom)"
    annotation(Dialog(tab="Cushions",group="Rod Cushion",enable=useCushionRod));
  parameter Modelica.Blocks.Types.Smoothness smoothnessRod=
    Modelica.Blocks.Types.Smoothness.LinearSegments
    "smoothness of table interpolation"
    annotation(Dialog(tab="Cushions",group="Rod Cushion",enable=useCushionRod));

  // sealing parameters
  parameter SI.Length L_A2B = 0.01 "Length of seal between chambers A and B"
    annotation (Dialog(tab="Seals",group="Piston"));
  parameter SI.Diameter D_A2B = 1e-5
    "Hydraulic diameter of seal between chambers A and B"
    annotation (Dialog(tab="Seals",group="Piston"));
  parameter SI.Length L_A2Env = 0.01
    "Length of seal between chamber A and Environment"
    annotation (Dialog(tab="Seals",group="Piston"));
  parameter SI.Diameter D_A2Env = 0
    "Hydraulic diameter of seal between chamber A and Environment"
    annotation (Dialog(tab="Seals",group="Piston"));
  parameter SI.Length L_B2Env = 0.01
    "Length of seal between chamber B and Environment"
    annotation (Dialog(tab="Seals",group="Piston"));
  parameter SI.Diameter D_B2Env = 0
    "Hydraulic diameter of seal between chamber B and Environment"
    annotation (Dialog(tab="Seals",group="Piston"));

  // initialization parameters
  parameter Types.RevoluteInit initType=Types.RevoluteInit.Free "Type of initialization (defines usage of start values below)" annotation (Dialog(tab="Initialization", group="Mechanical"));
  parameter SI.Distance s_init1 = 0 "Initial position >0 and <stroke"
    annotation (Dialog(tab="Initialization",group="Mechanical"));
  parameter SI.Distance s_init2 = 0 "Initial position >0 and <stroke"
    annotation (Dialog(tab="Initialization",group="Mechanical"));

  parameter SI.Velocity v_init = 0 "Initial velocity"
    annotation (Dialog(tab="Initialization",group="Mechanical"));
  parameter SI.Acceleration a_init = 0 "Initial acceleration"
    annotation (Dialog(tab="Initialization",group="Mechanical"));
  parameter Boolean fixHeadPressure = false
    "Initialize the pressure at the head side"
    annotation (Dialog(tab="Initialization",group="Fluid"));
  parameter Boolean fixRodPressure = false
    "Initialize the pressure at the rod side"
    annotation (Dialog(tab="Initialization",group="Fluid"));

  // the connectors
  OpenHydraulics.Interfaces.FluidPort port_a
    annotation (Placement(transformation(extent={{-90,-90},{-70,-70}})));

  // the components
  Basic.FluidPower2MechTrans chamberStage1(
    A=pi/4*(boreDiameter1^2 - boreDiameter2^2),
    stopStiffness=stopStiffness,
    stopDamping=stopDamping,
    n_ports=2,
    p_init=p_init,
    maxPressure=maxPressure*10)
    annotation (Placement(transformation(extent={{-62,-10},{-42,10}})));

  Basic.FluidPower2MechTrans air_side1(
    A=0,
    stopStiffness=stopStiffness,
    stopDamping=stopDamping,
    n_ports=1,
    p_init=p_init,
    maxPressure=maxPressure*10)
    annotation (Placement(transformation(extent={{-4,-10},{16,10}})));
  BaseClasses.CylinderCushion cushionHead(
    cushionTable=if useCushionHead then cushionTableHead else [0,0.001;
        stage1StrokeLength/1000,1; 1,1],
    smoothness=smoothnessHead,
    q_nom=q_nom,
    dp_nom=dp_nom,
    dp_relief=maxPressure*0.9)
    annotation (Placement(transformation(extent={{-62,-60},{-42,-40}})));
  Modelica.Mechanics.Translational.Components.Mass stage1_cylinder(m=pistonMass)
    annotation (Placement(transformation(extent={{-32,-10},{-12,10}})));
  Modelica.Mechanics.Translational.Components.Rod stage1(L=stage1StrokeLength)
    annotation (Placement(transformation(extent={{-80,40},{-60,60}})));
  Modelica.Mechanics.Translational.Components.Rod rod(
                                           L=closedLength)
    annotation (Placement(transformation(extent={{70,-10},{90,10}})));
  Modelica.Mechanics.Translational.Interfaces.Flange_a flange_a
    "(left) driving flange (flange axis directed INTO cut plane, e. g. from left to right)"
    annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
  Modelica.Mechanics.Translational.Interfaces.Flange_b flange_b
    "(right) driven flange (flange axis directed OUT OF cut plane, i. e. from right to left)"
    annotation (Placement(transformation(extent={{90,-10},{110,10}})));
  Modelica.Mechanics.Translational.Components.Damper damper(
                                                 d=damping)
    annotation (Placement(transformation(extent={{-60,14},{-40,34}})));
  Basic.ConstPressureSource envSinkA
    annotation (Placement(transformation(
        origin={-94,-20},
        extent={{-10,-10},{10,10}},
        rotation=270)));
  Basic.ConstPressureSource envSinkB
    annotation (Placement(transformation(
        origin={90,50},
        extent={{-10,-10},{10,10}},
        rotation=0)));
  Basic.LaminarRestriction leakage_Head2Env(
    L=L_A2Env,
    D=D_A2Env)
    annotation (Placement(transformation(extent={{-78,-30},{-58,-10}})));
  Lines.NJunction jA(n_ports=3)
    annotation (Placement(transformation(extent={{-50,-90},{-30,-70}})));

  Real pos1=chamberStage1.s_rel/stage1StrokeLength;
  Real pos2=chamberStage2.s_rel/stage2StrokeLength;

  extends OpenHydraulics.Interfaces.PartialFluidComponent;

  Modelica.Mechanics.Translational.Components.Mass stage2_cylinder(m=pistonMass)
    annotation (Placement(transformation(extent={{20,-40},{40,-20}})));
  Basic.FluidPower2MechTrans chamberStage2(
    A=pi/4*(boreDiameter2^2 - boreDiameter3^2),
    stopStiffness=stopStiffness,
    stopDamping=stopDamping,
    n_ports=1,
    p_init=p_init,
    maxPressure=maxPressure*10)
    annotation (Placement(transformation(extent={{-6,-40},{14,-20}})));
  Basic.FluidPower2MechTrans air_side2(
    A=0,
    stopStiffness=stopStiffness,
    stopDamping=stopDamping,
    n_ports=1,
    p_init=p_init,
    maxPressure=maxPressure*10)
    annotation (Placement(transformation(extent={{44,-40},{64,-20}})));
  Modelica.Mechanics.Translational.Components.Rod stage2(L=stage2StrokeLength)
    annotation (Placement(transformation(extent={{14,30},{34,50}})));
protected
  outer OpenHydraulics.Circuits.Environment environment;

initial equation
  assert(chamberStage1.s_rel >= 0, "Initial position is smaller than zero");
  assert(air_side1.s_rel >= 0, "Initial position is larger than strokeLength");
  assert(chamberStage2.s_rel >= 0, "Initial position is smaller than zero");
  assert(air_side2.s_rel >= 0, "Initial position is larger than strokeLength");

  // state initialization
  if initType == Types.RevoluteInit.Position then
    chamberStage1.s_rel = s_init1;
    chamberStage2.s_rel = s_init2;
  elseif initType == Types.RevoluteInit.Velocity then
    chamberStage1.v_rel = v_init;
    chamberStage2.v_rel = v_init;
  elseif initType == Types.RevoluteInit.PositionVelocity then
    chamberStage1.s_rel = s_init1;
    chamberStage1.v_rel = v_init;
    chamberStage2.s_rel = s_init2;
    chamberStage2.v_rel = v_init;
  elseif initType == Types.RevoluteInit.VelocityAcceleration then
    chamberStage1.v_rel = v_init;
    stage1_cylinder.a = a_init;
  elseif initType == Types.RevoluteInit.SteadyState then
    chamberStage1.v_rel = 0;
    stage1_cylinder.a = a_init;
  elseif initType == Types.RevoluteInit.PositionVelocityAcceleration then
    chamberStage1.s_rel = s_init1;
    chamberStage1.s_rel = s_init2;
    chamberStage1.v_rel = v_init;
    stage1_cylinder.a = a_init;
  elseif initType == Types.RevoluteInit.Free then
    // nothing
  else
    assert(true,"Invalid initialization type in FluidPower2MechTrans");
  end if;

  if fixHeadPressure then
    chamberStage1.p_vol = p_init;
  end if;
  if fixRodPressure then
    air_side1.p_vol = p_init;
  end if;

equation
  when cushionHead.reliefValve.valvePositionSteadyState>0 then
    Modelica.Utilities.Streams.print("\nWARNING: Cylinder exceeds maximum pressure at the head end.");
    Modelica.Utilities.Streams.print("         This could just be due to end-of-travel behavior.");
    Modelica.Utilities.Streams.print("         (time = "+String(time)+")");
  end when;

  connect(chamberStage1.flange_b, stage1_cylinder.flange_a)
    annotation (Line(points={{-42,0},{-32,0}}, color={0,127,0}));
  connect(stage1_cylinder.flange_b, air_side1.flange_a)
    annotation (Line(points={{-12,0},{-4,0}}, color={0,127,0}));
  connect(stage1.flange_b, air_side1.flange_b) annotation (Line(points={{-60,50},
          {24,50},{24,0},{16,0}}, color={0,127,0}));
  connect(chamberStage1.flange_a, stage1.flange_a) annotation (Line(points={{-62,
          0},{-86,0},{-86,50},{-80,50}}, color={0,127,0}));
  connect(chamberStage1.flange_a, flange_a)
    annotation (Line(points={{-62,0},{-100,0}}, color={0,127,0}));
  connect(rod.flange_b, flange_b)
    annotation (Line(points={{90,0},{100,0}}, color={0,127,0}));
  connect(flange_a, damper.flange_a)
    annotation (Line(points={{-100,0},{-66,0},{-66,24},{-60,24}},
                                      color={0,127,0}));
  connect(damper.flange_b, chamberStage1.flange_b) annotation (Line(points={{-40,24},
          {-38,24},{-38,0},{-42,0}},     color={0,127,0}));
  connect(envSinkA.port, leakage_Head2Env.port_a)
    annotation (Line(points={{-84,-20},{-78,-20}},
        color={255,0,0}));
  connect(port_a,jA. port[1]) annotation (Line(points={{-80,-80},{-40,-80},{-40,
          -80.3333}},   color={255,0,0}));

  connect(chamberStage1.port[1], cushionHead.port_a)
    annotation (Line(points={{-52,-0.3125},{-52,-40}},
                                                    color={255,0,0}));

  connect(cushionHead.port_b, jA.port[2]) annotation (Line(points={{-52,-60},{-52,
          -66},{-40,-66},{-40,-80}},
                        color={255,0,0}));
  connect(chamberStage1.flange_a, cushionHead.flange_a)
    annotation (Line(points={{-62,0},{-66,0},{-66,-50},{-62,-50}},
                                                 color={0,127,0}));
  connect(chamberStage1.flange_b, cushionHead.flange_b)
    annotation (Line(points={{-42,0},{-38,0},{-38,-50},{-42,-50}},
                                                 color={0,127,0}));
  connect(air_side1.port[1], envSinkB.port) annotation (Line(points={{6,-0.05},{
          6,70},{90,70},{90,60}}, color={255,0,0}));
  connect(damper.flange_b, stage1_cylinder.flange_a) annotation (Line(points={{-40,
          24},{-38,24},{-38,0},{-32,0}}, color={0,127,0}));
  connect(chamberStage2.flange_a, stage1_cylinder.flange_b) annotation (Line(
        points={{-6,-30},{-10,-30},{-10,0},{-12,0}}, color={0,127,0}));
  connect(chamberStage2.flange_b, stage2_cylinder.flange_a)
    annotation (Line(points={{14,-30},{20,-30}}, color={0,127,0}));
  connect(jA.port[3], chamberStage2.port[1]) annotation (Line(points={{-40,
          -79.6667},{-40,-66},{4,-66},{4,-30.05}},
                                         color={255,0,0}));
  connect(stage2_cylinder.flange_b, air_side2.flange_a)
    annotation (Line(points={{40,-30},{44,-30}}, color={0,127,0}));
  connect(envSinkB.port, air_side2.port[1]) annotation (Line(points={{90,60},{90,
          70},{118,70},{118,-30.05},{54,-30.05}}, color={255,0,0}));
  connect(stage2.flange_a, stage1_cylinder.flange_b)
    annotation (Line(points={{14,40},{-12,40},{-12,0}}, color={0,127,0}));
  connect(leakage_Head2Env.port_b, chamberStage1.port[2]) annotation (Line(
        points={{-58,-20},{-52,-20},{-52,0.2125}}, color={255,0,0}));
  connect(air_side2.flange_b, stage2.flange_b) annotation (Line(points={{64,-30},
          {60,-30},{60,40},{34,40}}, color={0,127,0}));
  connect(stage2_cylinder.flange_b, rod.flange_a) annotation (Line(points={{40,
          -30},{38,-30},{38,0},{70,0}}, color={0,127,0}));
  annotation (         Icon(coordinateSystem(preserveAspectRatio=false,
          extent={{-100,-100},{100,100}}), graphics={
        Rectangle(
          extent={{-90,80},{90,-90}},
          lineColor={255,255,255},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-90,40},{90,-40}},
          lineColor={0,0,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent=DynamicSelect({{90,30},{-80,-30}}, {{90 + pos1*170,30},{-80 +
              pos1*170,-30}}),
          lineColor={0,0,0},
          fillColor={184,184,184},
          fillPattern=FillPattern.Solid),
        Line(points={{-80,-40},{-80,-40},{-80,-40},{-80,-80}}, color={255,0,
              0}),
        Text(
          extent={{-64,-56},{-34,-96}},
          lineColor={0,0,0},
          fillColor={0,0,0},
          fillPattern=FillPattern.Solid,
          textString="A"),
        Text(
          extent={{0,84},{0,60}},
          lineColor={0,0,255},
          fillColor={0,0,0},
          fillPattern=FillPattern.Solid,
          textString="%name"),
        Rectangle(
          extent=DynamicSelect({{-34,18},{16,-18}},{{0,0},{0,0}}),
          lineColor={0,0,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent=DynamicSelect({{90,22},{-70,-22}}, {{90 + pos1*170 + pos2*160,
              22},{-70 + pos1*170 + pos2*160,-22}}),
          lineColor={0,0,0},
          fillColor={135,135,135},
          fillPattern=FillPattern.Solid)}));
end TeleCylinder2Stage;
