within OpenHydraulics.Components.Sensors;
model PressureSensor
  extends OpenHydraulics.Interfaces.PartialFluidComponent;

  // the parameters
  parameter Boolean absolute = true "Measure absolute pressure if true";
  parameter SI.Volume V = 0 "Volume for non-ideal pressure sensor"
    annotation(Dialog(tab="Advanced"));
  parameter SI.Pressure pHigh = 1000000 "maximum pressure for color animation"
    annotation(Dialog(tab="Animation"));
  Real pres = port_a.p/pHigh;
  // the ports
  OpenHydraulics.Interfaces.FluidPort port_a
    annotation (Placement(transformation(extent={{-10,-110},{10,-90}})));
  Modelica.Blocks.Interfaces.RealOutput y
    annotation (Placement(transformation(extent={{60,-10},{80,10}})));

  Basic.VolumeClosed volumeClosed(
    final V=V,
    final n_ports=1)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180)));
equation
  y = port_a.p - (if absolute then 0 else environment.p_ambient);

  connect(port_a, volumeClosed.port[1])
    annotation (Line(points={{0,-100},{0,-49.625},{0,0.05},{0,0.05}}, color=
         {255,0,0}));

  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},
            {100,100}}),       graphics={
        Ellipse(
          extent={{-54,54},{54,-54}},
          lineColor={0,0,0},
          fillColor=DynamicSelect({255,255,255},{pres*255,0,255-pres*255}),
          fillPattern=FillPattern.Solid),
        Line(points={{32,34},{-32,-30}}, color={0,0,0}),
        Polygon(
          points={{-32,-32},{-24,-14},{-14,-24},{-32,-32}},
          lineColor={0,0,0},
          fillColor={0,0,0},
          fillPattern=FillPattern.Solid),
        Line(points={{0,-54},{0,-100}}, color={255,0,0}),
        Text(
          extent={{0,56},{0,82}},
          lineColor={0,0,255},
          fillColor={0,0,0},
          fillPattern=FillPattern.Solid,
          textString="%name"),
        Line(points={{60,0},{54,0}}, color={0,0,127}),
        Text(
          extent={{-100,-80},{100,-48}},
          lineColor={0,0,0},
          textString=DynamicSelect("p", String(y/100000)))}),
                            Diagram(graphics={Line(
          points={{60,0},{10,0}},
          color={0,0,255},
          pattern=LinePattern.Dash)}));
end PressureSensor;
