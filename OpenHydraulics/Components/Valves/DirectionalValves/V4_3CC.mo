within OpenHydraulics.Components.Valves.DirectionalValves;
model V4_3CC "4-port 3-position closed center valve"
  extends
    OpenHydraulics.Components.Valves.DirectionalValves.BaseClasses.PartialValve4_3pos;

  // include just the arrows for each position
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{
            -100,-100},{100,100}}),
            extent={{-100,-100},{100,100}},
             graphics={
        Line(points=DynamicSelect({{-74,-30},{-74,30}},{{-74+control*60,-30},{-74+control*60,30}}), color={0,0,0}),
        Line(points=DynamicSelect({{-46,-30},{-46,30}},{{-46+control*60,-30},{-46+control*60,30}}), color={0,0,0}),
        Polygon(
          points=DynamicSelect({{-74,30},{-80,10},{-68,10},{-74,30}},{{-74+control*60,30},{-80+control*60,10},{-68+control*60,10},{-74+control*60,30}}),
          lineColor={0,0,0},
          fillColor={0,0,0},
          fillPattern=FillPattern.Solid),
        Polygon(
          points=DynamicSelect({{-46,-30},{-52,-10},{-40,-10},{-46,-30}},{{-46+control*60,-30},{-52+control*60,-10},{-40+control*60,-10},{-46+control*60,-30}}),
          lineColor={0,0,0},
          fillColor={0,0,0},
          fillPattern=FillPattern.Solid),
        Line(points=DynamicSelect({{74,-30},{46,30}},{{74+control*60,-30},{46+control*60,30}}), color={0,0,0}),
        Line(points=DynamicSelect({{46,-30},{74,30}},{{46+control*60,-30},{74+control*60,30}}), color={0,0,0}),
        Polygon(
          points=DynamicSelect({{74,-30},{58,-14},{70,-8},{74,-30}},{{74+control*60,-30},{58+control*60,-14},{70+control*60,-8},{74+control*60,-30}}),
          lineColor={0,0,0},
          fillColor={0,0,0},
          fillPattern=FillPattern.Solid),
        Polygon(
          points=DynamicSelect({{74,30},{70,6},{58,12},{74,30}},{{74+control*60,30},{70+control*60,6},{58+control*60,12},{74+control*60,30}}),
          lineColor={0,0,0},
          fillColor={0,0,0},
          fillPattern=FillPattern.Solid),
        Line(points=DynamicSelect({{-14,-30},{-14,-12}},{{-14+control*60,-30},{-14+control*60,-12}}), color={0,0,0}),
        Line(points=DynamicSelect({{-20,-12},{-8,-12}},{{-20+control*60,-12},{-8+control*60,-12}}), color={0,0,0}),
        Line(points=DynamicSelect({{-20,12},{-8,12}},{{-20+control*60,12},{-8+control*60,12}}), color={0,0,0}),
        Line(points=DynamicSelect({{8,12},{20,12}},{{8+control*60,12},{20+control*60,12}}), color={0,0,0}),
        Line(points=DynamicSelect({{8,-12},{20,-12}},{{8+control*60,-12},{20+control*60,-12}}), color={0,0,0}),
        Line(points=DynamicSelect({{-14,12},{-14,30}},{{-14+control*60,12},{-14+control*60,30}}), color={0,0,0}),
        Line(points=DynamicSelect({{14,12},{14,30}},{{14+control*60,12},{14+control*60,30}}), color={0,0,0}),
        Line(points=DynamicSelect({{14,-30},{14,-12}},{{14+control*60,-30},{14+control*60,-12}}), color={0,0,0}),
        Line(points=DynamicSelect({{-14,30},{-14,60},{-40,60},{-40,80}}, {{-14,
              30},{-14,60},{-40,60},{-40,80}}), color={255,0,0}),
        Line(points=DynamicSelect({{14,30},{14,60},{40,60},{40,80}}, {{14,30},{
              14,60},{40,60},{40,80}}), color={255,0,0}),
        Line(points=DynamicSelect({{-14,-30},{-14,-60},{-40,-60},{-40,-80}}, {{
              -14,-30},{-14,-60},{-40,-60},{-40,-80}}), color={255,0,0}),
        Line(points=DynamicSelect({{14,-30},{14,-60},{40,-60},{40,-80}}, {{14,-30},
              {14,-60},{40,-60},{40,-80}}), color={255,0,0})}));

end V4_3CC;
