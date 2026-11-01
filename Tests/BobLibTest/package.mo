within ;
package BobLibTest

  "Regression and component tests for BobLib"
  extends Modelica.Icons.ExamplesPackage;

  annotation(Documentation(info = "<html>
<p>
Package <code>BobLibTest</code> contains BobLib regression, component,
and physical smoke-test models. The package lives under the repository
<code>Tests</code> directory, but <code>Tests</code> itself is not a Modelica
package. Packages mirror BobLib. Each model or block has a Test-prefixed
fixture in the corresponding package; additional cases exercise variants.
See Tests/model_coverage.json for structural, component, and integration scope,
and Tests/modelica_test_library.py for profiling and package rollups.
</p>
</html>"));
  annotation(Icon(graphics = {Rectangle(extent = {{-100, 100}, {100, -100}}, lineColor = {0, 0, 0}, lineThickness = 0.5, fillPattern = FillPattern.None)}));
end BobLibTest;
