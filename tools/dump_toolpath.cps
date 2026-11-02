/** Diagnostic XYZ export for checking generated Trace motion. Not machine code. */
description = "Trace XYZ verification dump";
vendor = "";
extension = "csv";
capabilities = CAPABILITY_MILLING;
certificationLevel = 2;
tolerance = spatial(0.001, MM);
allowedCircularPlanes = 0;
var xyz = createFormat({decimals: 6, forceDecimal: true});

function point(kind, x, y, z) {
  writeln([kind, xyz.format(x), xyz.format(y), xyz.format(z)].join(","));
}
function onOpen() {
  writeln("type,x,y,z");
}
function onSection() {
  var p = getFramePosition(currentSection.getInitialPosition());
  point("INITIAL", p.x, p.y, p.z);
}
function onRapid(x, y, z) {
  point("RAPID", x, y, z);
}
function onLinear(x, y, z, feed) {
  point("LINEAR", x, y, z);
}
function onCircular(clockwise, cx, cy, cz, x, y, z, feed) {
  linearize(tolerance);
}
function onRapid5D(x, y, z, a, b, c) {
  error("Only three-axis toolpaths can be checked by this diagnostic post.");
}
function onLinear5D(x, y, z, a, b, c, feed) {
  error("Only three-axis toolpaths can be checked by this diagnostic post.");
}
function onClose() {}
