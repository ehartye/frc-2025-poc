"""Autodesk Fusion student master for one original tab saddle. All dimensions are in millimetres."""
import adsk.core
import adsk.fusion
import json
from pathlib import Path


def run(context):
    out = Path(context["output"] if isinstance(context, dict) and "output" in context else "FUSION_SADDLE_OUTPUT")
    app = adsk.core.Application.get()
    original = app.activeDocument
    doc = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    doc.name = "MAXSwerve tab saddle - original shop master"
    d = adsk.fusion.Design.cast(app.activeProduct)
    d.designType = adsk.fusion.DesignTypes.ParametricDesignType
    r = d.rootComponent
    p = adsk.core.Point3D.create
    value = adsk.core.ValueInput.createByString
    orient = adsk.fusion.DimensionOrientations
    op = adsk.fusion.FeatureOperations
    for name, number in dict(SaddleWidth=24.5,SaddleHeight=50.7,SaddleLength=35.7,
                             TabBore=5.3,FrameBore=6.5,LighteningBore=10.7).items():
        d.userParameters.add(name,value(str(number)+" mm"),"mm","Finished saddle dimension")
    sketch = r.sketches.add(r.xYConstructionPlane)
    sketch.name = "Width and captured tab height"
    lines = sketch.sketchCurves.sketchLines.addTwoPointRectangle(p(0,0,0),p(2.45,5.07,0))
    for a,b,kind,text,parameter in [(lines.item(0).startSketchPoint,lines.item(0).endSketchPoint,
                                    orient.HorizontalDimensionOrientation,p(1.2,-.4,0),"SaddleWidth"),
                                    (lines.item(1).startSketchPoint,lines.item(1).endSketchPoint,
                                    orient.VerticalDimensionOrientation,p(2.9,2.5,0),"SaddleHeight")]:
        sketch.sketchDimensions.addDistanceDimension(a,b,kind,text).parameter.expression=parameter
    extr = r.features.extrudeFeatures
    block = extr.addSimple(sketch.profiles.item(0),value("SaddleLength"),op.NewBodyFeatureOperation)
    block.name = "Mill full stock"
    def holes(label,plane,points,dimension,diameter):
        sk = r.sketches.add(plane)
        sk.name=label
        for point in points:
            model=p(*[v/10 for v in point])
            ctr=sk.modelToSketchSpace(model)
            circle=sk.sketchCurves.sketchCircles.addByCenterRadius(ctr,diameter/20)
            sk.sketchDimensions.addDiameterDimension(circle,p(ctr.x+.5,ctr.y+.4,0)).parameter.expression=dimension
        profiles=adsk.core.ObjectCollection.create()
        for pr in sk.profiles:profiles.add(pr)
        inp=extr.createInput(profiles,op.CutFeatureOperation)
        inp.setSymmetricExtent(value("200 mm"),True)
        cut=extr.add(inp);cut.name=label
    holes("Fixture-drill both tab through holes",r.xZConstructionPlane,
          [(16.25,0,5.15),(16.25,0,30.55)],"TabBore",5.3)
    holes("Cross-drill frame screws",r.yZConstructionPlane,
          [(0,14.2,17.85),(0,35.2,17.85)],"FrameBore",6.5)
    holes("Relieve the two webs",r.yZConstructionPlane,
          [(0,25.35,5.8),(0,25.35,29.9)],"LighteningBore",10.7)
    r.bRepBodies.item(0).name="Tab saddle"
    path=out/"tab-saddle.f3d"
    if path.exists():raise ValueError("Output exists")
    if not d.exportManager.execute(d.exportManager.createFusionArchiveExportOptions(str(path))):raise ValueError("Archive export")
    recovered=app.importManager.importToNewDocument(app.importManager.createFusionArchiveImportOptions(str(path)))
    dd=adsk.fusion.Design.cast(app.activeProduct);rr=dd.rootComponent
    volume=rr.bRepBodies.item(0).getPhysicalProperties(
        adsk.fusion.CalculationAccuracy.VeryHighCalculationAccuracy).volume*1000
    if rr.bRepBodies.count!=1 or abs(volume-36528.89836)>.05:raise ValueError("Saddle STEP/native feature disagreement "+str(volume))
    rec={"bodyCount":rr.bRepBodies.count,"volumeMm3":volume,"nativeSketches":rr.sketches.count,
         "nativeExtrusions":rr.features.extrudeFeatures.count,"parameters":{a.name:a.expression for a in dd.userParameters}}
    (out/"saddle-fusion.json").write_text(json.dumps(rec,indent=2))
    # This owned document is fully local and source-bound; return to the student's robot.
    recovered.close(False);doc.close(False);original.activate()
