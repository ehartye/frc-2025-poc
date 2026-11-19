import tempfile
from unittest.mock import patch
import unittest,pathlib,importlib.util,json,subprocess,sys,ast
P=pathlib.Path(__file__).with_name('run-coral-contact.py');s=importlib.util.spec_from_file_location('intake',P);r=importlib.util.module_from_spec(s);s.loader.exec_module(r)
class Contract(unittest.TestCase):
 def test_exact_assumptions(self):r.validate(json.loads(json.dumps(r.EXPECTED)))
 def test_promotions_and_unknowns_reject(self):
  for edit in (lambda e:e.update(capture=True),lambda e:e.update(engineeringAcceptance='passed'),lambda e:e.update(rollerMu=float('nan')),lambda e:e.update(floorMu=[True,.2,.4,.8]),lambda e:e.update(unknown=1)):
   e=json.loads(json.dumps(r.EXPECTED));edit(e)
   with self.assertRaises(ValueError):r.validate(e)
 def test_optimized_execution_cannot_remove_gates(self):
  self.assertFalse(any(isinstance(n,ast.Assert) for n in ast.walk(ast.parse(P.read_text()))))
  code="import importlib.util;s=importlib.util.spec_from_file_location('r',"+repr(str(P))+");r=importlib.util.module_from_spec(s);s.loader.exec_module(r);r.validate({})"
  self.assertNotEqual(subprocess.run([sys.executable,'-O','-c',code],capture_output=True).returncode,0)

 def test_failure_before_java_never_claims_io_execution(self):
  original=r.sha
  def digest(path):return '0'*64 if pathlib.Path(path).name=='java.exe' else original(path)
  with tempfile.TemporaryDirectory() as td:
   out=pathlib.Path(td)/'failed-run'
   with patch.object(sys,'argv',['runner','--output',str(out),'--java-home','unused','--gradle-home','unused']),patch.object(r,'sha',side_effect=digest),patch.object(r,'validate',side_effect=ValueError('injected pre-Java failure')),patch.object(r.subprocess,'run') as invoke:
    with self.assertRaisesRegex(ValueError,'injected'):r.main()
    invoke.assert_not_called()
   evidence=json.loads((out/'execution-receipt.json').read_text());self.assertEqual(evidence['status'],'failed');self.assertIs(evidence['actualRobotIoExecuted'],False)
