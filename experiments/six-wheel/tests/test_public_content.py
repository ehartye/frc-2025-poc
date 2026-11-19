import importlib.util,pathlib,subprocess,tempfile,unittest
from unittest.mock import patch
ROOT=pathlib.Path(__file__).resolve().parents[1]
s=importlib.util.spec_from_file_location('guard',ROOT/'scripts/check_public_content.py');guard=importlib.util.module_from_spec(s);s.loader.exec_module(guard)
class PublicContent(unittest.TestCase):
 def test_committed_content(self):self.assertEqual(guard.audit(),[])
 def test_audit_uses_one_commit_when_head_moves(self):
  with tempfile.TemporaryDirectory() as directory:
   repo = pathlib.Path(directory)
   def run(*args):
    return subprocess.check_output(['git', '-C', directory, *args])
   run('init', '--quiet')
   run('config', 'user.name', 'Fixture')
   run('config', 'user.email', 'fixture@example.org')
   sample = repo / 'bytes with spaces.txt'
   sample.write_bytes(b'first\n\0second\n')
   run('add', '.')
   run('commit', '-qm', 'Original')
   original = run('rev-parse', 'HEAD').decode().strip()
   sample.unlink()
   run('add', '-u')
   run('commit', '-qm', 'Replacement')
   replacement = run('rev-parse', 'HEAD').decode().strip()
   run('update-ref', 'HEAD', original)
   def move_head_after_resolution(*args):
    result = run(*args)
    if args[0] == 'rev-parse':
     run('update-ref', 'HEAD', replacement, original)
    return result
   with patch.object(guard, 'ROOT', repo), patch.object(guard, 'git', move_head_after_resolution):
    self.assertEqual(guard.audit(), [(sample.name, ['unexpected-binary'])])
    self.assertEqual(list(guard.committed_files(original)), [(sample.name, b'first\n\0second\n')])
 def test_synthetic_private_patterns(self):
  examples=['C:'+chr(92)+'Users'+chr(92)+'example'+chr(92)+'file','/ho'+'me/example/file','192'+'.168.10.2','https://device'+'.l'+'an/path','Run on '+'BUILD_NODE','execution on '+'TEST_BOX']
  for value in examples:
   with self.subTest(category=value.split()[0]):self.assertTrue(guard.findings(value.encode()))
 def test_portable_language_and_sources(self):
  self.assertEqual(guard.findings(b'Run on Windows with an existing JDK 17. https://github.com/example/project'),[])
if __name__=='__main__':unittest.main()
