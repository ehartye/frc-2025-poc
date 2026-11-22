"""Audit committed public text/metadata without printing matched private values."""
import argparse,hashlib,pathlib,re,subprocess,sys
ROOT=pathlib.Path(__file__).resolve().parents[1]
PATTERNS={
 'personal-absolute-path':r'(?<![A-Za-z])[A-Z]:[\\/](?:Users|Documents and Settings)[\\/]|/(?:Users|home)/[^\s/]+',
 'private-network-address':r'\b(?:192\.168\.\d{1,3}\.\d{1,3}|10\.\d{1,3}\.\d{1,3}\.\d{1,3}|172\.(?:1[6-9]|2\d|3[01])\.\d{1,3}\.\d{1,3})\b',
 'private-network-url':r'https?://[^/\s]+\.(?:local|lan|home)(?:[:/\s]|$)|\\\\[A-Za-z0-9_-]+\\(?:Users|home|share)',
 'unexplained-machine-alias':r'\b(?:Run on|execution on|tested on|verified on)\s+[A-Z][A-Z0-9_-]{1,}(?=[\s.,;]|$)',
 'internal-coordination':r'/(?:root|user_notes)/|private[-_]workspace|home[ ]network',
 'credential':r'-----BEGIN (?:RSA |EC |OPENSSH )?PRIVATE KEY-----|\bgh[pousr]_[A-Za-z0-9]{20,}|\bgithub_pat_[A-Za-z0-9_]{20,}'
}
# Reviewed binaries have exact, item-specific identities.
WRAPPER='experiments/six-wheel/shared/examples/java-2026-sim/gradle/wrapper/gradle-wrapper.jar'
WRAPPER_SHA='2db75c40782f5e8ba1fc278a5574bab070adccb2d21ca5a6e5ed840888448046'
SHOWCASE_ASSETS = {'docs/robot/cad/assembly.png': '6ce25b430ec46d594b42f348221d4acfc7d5bf97363e3c6119bb8c9881c5ca34', 'docs/robot/cad/assembly.stl.gz': 'd3a2bad9c3c084f2b94756cec5da9d08698a8d98b3afce2492e98ff9f5d3cbea', 'experiments/swerve-drive/packaging/tab-saddle.f3d': 'c15bf7f1923c892e96f5081a75d30dd7fa3c1b01d3616abef9bf057f5abfb232'}

# Project-built whole robot, authorized STEP inputs and a separate practice pipe.
SHOWCASE_ASSETS['cad/swerve-captive/Team-1780-public-captive-swerve.f3d'] = 'baffdfad1d78ab9807ae3cb6f2c3e3f0e37008bf9ef7ed3f70586f9d1d1ac9e5'
SHOWCASE_ASSETS['experiments/swerve-drive/retention/capture.png'] = '7945e7c2172e717400b7cc947d8648392455603f410b2f7f2f4f796581f47e17'
SHOWCASE_ASSETS['docs/robot/station-witness.png'] = '391f2d1fb95fda7c1a1352a9c34ed7ba1f5a9a3f550a942baa02a00ee4d41bec'
SHOWCASE_ASSETS['docs/robot/receiver-exit.png'] = '83ae4d0a7f18fe5684d145f2e61d432abe3dd6e1843bdcda9ff0aa5457d8eb47'
# Two project-machined bodies, six native features and no supplier geometry.
SHOWCASE_ASSETS['experiments/swerve-drive/score-stroke/cam-lands.f3d'] = 'b5453de8239aa798db60d625ca5925f08bf137860e27168e41a3d6db34a700a5'
SHOWCASE_ASSETS['experiments/swerve-drive/motion/opening.webm'] = '5c78a3df9c463879e6b3507cbb14a009abb4482a8f94e90f71a428c88579d26c'

SHOWCASE_ASSETS['experiments/swerve-drive/motion/fallback.webm'] = '3186a0cdaba2a2d0f8a5798173f9086d3390b71e6a89313f3c2631026e6058f5'

# Team-authored animation of public robot solids, finite tube and reconstructed target.
SHOWCASE_ASSETS['experiments/swerve-drive/score-stroke/receiver-motion.webm'] = '24330bbb69b2edc0cd9c064f04eea9dd11c38609173a53cd97fff153be884414'
SHOWCASE_ASSETS['docs/robot/receiver-motion.png'] = 'b21a798de53c5fbf182df77d14efbbd69a13c6daecbb036b7d9555c6d63bd3b3'

# Reopened project-owned full climber alternative and rendered native-group motion.
SHOWCASE_ASSETS['cad/shallow-cage/Team-1780-shallow-cage-study.f3d'] = '88cc5f0927094bd2bbc60da862a137e5eada4484cc84510a514d760f27117c49'
SHOWCASE_ASSETS['cad/shallow-cage/shallow-climb.png'] = 'e3578357e0f47dae130917b93f51b29d9c1c96377f9a9bfe05b666aac0668819'
SHOWCASE_ASSETS['cad/shallow-cage/shallow-climb.webm'] = '1752de27f32e99a38903e32b764d9394a21158305f6b92dd4a2b7c69c2811bc4'

SHOWCASE_ASSETS['cad/assembly-engineering/receiver-service/issue/assembly-drawings.pdf'] = 'aa1411c3725d1299de75371ed1c3c6ad4a0e139ebdf778080a1e7c915e9098f2'
SHOWCASE_ASSETS['cad/assembly-engineering/receiver-service/issue/installed.png'] = 'e865b212c46aeba7d2f765d221c6419272f4dca6c412a99032357d20c150138b'
SHOWCASE_ASSETS['cad/assembly-engineering/receiver-service/issue/installed.stl.gz'] = 'a642bed457a496fb8c1e44984f27a7fb7cd0a2fcc33185bb158367c63ef3cb9f'
SHOWCASE_ASSETS['cad/assembly-engineering/issue/assembly-drawings.pdf'] = 'cbb33076ad624cd143f05d9db4264bf4bd63b90e5eacbe0fcb1482a62e8f8f89'
SHOWCASE_ASSETS['cad/assembly-engineering/issue/installed.png'] = 'f2a91749de2c9341887ed89c9b8dfc460ac263b0fd0ab4657a17870f22b615b3'
SHOWCASE_ASSETS['cad/assembly-engineering/issue/installed.stl.gz'] = '65b7253c8eb7806b928c82b7eb1830962e270d327c80fd7848bd15924b3d7a69'
SHOWCASE_ASSETS['cad/assembly-engineering/receiver-service/issue/Team-1780-receiver-service.f3d'] = 'a5d4c627f7266414270b7f9389c9396b25ce50fd1ca429f64646b1469e77a065'
SHOWCASE_ASSETS['cad/assembly-engineering/issue/Team-1780-engineering-shallow.f3d'] = '29c777fc5e875e5f2131ee2e34c8d3eaaf46e5ff5f8edffdf3399a17c6182254'
SHOWCASE_ASSETS['cad/shallow-cage/climb-kit.stl.gz'] = '5cfdf0358350c50ea3c33ed343dcd8abeda5259c49240ac6777cdd8ace0513c5'

# Agent FRC 0.89.0 preliminary release tree: the plugin's OFL template fonts and the checked release's compressed display geometry (pinned by its receipt).
SHOWCASE_ASSETS['docs/preliminary-design/fonts/barlow-condensed-latin-600-normal.woff2'] = '0b281bf2f4179c06ba68f0a427f2341287c41eacc2ce9d534c6f5c513ac633fb'
SHOWCASE_ASSETS['docs/preliminary-design/fonts/barlow-condensed-latin-700-normal.woff2'] = '8320299532b4b81498d5b3714d49c9d5938883b55f4c2a1efe6f105bf4a942bd'
SHOWCASE_ASSETS['docs/preliminary-design/fonts/source-sans-3-latin-400-normal.woff2'] = '6edbd48bf7fa0e34a5ad9f8da9e73a70d3ae7f51791deb8362c868fed9a7f68c'
SHOWCASE_ASSETS['docs/preliminary-design/fonts/source-sans-3-latin-600-normal.woff2'] = '580c6561987cd9c704bc20c808c2aa7543cbdd4eaca8ec334ae4985e61971333'
SHOWCASE_ASSETS['docs/preliminary-design/fonts/source-sans-3-latin-700-normal.woff2'] = '1fc65bcc7790d534cf67a5e046f3f324aad8cc6dc364ec5ad296a18d42868464'
SHOWCASE_ASSETS['docs/preliminary-design/releases/layout-001/review-scene.json.gz'] = '75d35b69a1397930e6c24f1c760fd75cb849698b3e17a1b7ee92c584fdbc598d'

def findings(data):
 text=data.decode('utf-8',errors='replace')
 return [name for name,p in PATTERNS.items() if re.search(p,text)]
def git(*args):return subprocess.check_output(['git','-C',str(ROOT),*args])
def committed_files(commit):
 records = git('ls-tree', '--full-tree', '-rz', commit).split(b'\0')
 with subprocess.Popen(
     ['git', '-C', str(ROOT), 'cat-file', '--batch'],
     stdin=subprocess.PIPE, stdout=subprocess.PIPE,
 ) as stream:
  try:
   for record in filter(None, records):
    header, path = record.split(b'\t', 1)
    _, kind, oid = header.split()
    if kind != b'blob':
     raise ValueError('Unexpected Git object for ' + path.decode())
    stream.stdin.write(oid + b'\n')
    stream.stdin.flush()
    actual_oid, actual_kind, size = stream.stdout.readline().split()
    if actual_oid != oid or actual_kind != kind:
     raise ValueError('Unexpected Git object in batch response')
    data = stream.stdout.read(int(size))
    if len(data) != int(size) or stream.stdout.read(1) != b'\n':
     raise ValueError('Incomplete Git blob')
    yield path.decode(), data
  finally:
   stream.stdin.close()
  if stream.wait():
   raise RuntimeError('Could not read the committed Git objects')
def audit(revision='HEAD'):
 commit=git('rev-parse','--verify','--end-of-options',revision+'^{commit}').decode().strip()
 failures=[]
 for path,data in committed_files(commit):
  if path in SHOWCASE_ASSETS:
   if hashlib.sha256(data).hexdigest()!=SHOWCASE_ASSETS[path]:failures.append((path,['unreviewed-cad-export']))
   continue
  if path==WRAPPER:
   if hashlib.sha256(data).hexdigest()!=WRAPPER_SHA:failures.append((path,['unreviewed-binary']))
   continue
  if b'\0' in data:failures.append((path,['unexpected-binary']));continue
  categories=findings(data)
  if categories:failures.append((path,categories))
 # Commit identity/message metadata, not source history rewriting.
 metadata=git('log','--format=%an <%ae> %cn <%ce> %s',commit)
 if findings(metadata):failures.append(('commit metadata',findings(metadata)))
 return failures
if __name__=='__main__':
 a=argparse.ArgumentParser();a.add_argument('--revision',default='HEAD');v=a.parse_args();errors=audit(v.revision)
 for path,categories in errors:print(path+': '+', '.join(categories))
 if errors:sys.exit(1)
 print('Public-content guard passed: committed text, binary allowlist and commit metadata.')
