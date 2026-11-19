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
SHOWCASE_ASSETS = {'docs/robot/cad/assembly.png': '552351fad4fd747f866234827b64bf28361d9b1d5150f5c39c137beb640c2102', 'docs/robot/cad/assembly.stl.gz': 'c425fbde5b50ec9154d1a3f3019d5b41f6f04216f6225af21ac0a08fa8d15caf', 'experiments/swerve-drive/packaging/tab-saddle.f3d': 'c15bf7f1923c892e96f5081a75d30dd7fa3c1b01d3616abef9bf057f5abfb232'}

SHOWCASE_ASSETS['docs/robot/station-witness.png'] = '391f2d1fb95fda7c1a1352a9c34ed7ba1f5a9a3f550a942baa02a00ee4d41bec'
SHOWCASE_ASSETS['experiments/swerve-drive/motion/opening.webm'] = '5c78a3df9c463879e6b3507cbb14a009abb4482a8f94e90f71a428c88579d26c'

SHOWCASE_ASSETS['experiments/swerve-drive/motion/fallback.webm'] = '3186a0cdaba2a2d0f8a5798173f9086d3390b71e6a89313f3c2631026e6058f5'

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
