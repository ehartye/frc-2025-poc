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
# Only the upstream Apache-licensed Gradle wrapper binary is permitted.
WRAPPER='experiments/six-wheel/shared/examples/java-2026-sim/gradle/wrapper/gradle-wrapper.jar'
WRAPPER_SHA='2db75c40782f5e8ba1fc278a5574bab070adccb2d21ca5a6e5ed840888448046'
SHOWCASE_ASSETS = {'docs/robot/cad/assembly.png': '2d83b0cf0419e0a9a9f4df0248e38e45bb837379bca7b47f30468918d3839649', 'docs/robot/cad/assembly.stl.gz': '95bcf37c771eb2e5ba4b9b5ec17351fe20031d0b08ceea27cbf74bfa6eae7d31'}

def findings(data):
 text=data.decode('utf-8',errors='replace')
 return [name for name,p in PATTERNS.items() if re.search(p,text)]
def git(*args):return subprocess.check_output(['git','-C',str(ROOT),*args])
def committed_files(commit):
 records = git('ls-tree', '-rz', commit).split(b'\0')
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
