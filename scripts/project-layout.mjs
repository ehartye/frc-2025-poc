import {execFileSync} from 'node:child_process';
import {existsSync, mkdirSync, readFileSync, statSync} from 'node:fs';
import path from 'node:path';
import {pathToFileURL} from 'node:url';

export const folders = ['kickoff','cad','controls','sim','experiments','tests','scripts','docs','artifacts'];
const files = new Set(['README.md','LICENSE','LICENSE.md','AGENTS.md','.gitattributes','.gitignore',
  'run.ps1','cad-showcase.json','project-layout.json','package.json','package-lock.json','pyproject.toml',
  'requirements.txt','pytest.ini','settings.gradle','build.gradle','gradlew','gradlew.bat']);
const toolFolders = new Set(['.github','.vscode','.wpilib','src','gradle','vendordeps']);

export function inspect(root) {
  const configFile = path.join(root,'project-layout.json');
  const config = existsSync(configFile) ? JSON.parse(readFileSync(configFile,'utf8')) : {};
  // Permit individual toolchain paths. New top-level experiment folders require migration.
  const exceptions = new Set(config.toolchainFiles ?? []);
  const paths = execFileSync('git',['-C',root,'ls-files','-co','--exclude-standard','-z'],
    {encoding:'utf8', maxBuffer:64*1024*1024}).split('\0').filter(Boolean);
  const problems = new Set();
  for (const file of paths) {
    const [first] = file.split('/');
    if (first === 'artifacts') problems.add('Generated artifact in project inventory: ' + file);
    else if (file.includes('/') ? !folders.includes(first) && !toolFolders.has(first) && !exceptions.has(file)
      : !files.has(file) && !exceptions.has(file)) problems.add('Unexpected root entry: ' + first);
  }
  return [...problems].sort();
}

if (process.argv[1] && import.meta.url === pathToFileURL(path.resolve(process.argv[1])).href) {
  const [mode, directory] = process.argv.slice(2);
  if (!['init','check'].includes(mode) || !directory) throw Error('Usage: project-layout.mjs init|check PROJECT');
  const root = path.resolve(directory);
  const checkout = execFileSync('git',['-C',root,'rev-parse','--show-toplevel'],{encoding:'utf8'}).trim();
  if (path.relative(root,checkout) || !statSync(root).isDirectory()) throw Error('Select the project checkout root');
  if (mode === 'init') for (const name of folders) mkdirSync(path.join(root,name),{recursive:true});
  const issues = inspect(root);
  if (issues.length) { console.error(issues.join('\n')); process.exitCode = 1; }
  else console.log('Project layout passed.');
}

