// Publish only the checked field rehearsals into a fresh Pages directory.
import {createHash} from 'node:crypto';
import {mkdir, readFile, writeFile} from 'node:fs/promises';
import path from 'node:path';

const [project, output] = process.argv.slice(2);
await mkdir(output);
const read = name => readFile(path.join(project, name));
const field = JSON.parse(await read('highlights.json')).fieldLab;
const roleDir = 'experiments/swerve-drive/whole-robot-trade/';
const role = JSON.parse(await read(roleDir + 'role-publication.json'));
for (const [source, destination, hash] of [
  [field.file, 'field-lab.html', field.sha256],
  [roleDir + 'role-lab.html', 'role-lab.html', role.htmlSha256],
]) {
  const bytes = await read(source);
  if (createHash('sha256').update(bytes).digest('hex') !== hash) throw Error('Changed lab: ' + source);
  await writeFile(path.join(output, destination), bytes);
}
await writeFile(path.join(output, 'index.html'), await read('kickoff/index.html'));
