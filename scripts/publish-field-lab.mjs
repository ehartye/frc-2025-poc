import { copyFile, readFile, writeFile } from 'node:fs/promises';
import { resolve } from 'node:path';
import { fileURLToPath } from 'node:url';

if (process.argv.length !== 3) throw Error('Supply the built showcase directory.');
const out = resolve(process.argv[2]);
const source = fileURLToPath(new URL('../kickoff/field-lab.html', import.meta.url));
const path = resolve(out, 'index.html');
const html = await readFile(path, 'utf8');
const nav = /<nav\b[^>]*>[\s\S]*?<\/nav>/ig;
const [primary, ...others] = html.match(nav) ?? [];
if (!primary || others.length) throw Error('Expected a single showcase navigation.');
const link = '<a href="field-lab.html">Open the field lab</a>';
await copyFile(source, resolve(out, 'field-lab.html'));
if (!primary.includes(link)) {
  const updated = primary.replace(/^(<nav\b[^>]*>)/i, '$1' + link);
  await writeFile(path, html.replace(primary, updated));
}
