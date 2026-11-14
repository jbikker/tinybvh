// TinyBVH custom header page: checks a built site before it is published. The WebAssembly
// generator, driven through the page's own generator.js, must produce exactly what the native
// generator produces from the same headers. The GitHub Actions workflow runs this before it
// deploys; it also runs by hand:
//
//   node check_web.cjs <built site folder> <native tinybvh_custom executable>
"use strict";
const fs = require( "fs" ), os = require( "os" ), path = require( "path" ), vm = require( "vm" );
const { execFileSync } = require( "child_process" );

const [ site, native ] = process.argv.slice( 2 );
if (!site || !native)
{
	console.error( "usage: node check_web.cjs <built site folder> <native tinybvh_custom executable>" );
	process.exit( 2 );
}

// The page's generator.js, loaded as the page loads it: as a classic script.
vm.runInThisContext( fs.readFileSync( path.join( site, "generator.js" ), "utf8" ), { filename: "generator.js" } );
const HEADERS = vm.runInThisContext( "TINYBVH_HEADERS" );
// The module, through a .cjs copy, so that a package.json in a parent folder cannot change how
// Node reads it.
const tmp = fs.mkdtempSync( path.join( os.tmpdir(), "tinybvh-web-" ) );
fs.copyFileSync( path.join( site, "tinybvh_custom.js" ), path.join( tmp, "tinybvh_custom.cjs" ) );
const run = createGeneratorRunner( require( path.join( tmp, "tinybvh_custom.cjs" ) ) );

const OUT = [ "--in", "/in", "--out", "/out/tiny_bvh_custom.h" ];
const CONFIGURATIONS = [
	[],
	[ "--no-all-layouts" ],
	[ "--no-x86", "--no-templates" ],
	[ "--no-optimizer", "--no-mergeleafs", "--no-threading" ],
	[ "--no-arm", "--no-presplitting", "--no-voxelset", "--no-fullsweep", "--no-hploc" ],
];
let failures = 0;
const fail = what => { console.log( "FAIL " + what ); failures++; };

( async () =>
{
	for (const f of [ "index.html", "worker.js", "generator.js", "tinybvh_custom.js", ...HEADERS.map( h => "headers/" + h ) ])
		if (!fs.existsSync( path.join( site, f ) )) fail( "missing " + f );
	const headers = {};
	for (const h of HEADERS) headers[h] = new Uint8Array( fs.readFileSync( path.join( site, "headers", h ) ) );

	const d = await run( { headers: null, args: [ "--describe" ] } );
	let description = null;
	try { description = JSON.parse( d.log ); } catch (e) { description = null; }
	if (d.code !== 0 || !description || !description.toggles || !description.toggles.length) fail( "--describe gave no usable description" );
	else console.log( `--describe: ${description.toggles.length} toggles in ${description.groups.length} groups` );

	let sent = false;
	for (const c of CONFIGURATIONS)
	{
		const label = c.join( " " ) || "everything";
		const t0 = Date.now();
		const r = await run( { headers: sent ? null : headers, args: [ ...OUT, ...c ] } );
		const ms = Date.now() - t0;
		sent = true;
		const expected = path.join( tmp, "native.h" );
		execFileSync( native, [ "--in", path.join( site, "headers" ), "--out", expected, ...c ], { stdio: "pipe" } );
		if (r.code !== 0 || r.text !== fs.readFileSync( expected, "utf8" )) fail( `${label}: differs from the native generator` );
		else console.log( `${label}: identical to the native generator, ${r.text.length} bytes, ${ms} ms` );
	}

	// A malformed header must be reported, and must not break the module for the next run.
	const broken = await run( { headers: { ...headers, "tiny_bvh.h": new TextEncoder().encode( "not a header\n" ) }, args: OUT } );
	if (broken.code === 0 || !/^error/m.test( broken.log )) fail( "a malformed header was not reported as an error" );
	else console.log( "malformed header: reported as an error" );
	const after = await run( { headers, args: OUT } );
	if (after.code !== 0 || after.text === null) fail( "the generator did not recover after the malformed header" );

	console.log( failures ? `${failures} check(s) failed` : "all checks passed" );
	process.exit( failures ? 1 : 0 );
} )().catch( e => { console.log( "FAIL", e ); process.exit( 1 ); } );
