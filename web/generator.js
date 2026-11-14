// TinyBVH custom header: runs the tinybvh_custom generator, compiled to WebAssembly in
// tinybvh_custom.js, on headers kept in the module's in-memory file system. Used by worker.js
// and, where a browser refuses workers (a page opened from disk), by index.html directly.
"use strict";

const TINYBVH_HEADERS = [ "tiny_bvh.h", "tiny_bvh_base.h", "tiny_bvh_x86_float.h",
	"tiny_bvh_x86_double.h", "tiny_bvh_arm_float.h", "tiny_bvh_arm_double.h" ];
const TINYBVH_OUT = "/out/tiny_bvh_custom.h";

// FS.stat throws for a missing file; FS.analyzePath, the other way to ask, is deprecated since
// Emscripten 6.0.11.
function exists( m, path )
{
	try { m.FS.stat( path ); return true; } catch (e) { return false; }
}

function createGeneratorRunner( create )
{
	let log = [];
	const ready = create( { print: s => log.push( s ), printErr: s => log.push( s ) } ).then( m =>
	{
		m.FS.mkdir( "/in" ), m.FS.mkdir( "/out" );
		return m;
	} );
	// request: { headers: { name: Uint8Array } or null to keep the current ones, args: [ ... ] }
	// result: { code, log, text }, where text is the generated header, or null.
	return async function run( request )
	{
		const m = await ready;
		if (request.headers) for (const name of TINYBVH_HEADERS) m.FS.writeFile( "/in/" + name, request.headers[name] );
		log = [];
		const code = m.ccall( "tinybvh_custom_run", "number", [ "string" ], [ request.args.join( "\n" ) ] );
		let text = null;
		if (code === 0 && exists( m, TINYBVH_OUT ))
			text = m.FS.readFile( TINYBVH_OUT, { encoding: "utf8" } ), m.FS.unlink( TINYBVH_OUT );
		return { code, log: log.join( "\n" ), text };
	};
}
