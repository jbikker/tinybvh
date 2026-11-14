// TinyBVH custom header: the generator, off the page's main thread. Messages in: { id, headers,
// args }; out: { id, code, log, text }, or { id, error } if the module itself failed.
"use strict";
importScripts( "tinybvh_custom.js", "generator.js" );
const run = createGeneratorRunner( createTinyBVHCustom );
onmessage = async e =>
{
	const { id, headers, args } = e.data;
	try { postMessage( { id, ...await run( { headers, args } ) } ); }
	catch (err) { postMessage( { id, error: String( err && err.message || err ) } ); }
};
