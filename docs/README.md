# TinyBVH custom header, on the web

A static page that does what `tinybvh_custom` does: choose the parts of TinyBVH a project uses,
and download a single header that contains only those. It runs the same generator, compiled to
WebAssembly, entirely in the browser; nothing is uploaded, and no server code is needed.

## Files

| File | What it is |
|---|---|
| `index.html` | The page. Its toggles come from the generator itself (`--describe`). |
| `tinybvh_custom.js` | The generator, compiled to WebAssembly; the module is embedded in this one file. |
| `generator.js` | Runs the generator on headers in the module's in-memory file system. |
| `worker.js` | Runs it in a background worker, so the page stays responsive. |
| `headers/` | The six TinyBVH headers the page generates from. |
| `build_web.sh` | Rebuilds `tinybvh_custom.js` and copies the headers into `headers/`. |

## Hosting

Any static web host works. For GitHub Pages, put this folder in the repository (for instance as
`custom/`), and enable Pages for the branch; the page is then at `.../custom/`.

To try it locally, serve the folder over HTTP, for example with `python -m http.server`, and open
`http://localhost:8000/`. Opened straight from disk, browsers refuse to load the bundled headers;
the page then says so, and can still generate from headers chosen with "Generate from your own copy
of TinyBVH".

## Updating

The generator's rules match the headers' exact text, so `tinybvh_custom.js` and `headers/` must
come from the same release of TinyBVH. After a change to `tinybvh_custom.cpp` or to the headers,
rebuild both, with Emscripten (emsdk) on the path:

    sh build_web.sh path/to/tinybvh_custom.cpp path/to/folder/with/the/headers

The checkboxes need no change: the page builds them from the generator's `--describe` output,
which lists the toggles, their labels, their layout and their dependencies.

## Browser support

The generator uses WebAssembly's own exception handling, to report malformed headers: Chrome and
Edge 95, Firefox 100, Safari 15.2, and later.
